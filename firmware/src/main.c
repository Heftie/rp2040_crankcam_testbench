// Crank/cam testbench firmware. One binary, three modes selected from a
// boot-time menu over USB serial (no reflashing needed to switch):
//   1) continuous double-buffered generation, RPM profile   (normal operation)
//   2) single-shot 2-rev diagnostic, constant RPM           (scope bring-up)
//   3) one-shot gen + 6ch capture + angle + pass/fail        (acquisition test)
//
// A selected mode runs forever (matches how each was validated standalone
// on real hardware) -- to pick a different mode, reset/replug the board.

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"

#include "event_gen.pio.h"
#include "capture.pio.h"

// ===================== Shared crank/cam config =====================

#define CRANK_PIN 2
#define CAM_PIN 3

#define TEETH_PER_REV 60
#define MISSING_TEETH 2
#define REAL_TEETH_PER_REV (TEETH_PER_REV - MISSING_TEETH) // 58
#define REVS_PER_CYCLE 2
#define POSITIONS_PER_REV TEETH_PER_REV                      // 60
#define POSITIONS_TOTAL (POSITIONS_PER_REV * REVS_PER_CYCLE) // 120
#define DEG_PER_POSITION (360.0 / POSITIONS_PER_REV)         // 6 deg

#define CRANK_EVENTS_PER_REV (REAL_TEETH_PER_REV * 2 + 1) // 116 edges + 1 gap = 117
#define CRANK_EVENTS_TOTAL (CRANK_EVENTS_PER_REV * REVS_PER_CYCLE) // 234
#define CRANK_WORDS_TOTAL (CRANK_EVENTS_TOTAL * 2) // 468

// Cam: one pulse per 720deg cycle, 120-300deg, aligned to positions 20/50
// (120/6=20, 300/6=50) so it reuses crank's own per-position cycle counts.
#define CAM_RISE_POSITION 20
#define CAM_FALL_POSITION 50
#define CAM_EVENTS_TOTAL 3
#define CAM_WORDS_TOTAL (CAM_EVENTS_TOTAL * 2)

#define EVENT_MIN_CYCLES 3 // 2 `out` + at least 1 delay_loop iteration
#define NUM_BUFFERS 2
#define CONSTANT_RPM 1000.0 // used by modes 2 and 3

static uint32_t crank_events[NUM_BUFFERS][CRANK_WORDS_TOTAL];
static uint32_t cam_events[NUM_BUFFERS][CAM_WORDS_TOTAL];
static uint32_t position_cycles[POSITIONS_TOTAL];
static double f_pio_hz;

static void push_event(uint32_t *buf, uint *idx, uint32_t state, uint32_t desired_cycles) {
    // event_gen.pio: hold = 2 (`out`x2) + (D+1) (`jmp x--` loop) cycles.
    assert(desired_cycles >= EVENT_MIN_CYCLES);
    buf[(*idx)++] = state;
    buf[(*idx)++] = desired_cycles - EVENT_MIN_CYCLES;
}

static void build_crank_events(uint32_t *buf) {
    uint idx = 0;
    for (uint rev = 0; rev < REVS_PER_CYCLE; rev++) {
        uint32_t gap_cycles = 0;
        for (uint pos = 0; pos < POSITIONS_PER_REV; pos++) {
            uint32_t c = position_cycles[rev * POSITIONS_PER_REV + pos];
            if (pos < REAL_TEETH_PER_REV) {
                uint32_t half = c / 2;
                push_event(buf, &idx, 1, half);
                push_event(buf, &idx, 0, c - half);
            } else {
                gap_cycles += c; // missing-tooth position, no edge
            }
        }
        push_event(buf, &idx, 0, gap_cycles);
    }
}

static uint32_t sum_position_cycles(uint from, uint to_exclusive) {
    uint32_t sum = 0;
    for (uint pos = from; pos < to_exclusive; pos++) {
        sum += position_cycles[pos];
    }
    return sum;
}

static void build_cam_events(uint32_t *buf) {
    uint idx = 0;
    push_event(buf, &idx, 0, sum_position_cycles(0, CAM_RISE_POSITION));
    push_event(buf, &idx, 1, sum_position_cycles(CAM_RISE_POSITION, CAM_FALL_POSITION));
    push_event(buf, &idx, 0, sum_position_cycles(CAM_FALL_POSITION, POSITIONS_TOTAL));
}

// Fills position_cycles[] with a single constant value (modes 2 and 3:
// no RPM ramp, simplest possible signal for scope/capture validation).
static void build_position_cycles_constant(double rpm) {
    uint32_t c = (uint32_t)(60.0 * f_pio_hz / (rpm * TEETH_PER_REV) + 0.5);
    for (uint pos = 0; pos < POSITIONS_TOTAL; pos++) {
        position_cycles[pos] = c;
    }
}

// =============== Mode 1: continuous double-buffered generation ===============

// Section 3 profile: desired RPM at 0/180/360/540/720 deg crank angle.
typedef struct {
    double angle_deg;
    double rpm;
} RpmPoint;

static const RpmPoint base_rpm_profile[] = {
    {0.0, 1000.0},
    {180.0, 1500.0},
    {360.0, 2500.0},
    {540.0, 3500.0},
    {720.0, 4000.0},
};
#define RPM_PROFILE_POINTS (sizeof(base_rpm_profile) / sizeof(base_rpm_profile[0]))

// Double buffering demo: alternate between two RPM-profile scales each
// 720deg cycle, so buffer content visibly changes while the other buffer
// plays. The switch happens only at a 720deg cycle boundary (not
// mid-cycle), matching doc section 11's "no physically impossible
// instantaneous changes" -- a full-cycle RPM step is a normal test move.
static const double cycle_rpm_scale[2] = {1.0, 1.15};

static uint dma_crank_chan[NUM_BUFFERS];
static uint dma_cam_chan[NUM_BUFFERS];
static volatile uint32_t cycles_completed = 0;

static double interpolate_rpm(double angle_deg, double scale) {
    if (angle_deg <= base_rpm_profile[0].angle_deg) {
        return base_rpm_profile[0].rpm * scale;
    }
    for (uint i = 0; i + 1 < RPM_PROFILE_POINTS; i++) {
        const RpmPoint *a = &base_rpm_profile[i];
        const RpmPoint *b = &base_rpm_profile[i + 1];
        if (angle_deg <= b->angle_deg) {
            double frac = (angle_deg - a->angle_deg) / (b->angle_deg - a->angle_deg);
            return (a->rpm + frac * (b->rpm - a->rpm)) * scale;
        }
    }
    return base_rpm_profile[RPM_PROFILE_POINTS - 1].rpm * scale;
}

// Delta t_i = 60 / (RPM_i * N); C_i = Delta t_i * f_PIO (doc section 3).
static uint32_t cycles_for_position(double angle_deg, double scale) {
    double rpm = interpolate_rpm(angle_deg, scale);
    double dt = 60.0 / (rpm * TEETH_PER_REV);
    double cycles = dt * f_pio_hz;
    return (uint32_t)(cycles + 0.5);
}

static void build_position_cycles_profile(double scale) {
    for (uint pos = 0; pos < POSITIONS_TOTAL; pos++) {
        double angle = pos * DEG_PER_POSITION;
        position_cycles[pos] = cycles_for_position(angle, scale);
    }
}

// Recomputes both crank_events[slot] and cam_events[slot] from one shared
// position_cycles[] pass, keeping crank/cam phase-locked by construction.
static void fill_buffer_slot(uint slot, double scale) {
    build_position_cycles_profile(scale);
    build_crank_events(crank_events[slot]);
    build_cam_events(cam_events[slot]);
}

// Configures (but does not start) a DMA channel to stream `buf` to the
// SM's TX FIFO once, then chain-trigger `chain_to_chan` (the other
// buffer's channel) on completion. Both channels of a pair must already
// be claimed so each can name the other as its chain target.
static void configure_ping_pong_channel(PIO pio, uint sm, uint32_t *buf, uint32_t word_count,
                                         uint this_chan, uint chain_to_chan) {
    dma_channel_config cfg = dma_channel_get_default_config(this_chan);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&cfg, true);
    channel_config_set_write_increment(&cfg, false);
    channel_config_set_dreq(&cfg, pio_get_dreq(pio, sm, true));
    channel_config_set_chain_to(&cfg, chain_to_chan);
    dma_channel_configure(this_chan, &cfg, &pio->txf[sm], buf, word_count, false);
}

// Fires when a crank buffer finishes playing (DMA has just chained to the
// other crank buffer). The buffer that just finished is safe to refill --
// it will not be read again until the other buffer finishes its own turn,
// a full 720deg cycle away, which is orders of magnitude longer than the
// microseconds this refill takes.
static void dma_irq_handler(void) {
    for (uint slot = 0; slot < NUM_BUFFERS; slot++) {
        uint chan = dma_crank_chan[slot];
        if (dma_hw->ints0 & (1u << chan)) {
            dma_hw->ints0 = 1u << chan;
            cycles_completed++;
            double scale = cycle_rpm_scale[cycles_completed % 2];
            fill_buffer_slot(slot, scale);
        }
    }
}

static void run_mode_continuous(void) {
    PIO pio = pio0;
    uint sm_crank = pio_claim_unused_sm(pio, true);
    uint sm_cam = pio_claim_unused_sm(pio, true);
    uint offset = pio_add_program(pio, &event_gen_program);

    // Run PIO at full system clock so C_i = Delta t_i * f_PIO cycle
    // counts are computed directly against f_PIO with no extra scaling.
    float clkdiv = 1.0f;
    f_pio_hz = (double)clock_get_hz(clk_sys) / clkdiv;

    fill_buffer_slot(0, cycle_rpm_scale[0]);
    fill_buffer_slot(1, cycle_rpm_scale[1]);

    event_gen_program_init(pio, sm_crank, offset, CRANK_PIN, clkdiv);
    event_gen_program_init(pio, sm_cam, offset, CAM_PIN, clkdiv);

    // Claim both channels of each pair first so each can name the other
    // as its chain_to target, then configure both (armed, not started).
    dma_crank_chan[0] = dma_claim_unused_channel(true);
    dma_crank_chan[1] = dma_claim_unused_channel(true);
    configure_ping_pong_channel(pio, sm_crank, crank_events[0], CRANK_WORDS_TOTAL,
                                 dma_crank_chan[0], dma_crank_chan[1]);
    configure_ping_pong_channel(pio, sm_crank, crank_events[1], CRANK_WORDS_TOTAL,
                                 dma_crank_chan[1], dma_crank_chan[0]);

    dma_cam_chan[0] = dma_claim_unused_channel(true);
    dma_cam_chan[1] = dma_claim_unused_channel(true);
    configure_ping_pong_channel(pio, sm_cam, cam_events[0], CAM_WORDS_TOTAL,
                                 dma_cam_chan[0], dma_cam_chan[1]);
    configure_ping_pong_channel(pio, sm_cam, cam_events[1], CAM_WORDS_TOTAL,
                                 dma_cam_chan[1], dma_cam_chan[0]);

    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);
    dma_channel_set_irq0_enabled(dma_crank_chan[0], true);
    dma_channel_set_irq0_enabled(dma_crank_chan[1], true);
    irq_set_enabled(DMA_IRQ_0, true);

    // Start both SMs on the same clock cycle so crank and cam stay
    // phase-locked, then trigger both slot-0 DMA channels together so
    // crank/cam begin their first 720deg cycle in sync.
    dma_channel_start(dma_crank_chan[0]);
    dma_channel_start(dma_cam_chan[0]);
    pio_enable_sm_mask_in_sync(pio, (1u << sm_crank) | (1u << sm_cam));

    printf("event_gen double-buffered: crank pin=%d, cam pin=%d, f_pio=%.0f Hz\n",
           CRANK_PIN, CAM_PIN, f_pio_hz);

    while (true) {
        tight_loop_contents();
    }
}

// =============== Mode 2: single-shot 2-rev diagnostic ===============

// Resets both SMs/DMA to a clean state, rebuilds crank_events[0]/
// cam_events[0] from the current position_cycles[], and fires them
// phase-locked. Shared by modes 2 and 3 (same one-shot generator).
static void fire_gen_one_shot(PIO pio, uint sm_crank, uint sm_cam, uint offset, float clkdiv,
                               uint dma_crank_chan_, uint dma_cam_chan_) {
    // Clean up any previous shot first: by now it has long since finished
    // and stalled on its own (see note below), so this is a harmless
    // reset, not a mid-flight interruption.
    pio_sm_set_enabled(pio, sm_crank, false);
    pio_sm_set_enabled(pio, sm_cam, false);
    dma_channel_abort(dma_crank_chan_);
    dma_channel_abort(dma_cam_chan_);
    pio_sm_clear_fifos(pio, sm_crank);
    pio_sm_clear_fifos(pio, sm_cam);

    // Fully re-init both SMs: resets PC to program start and clears
    // OSR/X/Y, so every shot starts from the same clean state.
    event_gen_program_init(pio, sm_crank, offset, CRANK_PIN, clkdiv);
    event_gen_program_init(pio, sm_cam, offset, CAM_PIN, clkdiv);

    dma_channel_config cfg_crank = dma_channel_get_default_config(dma_crank_chan_);
    channel_config_set_transfer_data_size(&cfg_crank, DMA_SIZE_32);
    channel_config_set_read_increment(&cfg_crank, true);
    channel_config_set_write_increment(&cfg_crank, false);
    channel_config_set_dreq(&cfg_crank, pio_get_dreq(pio, sm_crank, true));
    dma_channel_configure(dma_crank_chan_, &cfg_crank, &pio->txf[sm_crank],
                           crank_events[0], CRANK_WORDS_TOTAL, false);

    dma_channel_config cfg_cam = dma_channel_get_default_config(dma_cam_chan_);
    channel_config_set_transfer_data_size(&cfg_cam, DMA_SIZE_32);
    channel_config_set_read_increment(&cfg_cam, true);
    channel_config_set_write_increment(&cfg_cam, false);
    channel_config_set_dreq(&cfg_cam, pio_get_dreq(pio, sm_cam, true));
    dma_channel_configure(dma_cam_chan_, &cfg_cam, &pio->txf[sm_cam],
                           cam_events[0], CAM_WORDS_TOTAL, false);

    dma_channel_start(dma_crank_chan_);
    dma_channel_start(dma_cam_chan_);

    // Same clock cycle start so crank/cam stay phase-locked, as before.
    pio_enable_sm_mask_in_sync(pio, (1u << sm_crank) | (1u << sm_cam));

    // Deliberately not blocking/disabling here: dma_channel_wait_for_finish
    // only confirms DMA finished *writing* words into the FIFO, not that
    // the SM finished *playing* them (FIFO depth 8 = up to 4 events still
    // queued). Disabling the SM right after that wait would freeze it
    // mid-event. Instead just let it run free -- the SM stalls itself
    // cleanly once the buffer runs dry, which only happens once every
    // event's full delay has actually played out on the pin.
}

static void run_mode_singleshot(void) {
    PIO pio = pio0;
    uint sm_crank = pio_claim_unused_sm(pio, true);
    uint sm_cam = pio_claim_unused_sm(pio, true);
    uint offset = pio_add_program(pio, &event_gen_program);

    float clkdiv = 1.0f;
    f_pio_hz = (double)clock_get_hz(clk_sys) / clkdiv;

    build_position_cycles_constant(CONSTANT_RPM);
    build_crank_events(crank_events[0]);
    build_cam_events(cam_events[0]);

    uint dma_crank_chan_ = dma_claim_unused_channel(true);
    uint dma_cam_chan_ = dma_claim_unused_channel(true);

    double total_ms = (double)position_cycles[0] * POSITIONS_TOTAL / f_pio_hz * 1000.0;
    printf("singleshot: crank pin=%d, cam pin=%d, const RPM=%.0f, "
           "cycle=%.2f ms (arm scope SINGLE, then press Enter to fire)\n",
           CRANK_PIN, CAM_PIN, CONSTANT_RPM, total_ms);

    while (true) {
        int ch = getchar(); // blocks until a byte arrives over USB serial
        (void)ch;
        fire_gen_one_shot(pio, sm_crank, sm_cam, offset, clkdiv, dma_crank_chan_, dma_cam_chan_);
        // Runs free in hardware from here -- not blocking on completion
        // (see fire_gen_one_shot). Press Enter again any time after the
        // cycle has had time to finish (~total_ms) for a clean re-fire.
        printf("fired, runs for %.0f ms then holds. Press Enter for next shot.\n", total_ms);
    }
}

// =============== Mode 3: capture test (gen + 6ch capture + angle + pass/fail) ===============

#define CAPTURE_BASE_PIN 6
#define CAPTURE_PIN_COUNT 6
#define CAPTURE_SAMPLE_HZ 100000.0
#define CAPTURE_DURATION_MS 150.0
// Plain integer literal (not computed via a float macro): a float-cast
// expression here previously made GCC treat capture_buf as a "variably
// modified" array at file scope (UB for a static array), which silently
// crashed at boot. Keep in sync with CAPTURE_SAMPLE_HZ*CAPTURE_DURATION_MS/1000.
#define CAPTURE_SAMPLES 15000u

static uint32_t capture_buf[CAPTURE_SAMPLES];

// Collects up to max_out timestamps (ms) of edges on `ch` matching
// want_rising. Returns the count found (may exceed max_out; only the
// first max_out are stored).
static uint find_edge_times(uint ch, bool want_rising, double sample_period_ms,
                             double *out, uint max_out) {
    uint32_t prev = capture_buf[0] & (1u << ch);
    uint count = 0;
    for (uint32_t i = 1; i < CAPTURE_SAMPLES; i++) {
        uint32_t cur = capture_buf[i] & (1u << ch);
        if (cur != prev) {
            bool is_rising = (cur != 0);
            if (is_rising == want_rising && count < max_out) {
                out[count] = i * sample_period_ms;
            }
            if (is_rising == want_rising) count++;
            prev = cur;
        }
    }
    return count;
}

static void print_channel_edges(uint ch, double sample_period_ms) {
    uint32_t prev = capture_buf[0] & (1u << ch);
    uint rising = 0, falling = 0;
    uint printed = 0;
    printf("  ch%u: ", ch);
    for (uint32_t i = 1; i < CAPTURE_SAMPLES; i++) {
        uint32_t cur = capture_buf[i] & (1u << ch);
        if (cur != prev) {
            bool is_rising = (cur != 0);
            if (is_rising) rising++; else falling++;
            if (printed < 4) {
                printf("%s@%.3fms ", is_rising ? "R" : "F", i * sample_period_ms);
                printed++;
            }
            prev = cur;
        }
    }
    printf("(rising=%u falling=%u)\n", rising, falling);
}

#define MAX_CRANK_RISING 200
#define MAX_REFS 8
#define MAX_CAM_EDGES 4

// Doc section 8: reference = first crank tooth after the missing-tooth
// gap = angle 0deg. C_rev = reference-to-reference duration (one full
// revolution, gap included). angle = 360 * C_event / C_rev, using each
// revolution's own measured C_rev rather than one fixed factor, so a
// constant-vs-varying RPM profile is handled the same way.
//
// This test's capture window starts exactly at generation's own start
// (tooth 1 of rev 0), so refs[0] = the very first crank rising edge, not
// a detected gap. Every later reference comes from finding the
// rising-to-rising interval that's a clear outlier (~3x a normal tooth
// period at the gap) vs. the smallest observed interval (~1 normal
// tooth). The last revolution in a one-shot capture has no trailing
// reference (generation just stops there, no further gap) -- for that
// window this falls back to reusing the previous revolution's C_rev,
// noted explicitly below; a continuous double-buffered capture would not
// have this edge case.
static uint find_crank_references(const double *rising, uint n, double *refs, uint max_refs) {
    if (n < 2 || max_refs == 0) return 0;
    double min_interval = rising[1] - rising[0];
    for (uint i = 1; i + 1 < n; i++) {
        double iv = rising[i + 1] - rising[i];
        if (iv < min_interval) min_interval = iv;
    }
    uint count = 0;
    refs[count++] = rising[0];
    for (uint i = 0; i + 1 < n && count < max_refs; i++) {
        double iv = rising[i + 1] - rising[i];
        if (iv > 2.0 * min_interval) {
            refs[count++] = rising[i + 1];
        }
    }
    return count;
}

// Converts timestamp t (ms) to an absolute crank angle (deg) using the
// reference windows in refs[0..n_refs-1]. rev0_window is the index of
// the window that contains the cam pulse (angle 0-360deg); other
// windows are numbered relative to it. Falls back to reusing the last
// full window's C_rev for a trailing timestamp with no closing
// reference (see find_crank_references' header comment).
static double convert_to_angle(double t, const double *refs, uint n_refs, int rev0_window) {
    for (uint w = 0; w + 1 < n_refs; w++) {
        bool in_window = (t >= refs[w] && t < refs[w + 1]);
        bool in_trailing = (w + 2 == n_refs && t >= refs[w + 1]);
        if (in_window || in_trailing) {
            double c_rev = refs[w + 1] - refs[w];
            double c_event = t - refs[w];
            double angle_in_rev = 360.0 * c_event / c_rev;
            int rev_index = (rev0_window >= 0) ? ((int)w - rev0_window) : 0;
            return angle_in_rev + rev_index * 360.0;
        }
    }
    return -1.0; // before refs[0] or no windows at all
}

// Computes the crank reference points and identifies which window is
// rev0 (the one containing the cam pulse). Shared by the angle-print
// demo and the pass/fail evaluator so both work from the same data.
// Returns false if there weren't enough crank edges to get references.
static bool compute_crank_reference(double sample_period_ms, double *refs, uint *n_refs_out,
                                     int *rev0_window_out) {
    static double crank_rising[MAX_CRANK_RISING];
    uint n_rising = find_edge_times(0, true, sample_period_ms, crank_rising, MAX_CRANK_RISING);
    if (n_rising > MAX_CRANK_RISING) n_rising = MAX_CRANK_RISING;
    uint n_refs = find_crank_references(crank_rising, n_rising, refs, MAX_REFS);

    printf("crank references (angle=0deg points) found: %u\n", n_refs);
    for (uint i = 0; i < n_refs; i++) {
        printf("  ref[%u] @ %.3fms\n", i, refs[i]);
    }
    if (n_refs < 2) {
        printf("  not enough references to compute C_rev.\n");
        *n_refs_out = n_refs;
        *rev0_window_out = -1;
        return false;
    }

    double cam_rise[MAX_CAM_EDGES];
    uint n_cam_rise = find_edge_times(1, true, sample_period_ms, cam_rise, MAX_CAM_EDGES);
    if (n_cam_rise > MAX_CAM_EDGES) n_cam_rise = MAX_CAM_EDGES;

    // Which reference window contains the cam pulse identifies rev 0
    // (cam is silent through rev 1 by design).
    int rev0_window = -1;
    for (uint w = 0; w + 1 < n_refs && rev0_window < 0; w++) {
        for (uint e = 0; e < n_cam_rise; e++) {
            if (cam_rise[e] >= refs[w] && cam_rise[e] < refs[w + 1]) {
                rev0_window = (int)w;
                break;
            }
        }
    }
    printf("cam pulse found in reference window %d (that window = rev0/0-360deg)\n", rev0_window);

    *n_refs_out = n_refs;
    *rev0_window_out = rev0_window;
    return true;
}

static void angle_conversion_demo(double sample_period_ms, const double *refs, uint n_refs,
                                   int rev0_window) {
    double cam_rise[MAX_CAM_EDGES], cam_fall[MAX_CAM_EDGES];
    uint n_cam_rise = find_edge_times(1, true, sample_period_ms, cam_rise, MAX_CAM_EDGES);
    uint n_cam_fall = find_edge_times(1, false, sample_period_ms, cam_fall, MAX_CAM_EDGES);
    if (n_cam_rise > MAX_CAM_EDGES) n_cam_rise = MAX_CAM_EDGES;
    if (n_cam_fall > MAX_CAM_EDGES) n_cam_fall = MAX_CAM_EDGES;

    printf("cam angles (expect ~120deg rise, ~300deg fall, matching doc section 9):\n");
    for (uint e = 0; e < n_cam_rise; e++) {
        double angle = convert_to_angle(cam_rise[e], refs, n_refs, rev0_window);
        printf("  rise@%.3fms -> %.2f deg\n", cam_rise[e], angle);
    }
    for (uint e = 0; e < n_cam_fall; e++) {
        double angle = convert_to_angle(cam_fall[e], refs, n_refs, rev0_window);
        printf("  fall@%.3fms -> %.2f deg\n", cam_fall[e], angle);
    }
}

// Step 10: doc section 9's expected-window pass/fail evaluation. An ECU
// output is a rise-then-fall pulse at known angles, in a known 720deg
// revolution, within a tolerance.
typedef struct {
    const char *name;
    uint channel;
    double rise_deg, rise_tol_deg;
    double fall_deg, fall_tol_deg;
    int expected_cycle; // 0 or 1
} EcuOutputSpec;

#define MAX_EDGES_PER_CHECK 8

// Angle-wraps a possibly-negative or out-of-range revolution index back
// into [0, 360) by construction: convert_to_angle already returns
// rev_index*360 + angle_in_rev, so recovering rev_index is just floor/360.
static int angle_to_rev_index(double angle) {
    return (angle >= 0.0) ? (int)(angle / 360.0) : -1;
}

// Among angles[0..n), returns the index of the one in [cycle*360,
// cycle*360+360) closest to target_deg, or -1 if none are in that cycle.
static int pick_in_cycle(const double *angles, uint n, int cycle, double target_deg) {
    int best = -1;
    double best_diff = 1e18;
    for (uint i = 0; i < n; i++) {
        if (angle_to_rev_index(angles[i]) != cycle) continue;
        double diff = angles[i] - target_deg;
        if (diff < 0) diff = -diff;
        if (diff < best_diff) {
            best_diff = diff;
            best = (int)i;
        }
    }
    return best;
}

static uint count_in_cycle(const double *angles, uint n, int cycle) {
    uint c = 0;
    for (uint i = 0; i < n; i++) {
        if (angle_to_rev_index(angles[i]) == cycle) c++;
    }
    return c;
}

static void evaluate_spec(const EcuOutputSpec *spec, double sample_period_ms,
                           const double *refs, uint n_refs, int rev0_window) {
    double rise_t[MAX_EDGES_PER_CHECK], fall_t[MAX_EDGES_PER_CHECK];
    uint n_rise = find_edge_times(spec->channel, true, sample_period_ms, rise_t, MAX_EDGES_PER_CHECK);
    uint n_fall = find_edge_times(spec->channel, false, sample_period_ms, fall_t, MAX_EDGES_PER_CHECK);
    if (n_rise > MAX_EDGES_PER_CHECK) n_rise = MAX_EDGES_PER_CHECK;
    if (n_fall > MAX_EDGES_PER_CHECK) n_fall = MAX_EDGES_PER_CHECK;

    double rise_a[MAX_EDGES_PER_CHECK], fall_a[MAX_EDGES_PER_CHECK];
    for (uint i = 0; i < n_rise; i++) rise_a[i] = convert_to_angle(rise_t[i], refs, n_refs, rev0_window);
    for (uint i = 0; i < n_fall; i++) fall_a[i] = convert_to_angle(fall_t[i], refs, n_refs, rev0_window);

    printf("[%s] ch%u expect rise=%.1f+-%.1fdeg fall=%.1f+-%.1fdeg cycle=%d:\n",
           spec->name, spec->channel, spec->rise_deg, spec->rise_tol_deg,
           spec->fall_deg, spec->fall_tol_deg, spec->expected_cycle);

    bool pass = true;

    int ri = pick_in_cycle(rise_a, n_rise, spec->expected_cycle, spec->rise_deg);
    if (ri < 0) {
        uint elsewhere = n_rise - count_in_cycle(rise_a, n_rise, spec->expected_cycle);
        if (n_rise == 0) {
            printf("  FAIL: missing rising edge\n");
        } else if (elsewhere > 0) {
            printf("  FAIL: rising edge active in wrong revolution (expected cycle %d)\n",
                   spec->expected_cycle);
        } else {
            printf("  FAIL: missing rising edge in expected cycle\n");
        }
        pass = false;
    } else {
        double err = rise_a[ri] - spec->rise_deg;
        if (err < 0) err = -err;
        if (err > spec->rise_tol_deg) {
            printf("  FAIL: rising edge at %.2fdeg, out of tolerance\n", rise_a[ri]);
            pass = false;
        }
        if (count_in_cycle(rise_a, n_rise, spec->expected_cycle) > 1) {
            printf("  FAIL: extra rising edge(s) in expected cycle\n");
            pass = false;
        }
    }

    int fi = pick_in_cycle(fall_a, n_fall, spec->expected_cycle, spec->fall_deg);
    if (fi < 0) {
        uint elsewhere = n_fall - count_in_cycle(fall_a, n_fall, spec->expected_cycle);
        if (n_fall == 0) {
            printf("  FAIL: missing falling edge\n");
        } else if (elsewhere > 0) {
            printf("  FAIL: falling edge active in wrong revolution (expected cycle %d)\n",
                   spec->expected_cycle);
        } else {
            printf("  FAIL: missing falling edge in expected cycle\n");
        }
        pass = false;
    } else {
        double err = fall_a[fi] - spec->fall_deg;
        if (err < 0) err = -err;
        if (err > spec->fall_tol_deg) {
            printf("  FAIL: falling edge at %.2fdeg, out of tolerance\n", fall_a[fi]);
            pass = false;
        }
        if (count_in_cycle(fall_a, n_fall, spec->expected_cycle) > 1) {
            printf("  FAIL: extra falling edge(s) in expected cycle\n");
            pass = false;
        }
    }

    if (ri >= 0 && fi >= 0) {
        if (fall_a[fi] < rise_a[ri]) {
            printf("  FAIL: incorrect polarity (falling edge before rising edge)\n");
            pass = false;
        } else {
            double width = fall_a[fi] - rise_a[ri];
            double expected_width = spec->fall_deg - spec->rise_deg;
            double width_tol = spec->rise_tol_deg + spec->fall_tol_deg;
            double width_err = width - expected_width;
            if (width_err < 0) width_err = -width_err;
            if (width_err > width_tol) {
                printf("  FAIL: pulse width %.2fdeg, out of tolerance (expected %.2fdeg)\n",
                       width, expected_width);
                pass = false;
            }
        }
    }

    printf("  %s\n", pass ? "PASS" : "-> FAIL (see above)");
}

static void run_mode_capture_test(void) {
    // --- pio0: crank/cam one-shot generator ---
    PIO pio_gen = pio0;
    uint sm_crank = pio_claim_unused_sm(pio_gen, true);
    uint sm_cam = pio_claim_unused_sm(pio_gen, true);
    uint gen_offset = pio_add_program(pio_gen, &event_gen_program);
    float gen_clkdiv = 1.0f;
    f_pio_hz = (double)clock_get_hz(clk_sys) / gen_clkdiv;

    build_position_cycles_constant(CONSTANT_RPM);
    build_crank_events(crank_events[0]);
    build_cam_events(cam_events[0]);

    uint dma_crank_chan_ = dma_claim_unused_channel(true);
    uint dma_cam_chan_ = dma_claim_unused_channel(true);

    // --- pio1: 6-channel capture ---
    PIO pio_cap = pio1;
    uint sm_capture = pio_claim_unused_sm(pio_cap, true);
    uint cap_offset = pio_add_program(pio_cap, &capture_program);
    float cap_clkdiv = (float)(clock_get_hz(clk_sys) / (2.0 * CAPTURE_SAMPLE_HZ));
    uint dma_capture_chan = dma_claim_unused_channel(true);

    double sample_period_ms = 1000.0 / CAPTURE_SAMPLE_HZ;
    printf("capture_test: crank pin=%d, cam pin=%d, capture base=%d (%d ch), "
           "sample=%.0f Hz, window=%.0f ms (%u samples)\n",
           CRANK_PIN, CAM_PIN, CAPTURE_BASE_PIN, CAPTURE_PIN_COUNT,
           CAPTURE_SAMPLE_HZ, CAPTURE_DURATION_MS, CAPTURE_SAMPLES);
    printf("wire GPIO2->GPIO6 and GPIO3->GPIO7, then press Enter to capture.\n");

    while (true) {
        int ch = getchar();
        (void)ch;

        // Reset generator SMs/DMA (harmless no-op on the first run).
        pio_sm_set_enabled(pio_gen, sm_crank, false);
        pio_sm_set_enabled(pio_gen, sm_cam, false);
        dma_channel_abort(dma_crank_chan_);
        dma_channel_abort(dma_cam_chan_);
        pio_sm_clear_fifos(pio_gen, sm_crank);
        pio_sm_clear_fifos(pio_gen, sm_cam);
        event_gen_program_init(pio_gen, sm_crank, gen_offset, CRANK_PIN, gen_clkdiv);
        event_gen_program_init(pio_gen, sm_cam, gen_offset, CAM_PIN, gen_clkdiv);

        dma_channel_config cfg_crank = dma_channel_get_default_config(dma_crank_chan_);
        channel_config_set_transfer_data_size(&cfg_crank, DMA_SIZE_32);
        channel_config_set_read_increment(&cfg_crank, true);
        channel_config_set_write_increment(&cfg_crank, false);
        channel_config_set_dreq(&cfg_crank, pio_get_dreq(pio_gen, sm_crank, true));
        dma_channel_configure(dma_crank_chan_, &cfg_crank, &pio_gen->txf[sm_crank],
                               crank_events[0], CRANK_WORDS_TOTAL, false);

        dma_channel_config cfg_cam = dma_channel_get_default_config(dma_cam_chan_);
        channel_config_set_transfer_data_size(&cfg_cam, DMA_SIZE_32);
        channel_config_set_read_increment(&cfg_cam, true);
        channel_config_set_write_increment(&cfg_cam, false);
        channel_config_set_dreq(&cfg_cam, pio_get_dreq(pio_gen, sm_cam, true));
        dma_channel_configure(dma_cam_chan_, &cfg_cam, &pio_gen->txf[sm_cam],
                               cam_events[0], CAM_WORDS_TOTAL, false);

        // Reset capture SM/DMA.
        pio_sm_set_enabled(pio_cap, sm_capture, false);
        dma_channel_abort(dma_capture_chan);
        pio_sm_clear_fifos(pio_cap, sm_capture);
        capture_program_init(pio_cap, sm_capture, cap_offset, CAPTURE_BASE_PIN,
                              CAPTURE_PIN_COUNT, cap_clkdiv);

        dma_channel_config cfg_cap = dma_channel_get_default_config(dma_capture_chan);
        channel_config_set_transfer_data_size(&cfg_cap, DMA_SIZE_32);
        channel_config_set_read_increment(&cfg_cap, false);
        channel_config_set_write_increment(&cfg_cap, true);
        channel_config_set_dreq(&cfg_cap, pio_get_dreq(pio_cap, sm_capture, false)); // RX
        dma_channel_configure(dma_capture_chan, &cfg_cap, capture_buf, &pio_cap->rxf[sm_capture],
                               CAPTURE_SAMPLES, false);

        // Arm capture first so it's running before generation starts --
        // angle-correlated sync isn't needed here (see capture.pio header
        // comment); we just don't want to miss the first edge. This
        // margin used to come for free from code-sequence timing, which
        // isn't a real guarantee -- shifting code layout elsewhere (e.g.
        // merging this into a bigger binary) can tip that race the wrong
        // way silently (missed the very first crank rising edge, which
        // corrupted the reference-detection in compute_crank_reference).
        // An explicit margin makes this robust instead of timing-lucky.
        dma_channel_start(dma_capture_chan);
        pio_sm_set_enabled(pio_cap, sm_capture, true);
        busy_wait_us(100);

        dma_channel_start(dma_crank_chan_);
        dma_channel_start(dma_cam_chan_);
        pio_enable_sm_mask_in_sync(pio_gen, (1u << sm_crank) | (1u << sm_cam));

        printf("capturing...\n");
        dma_channel_wait_for_finish_blocking(dma_capture_chan);
        pio_sm_set_enabled(pio_cap, sm_capture, false);

        printf("done. Edges found:\n");
        for (uint ch2 = 0; ch2 < CAPTURE_PIN_COUNT; ch2++) {
            print_channel_edges(ch2, sample_period_ms);
        }

        double refs[MAX_REFS];
        uint n_refs;
        int rev0_window;
        bool have_refs = compute_crank_reference(sample_period_ms, refs, &n_refs, &rev0_window);
        if (have_refs) {
            angle_conversion_demo(sample_period_ms, refs, n_refs, rev0_window);

            printf("pass/fail evaluation:\n");
            // ch1 (cam loopback): matches doc section 9's own example --
            // expect PASS, validating the checker against known-good data.
            EcuOutputSpec spec_cam = {"cam-loopback", 1, 120.0, 1.0, 300.0, 1.0, 0};
            evaluate_spec(&spec_cam, sample_period_ms, refs, n_refs, rev0_window);
            // ch2 has nothing wired to it: expect FAIL (missing edges),
            // demonstrating the checker actually catches a bad output.
            EcuOutputSpec spec_unwired = {"ch2-unwired-demo", 2, 200.0, 1.0, 260.0, 1.0, 0};
            evaluate_spec(&spec_unwired, sample_period_ms, refs, n_refs, rev0_window);
        }
        printf("Press Enter to capture again.\n");
    }
}

// ===================== Boot menu =====================

int main(void) {
    stdio_init_all();
    sleep_ms(1500); // give the host time to open the serial port

    printf("\ncrankcam testbench\n");
    printf(" 1) continuous double-buffered generation (normal operation)\n");
    printf(" 2) single-shot 2-rev diagnostic, constant RPM (scope bring-up)\n");
    printf(" 3) capture test: one-shot gen + 6ch capture + angle + pass/fail\n");
    printf("select mode (reset/replug to change mode later): ");
    fflush(stdout);

    int c;
    do {
        c = getchar();
    } while (c != '1' && c != '2' && c != '3');
    printf("%c\n", (char)c);

    if (c == '1') {
        run_mode_continuous();
    } else if (c == '2') {
        run_mode_singleshot();
    } else {
        run_mode_capture_test();
    }
    // unreachable: each mode loops forever
}
