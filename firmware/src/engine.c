#include <stdio.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"

#include "engine.h"
#include "event_gen.pio.h"
#include "capture.pio.h"
#include "event_table.h"
#include "gen_fire.h"
#include "capture_analysis.h"

// Capture buffer capacity: sized for CAPTURE_MIN_RPM, the slowest RPM
// capture supports (most samples needed for one 720deg cycle). Faster
// RPM just uses fewer of each buffer's entries -- see samples_for_rpm().
#define LIVE_MAX_CYCLE_SAMPLES 12000u
static uint32_t live_capture_buf[NUM_BUFFERS][LIVE_MAX_CYCLE_SAMPLES];

static PIO pio_gen, pio_cap;
static uint sm_crank, sm_cam, sm_capture;
static uint gen_offset, cap_offset;
static float gen_clkdiv = 1.0f;
static float cap_clkdiv;

static uint dma_crank_chan[NUM_BUFFERS];
static uint dma_cam_chan[NUM_BUFFERS];
static uint dma_capture_chan[NUM_BUFFERS];

static const CrankCamProfile *current_profile;
static volatile uint32_t target_rpm = 1000; // single 32-bit word: atomic read/write on Cortex-M0+
static volatile bool gen_running = false;
static volatile bool capture_running = false;

// Generation's own timebase, replacing captured-crank-edge decoding as
// the angle reference (see capture_analysis.h's CycleBoundary): a short
// history of recent 720deg cycle-start instants + the constant RPM each
// one plays at, latched by dma_irq_handler's crank branch (and once
// directly by engine_start_gen() for the very first cycle). Keeping a
// few entries, not just the latest, covers a capture buffer whose
// independent DMA chain happens to straddle a cycle boundary.
#define CYCLE_HISTORY 3
static CycleBoundary cycle_history[CYCLE_HISTORY];
static uint cycle_history_count = 0;

// RPM each slot's crank buffer was actually built at, carried from fill
// time to the moment that buffer starts playing (one dma_irq_handler
// crank-branch call later) -- target_rpm may have changed live in
// between, so the completing IRQ can't just re-read target_rpm to learn
// what the buffer that's starting *now* was built for.
static volatile uint32_t pending_rpm[NUM_BUFFERS];

// Timebase instant each capture slot's buffer started filling, latched
// the same way as cycle_history (DMA completion IRQ = other slot's
// chain_to just fired) plus once directly by engine_start_capture() for
// slot 0.
static volatile uint64_t cap_slot_start_us[NUM_BUFFERS];

static void push_cycle_boundary(uint64_t start_us, uint32_t rpm) {
    if (cycle_history_count < CYCLE_HISTORY) {
        cycle_history[cycle_history_count++] = (CycleBoundary){start_us, rpm};
        return;
    }
    for (uint i = 1; i < CYCLE_HISTORY; i++) cycle_history[i - 1] = cycle_history[i];
    cycle_history[CYCLE_HISTORY - 1] = (CycleBoundary){start_us, rpm};
}

// event_gen_program_init() joins each generation SM's TX FIFO
// (PIO_FIFO_JOIN_TX), doubling it to 8 words = 4 events. The crank DMA
// channel's dreq only fires on FIFO space, so in steady playback the
// FIFO sits essentially always full -- meaning at the instant a crank
// channel's *last* word is accepted ("DMA complete"), up to
// CRANK_FIFO_BACKLOG_WORDS worth of that same buffer's trailing events
// are still queued, not yet actually shifted out to the pin. Measured on
// hardware: without this compensation, every cycle boundary latched from
// this IRQ (all but the very first, which is latched directly at
// pio_enable_sm_mask_in_sync -- no DMA involved, no backlog) came in
// early by a constant amount, e.g. a fixed ~24deg phase offset on cam's
// 120/300deg pulse at 3000 RPM on a 60-2 profile. This sums the still-
// queued trailing events' real durations (from the buffer that's about
// to be overwritten, so this must run before fill_buffer_slot_constant()
// touches it) and converts to microseconds via f_pio_hz.
#define CRANK_FIFO_BACKLOG_WORDS 10u

static uint64_t crank_tail_backlog_us(uint slot) {
    uint32_t total = crank_words_total;
    uint32_t start = (total > CRANK_FIFO_BACKLOG_WORDS) ? total - CRANK_FIFO_BACKLOG_WORDS : 0;
    uint32_t sum_cycles = 0;
    for (uint32_t i = start; i + 1 < total; i += 2) {
        sum_cycles += crank_events[slot][i + 1] + EVENT_MIN_CYCLES;
    }
    return (uint64_t)((double)sum_cycles / f_pio_hz * 1e6 + 0.5);
}

// Set by dma_irq_handler() when a capture buffer finishes; consumed by
// engine_poll_capture(). -1 = nothing new since it was last checked.
static volatile int ready_slot = -1;
// Bumped on every capture-buffer completion; lets engine_poll_capture()
// notice and report if it fell behind (printing is slower than capture).
static volatile uint32_t produced_cycles = 0;
static uint32_t consumed_cycles = 0; // main-loop only, never touched by the ISR

static uint32_t samples_for_rpm(uint32_t rpm) {
    double cycle_ms = REVS_PER_CYCLE * 60000.0 / (double)rpm;
    uint32_t samples = (uint32_t)(cycle_ms / 1000.0 * CAPTURE_SAMPLE_HZ + 0.5);
    if (samples > LIVE_MAX_CYCLE_SAMPLES) samples = LIVE_MAX_CYCLE_SAMPLES;
    return samples;
}

static void configure_capture_ping_pong(uint32_t *buf, uint32_t word_count,
                                         uint this_chan, uint chain_to_chan) {
    dma_channel_config cfg = dma_channel_get_default_config(this_chan);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&cfg, false);
    channel_config_set_write_increment(&cfg, true);
    channel_config_set_dreq(&cfg, pio_get_dreq(pio_cap, sm_capture, false)); // RX
    channel_config_set_chain_to(&cfg, chain_to_chan);
    dma_channel_configure(this_chan, &cfg, buf, &pio_cap->rxf[sm_capture], word_count, false);
}

// Two independent completion events land in this one combined handler,
// exactly as in the old mode 1 / mode 4 (see CLAUDE.md's mode summaries
// for the two hardware bugs this rearm logic works around):
//
// crank_chan[slot] completing -- the slot that just finished is safe to
// refill (won't be read again for a full cycle); hardware has already
// auto-chained the other slot's crank channel, so that other slot's cam
// channel (which can't self-chain -- see below) is started explicitly
// here, right after it. Rebuilding from target_rpm on every boundary
// (instead of once at mode entry) is what makes 'r' take effect live.
//
// capture_chan[slot] completing -- only enabled while capture_running.
// Recomputing live_want_samples from target_rpm here too means a live
// RPM change is reflected in the next capture buffer's length as well.
static void dma_irq_handler(void) {
    for (uint slot = 0; slot < NUM_BUFFERS; slot++) {
        uint chan = dma_crank_chan[slot];
        if (dma_hw->ints0 & (1u << chan)) {
            dma_hw->ints0 = 1u << chan;

            uint other = slot ^ 1; // NUM_BUFFERS == 2

            // 'other's buffer (built the last time slot's crank
            // completed, at pending_rpm[other]) starts playing right now
            // via hardware chain_to -- this instant, plus however much
            // of slot's own just-completed buffer is still backlogged in
            // the FIFO (see crank_tail_backlog_us()), is the new cycle's
            // 0deg reference. Must run before fill_buffer_slot_constant()
            // below overwrites crank_events[slot].
            push_cycle_boundary(time_us_64() + crank_tail_backlog_us(slot), pending_rpm[other]);

            fill_buffer_slot_constant(slot, (double)target_rpm);
            pending_rpm[slot] = target_rpm;

            dma_channel_set_read_addr(dma_crank_chan[slot], crank_events[slot], false);
            dma_channel_set_trans_count(dma_crank_chan[slot], crank_words_total, false);

            dma_channel_set_read_addr(dma_cam_chan[other], cam_events[other], false);
            dma_channel_set_trans_count(dma_cam_chan[other], CAM_WORDS_TOTAL, true);
        }

        uint cap_chan = dma_capture_chan[slot];
        if (dma_hw->ints0 & (1u << cap_chan)) {
            dma_hw->ints0 = 1u << cap_chan;

            uint32_t samples = samples_for_rpm(target_rpm);
            dma_channel_set_write_addr(dma_capture_chan[slot], live_capture_buf[slot], false);
            dma_channel_set_trans_count(dma_capture_chan[slot], samples, false);

            cap_slot_start_us[slot ^ 1] = time_us_64(); // other slot's capture starts now via chain_to

            ready_slot = (int)slot;
            produced_cycles++;
        }
    }
}

void engine_init(void) {
    pio_gen = pio0;
    sm_crank = pio_claim_unused_sm(pio_gen, true);
    sm_cam = pio_claim_unused_sm(pio_gen, true);
    gen_offset = pio_add_program(pio_gen, &event_gen_program);
    f_pio_hz = (double)clock_get_hz(clk_sys) / gen_clkdiv;

    pio_cap = pio1;
    sm_capture = pio_claim_unused_sm(pio_cap, true);
    cap_offset = pio_add_program(pio_cap, &capture_program);
    cap_clkdiv = (float)(clock_get_hz(clk_sys) / (2.0 * CAPTURE_SAMPLE_HZ));
    capture_program_init(pio_cap, sm_capture, cap_offset, CAPTURE_BASE_PIN,
                          CAPTURE_PIN_COUNT, cap_clkdiv);

    dma_crank_chan[0] = dma_claim_unused_channel(true);
    dma_crank_chan[1] = dma_claim_unused_channel(true);
    dma_cam_chan[0] = dma_claim_unused_channel(true);
    dma_cam_chan[1] = dma_claim_unused_channel(true);
    dma_capture_chan[0] = dma_claim_unused_channel(true);
    dma_capture_chan[1] = dma_claim_unused_channel(true);

    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    current_profile = &crankcam_profiles[0];
    select_profile(current_profile);
}

bool engine_select_profile(uint index) {
    if (gen_running) return false;
    if (index < 1 || index > crankcam_profile_count) return false;
    current_profile = &crankcam_profiles[index - 1];
    select_profile(current_profile);
    return true;
}

bool engine_set_rpm(uint32_t rpm) {
    if (rpm == 0 || rpm > ENGINE_MAX_RPM) return false;
    target_rpm = rpm;
    return true;
}

void engine_start_gen(void) {
    if (gen_running) return;

    // See the same-purpose mask in engine_start_capture() -- not observed
    // to bite here (crank/cam start from a fully-stopped state, so there's
    // no routine unrelated IRQ that could land mid-setup the way capture's
    // always-running-crank-IRQ can), but it's the identical hazard class
    // and equally cheap to close off.
    irq_set_enabled(DMA_IRQ_0, false);

    fill_buffer_slot_constant(0, (double)target_rpm);
    fill_buffer_slot_constant(1, (double)target_rpm);
    pending_rpm[0] = target_rpm;
    pending_rpm[1] = target_rpm;
    cycle_history_count = 0;

    event_gen_program_init(pio_gen, sm_crank, gen_offset, CRANK_PIN, gen_clkdiv);
    event_gen_program_init(pio_gen, sm_cam, gen_offset, CAM_PIN, gen_clkdiv);

    configure_ping_pong_channel(pio_gen, sm_crank, crank_events[0], crank_words_total,
                                 dma_crank_chan[0], dma_crank_chan[1]);
    configure_ping_pong_channel(pio_gen, sm_crank, crank_events[1], crank_words_total,
                                 dma_crank_chan[1], dma_crank_chan[0]);

    // Cam channels chain to themselves (disabled) -- driven explicitly
    // from the crank completion IRQ instead. See dma_irq_handler().
    configure_ping_pong_channel(pio_gen, sm_cam, cam_events[0], CAM_WORDS_TOTAL,
                                 dma_cam_chan[0], dma_cam_chan[0]);
    configure_ping_pong_channel(pio_gen, sm_cam, cam_events[1], CAM_WORDS_TOTAL,
                                 dma_cam_chan[1], dma_cam_chan[1]);

    // Clear any completion flag left sticky from a previous gen session --
    // dma_channel_set_irq0_enabled() only gates whether this channel's
    // completion *interrupts* the CPU, it does not clear the sticky
    // ints0 status bit itself (same latent issue fixed for capture below;
    // not observed to bite here, but it's the identical hazard).
    dma_hw->ints0 = (1u << dma_crank_chan[0]) | (1u << dma_crank_chan[1]);
    dma_channel_set_irq0_enabled(dma_crank_chan[0], true);
    dma_channel_set_irq0_enabled(dma_crank_chan[1], true);

    dma_channel_start(dma_crank_chan[0]);
    dma_channel_start(dma_cam_chan[0]);
    pio_enable_sm_mask_in_sync(pio_gen, (1u << sm_crank) | (1u << sm_cam));
    push_cycle_boundary(time_us_64(), target_rpm); // slot 0's cycle starts right now

    gen_running = true;
    irq_set_enabled(DMA_IRQ_0, true);
}

void engine_stop_gen(void) {
    if (!gen_running) return;
    if (capture_running) engine_stop_capture();

    dma_channel_set_irq0_enabled(dma_crank_chan[0], false);
    dma_channel_set_irq0_enabled(dma_crank_chan[1], false);

    pio_sm_set_enabled(pio_gen, sm_crank, false);
    pio_sm_set_enabled(pio_gen, sm_cam, false);
    dma_channel_abort(dma_crank_chan[0]);
    dma_channel_abort(dma_crank_chan[1]);
    dma_channel_abort(dma_cam_chan[0]);
    dma_channel_abort(dma_cam_chan[1]);
    pio_sm_clear_fifos(pio_gen, sm_crank);
    pio_sm_clear_fifos(pio_gen, sm_cam);

    gen_running = false;
}

bool engine_start_capture(void) {
    if (!gen_running) return false;
    if (target_rpm < CAPTURE_MIN_RPM) return false;
    if (capture_running) return true;

    // Mask DMA_IRQ_0 for this whole setup. dma_irq_handler()'s capture
    // branch is unconditional -- it doesn't check capture_running -- so
    // it runs on every crank-completion IRQ regardless (crank completes
    // once per revolution, forever, while gen is running). Without
    // masking, one of those routine, unrelated IRQs landing mid-setup
    // here can read/rearm a capture channel's registers concurrently
    // with configure_capture_ping_pong() below -- a genuine foreground/
    // ISR race on the same registers. Reproduced on hardware: a fresh
    // start_gen()+start_capture() sequence, with no delay between them
    // (so an unrelated crank IRQ was very likely to land mid-setup),
    // ended up with a capture buffer that was never actually written by
    // DMA at all (read back as all-zero for its full length) roughly
    // half the time, and it never recovered until capture was restarted.
    // A short mask here (microseconds; nowhere near a revolution period)
    // removes the race outright instead of trying to sequence around it.
    irq_set_enabled(DMA_IRQ_0, false);

    uint32_t samples = samples_for_rpm(target_rpm);
    configure_capture_ping_pong(live_capture_buf[0], samples,
                                 dma_capture_chan[0], dma_capture_chan[1]);
    configure_capture_ping_pong(live_capture_buf[1], samples,
                                 dma_capture_chan[1], dma_capture_chan[0]);

    // Also clear any stale completion flag left set from a previous
    // capture session -- dma_channel_set_irq0_enabled() only gates
    // whether this channel's completion *interrupts* the CPU, it does
    // not clear the sticky ints0 status bit itself. Every session's
    // capture channels complete (ping-ponging) many times before being
    // stopped, so this bit is essentially always left set; without
    // clearing it, the handler would misread it as a genuine completion
    // the instant IRQs are re-enabled below, before this channel has
    // even started for real this session.
    dma_hw->ints0 = (1u << dma_capture_chan[0]) | (1u << dma_capture_chan[1]);
    dma_channel_set_irq0_enabled(dma_capture_chan[0], true);
    dma_channel_set_irq0_enabled(dma_capture_chan[1], true);

    ready_slot = -1;
    produced_cycles = 0;
    consumed_cycles = 0;

    // Arm capture first so it's running before... well, generation is
    // already running here, but starting capture's own SM/DMA first
    // still avoids missing an edge that happens to land right as it
    // starts (same margin rationale as the old one-shot capture test).
    dma_channel_start(dma_capture_chan[0]);
    pio_sm_set_enabled(pio_cap, sm_capture, true);
    cap_slot_start_us[0] = time_us_64(); // slot 0's capture starts right now
    busy_wait_us(100);

    capture_running = true;
    irq_set_enabled(DMA_IRQ_0, true);
    return true;
}

void engine_stop_capture(void) {
    if (!capture_running) return;

    dma_channel_set_irq0_enabled(dma_capture_chan[0], false);
    dma_channel_set_irq0_enabled(dma_capture_chan[1], false);

    pio_sm_set_enabled(pio_cap, sm_capture, false);
    dma_channel_abort(dma_capture_chan[0]);
    dma_channel_abort(dma_capture_chan[1]);
    pio_sm_clear_fifos(pio_cap, sm_capture);

    capture_running = false;
}

bool engine_gen_running(void) { return gen_running; }
bool engine_capture_running(void) { return capture_running; }
uint32_t engine_rpm(void) { return target_rpm; }
const CrankCamProfile *engine_profile(void) { return current_profile; }

void engine_poll_capture(void) {
    if (!capture_running) return;

    int slot = ready_slot;
    if (slot < 0) return;
    ready_slot = -1;
    uint32_t produced_now = produced_cycles;

    capture_buf = live_capture_buf[slot];
    capture_samples_used = samples_for_rpm(target_rpm);

    double sample_period_us = 1000000.0 / CAPTURE_SAMPLE_HZ;

    // Snapshot the cycle-boundary history and this slot's start time --
    // brief DMA_IRQ_0 mask (same pattern used elsewhere in this file) so
    // a boundary push mid-copy can't be read half-written.
    irq_set_enabled(DMA_IRQ_0, false);
    CycleBoundary boundaries[CYCLE_HISTORY];
    uint n_boundaries = cycle_history_count;
    for (uint i = 0; i < n_boundaries; i++) boundaries[i] = cycle_history[i];
    uint64_t capture_start_us = cap_slot_start_us[slot];
    irq_set_enabled(DMA_IRQ_0, true);

    uint32_t skipped = produced_now - consumed_cycles - 1;
    consumed_cycles = produced_now;

    printf("cycle %u:\n", consumed_cycles);
    if (skipped > 0) {
        printf("  (%u earlier cycle(s) skipped -- printing fell behind; generation kept running)\n",
               skipped);
    }
    if (n_boundaries > 0) {
        report_pulse_angles(CAPTURE_PIN_COUNT, capture_start_us, sample_period_us,
                             boundaries, n_boundaries);
    } else {
        printf("  no valid cycle reference this pass\n");
    }
}
