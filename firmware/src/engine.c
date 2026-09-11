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

            fill_buffer_slot_constant(slot, (double)target_rpm);

            dma_channel_set_read_addr(dma_crank_chan[slot], crank_events[slot], false);
            dma_channel_set_trans_count(dma_crank_chan[slot], crank_words_total, false);

            uint other = slot ^ 1; // NUM_BUFFERS == 2
            dma_channel_set_read_addr(dma_cam_chan[other], cam_events[other], false);
            dma_channel_set_trans_count(dma_cam_chan[other], CAM_WORDS_TOTAL, true);
        }

        uint cap_chan = dma_capture_chan[slot];
        if (dma_hw->ints0 & (1u << cap_chan)) {
            dma_hw->ints0 = 1u << cap_chan;

            uint32_t samples = samples_for_rpm(target_rpm);
            dma_channel_set_write_addr(dma_capture_chan[slot], live_capture_buf[slot], false);
            dma_channel_set_trans_count(dma_capture_chan[slot], samples, false);

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

    double sample_period_ms = 1000.0 / CAPTURE_SAMPLE_HZ;
    double refs[MAX_REFS];
    uint n_refs;
    int rev0_window;
    bool have_refs = compute_crank_reference(sample_period_ms, refs, &n_refs, &rev0_window);

    uint32_t skipped = produced_now - consumed_cycles - 1;
    consumed_cycles = produced_now;

    printf("cycle %u:\n", consumed_cycles);
    if (skipped > 0) {
        printf("  (%u earlier cycle(s) skipped -- printing fell behind; generation kept running)\n",
               skipped);
    }
    if (have_refs) {
        report_pulse_angles(CAPTURE_PIN_COUNT, sample_period_ms, refs, n_refs, rev0_window);
    } else {
        printf("  no valid cycle reference this pass\n");
    }
}
