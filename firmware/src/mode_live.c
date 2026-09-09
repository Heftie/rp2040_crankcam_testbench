#include <stdio.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"

#include "mode_live.h"
#include "event_gen.pio.h"
#include "capture.pio.h"
#include "event_table.h"
#include "gen_fire.h"
#include "capture_analysis.h"

// RPM choices for the live-report menu; 1000 RPM is the floor (sizes the
// capture buffers below), 7000 RPM the requested ceiling.
static const double live_rpm_choices[] = {1000.0, 3000.0, 5000.0, 7000.0};
#define LIVE_RPM_CHOICE_COUNT (sizeof(live_rpm_choices) / sizeof(live_rpm_choices[0]))

// Two physical capture buffers, genuinely ping-ponged the same way
// crank_events[] is (real mutual chain_to, own completion IRQ -- unlike
// cam, capture's buffer is always far bigger than the SM's 8-word FIFO,
// so its DMA completion is genuinely paced by real sample arrival, same
// as crank's TX case; it doesn't have cam's "fits the FIFO instantly"
// problem, so it doesn't need cam's explicit-drive workaround either).
// Sized above the slowest (most samples-per-cycle) RPM choice's need.
#define LIVE_MAX_CYCLE_SAMPLES 12000u
static uint32_t live_capture_buf[NUM_BUFFERS][LIVE_MAX_CYCLE_SAMPLES];

static uint dma_crank_chan[NUM_BUFFERS];
static uint dma_cam_chan[NUM_BUFFERS];
static uint dma_capture_chan[NUM_BUFFERS];
static uint32_t live_want_samples; // per-cycle capture length for the chosen RPM

// Set by dma_irq_handler() when a capture buffer finishes; consumed by
// the main loop. -1 = nothing new since the main loop last checked.
static volatile int ready_slot = -1;
// Every ISR pass that sets ready_slot bumps this; the main loop compares
// against its own count of cycles it has actually processed so it can
// report if printing fell behind and a cycle's report was skipped --
// the capture/generation hardware itself never skips or stalls either
// way, only the printed report can fall behind.
static volatile uint32_t produced_cycles = 0;

static double select_rpm_menu(void) {
    printf("\nlive capture: select constant RPM:\n");
    for (uint i = 0; i < LIVE_RPM_CHOICE_COUNT; i++) {
        printf(" %u) %.0f RPM\n", i + 1, live_rpm_choices[i]);
    }
    printf("RPM: ");
    fflush(stdout);

    int c;
    do {
        c = getchar();
    } while (c < '1' || c > '0' + (int)LIVE_RPM_CHOICE_COUNT);
    printf("%c\n", (char)c);
    return live_rpm_choices[c - '1'];
}

static void configure_capture_ping_pong(PIO pio, uint sm, uint32_t *buf, uint32_t word_count,
                                         uint this_chan, uint chain_to_chan) {
    dma_channel_config cfg = dma_channel_get_default_config(this_chan);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&cfg, false);
    channel_config_set_write_increment(&cfg, true);
    channel_config_set_dreq(&cfg, pio_get_dreq(pio, sm, false)); // RX
    channel_config_set_chain_to(&cfg, chain_to_chan);
    dma_channel_configure(this_chan, &cfg, buf, &pio->rxf[sm], word_count, false);
}

// Two independent completion events land in the same combined handler:
//
// crank_chan[slot] completing -- exactly the event mode_continuous.c's
// fix rearms crank/cam from. crank_chan[other] has already been
// hardware-chained by this same completion, so "other" is the slot
// beginning its cycle right now; "slot" just finished and is safe to
// rearm/restart (a full cycle away from being read/played again). Cam
// still can't self-chain (see run_mode_continuous()'s comment in
// mode_continuous.c -- same reasoning here), so it's driven explicitly
// from this event, same as mode 1.
//
// capture_chan[slot] completing (a *separate* event, not necessarily at
// the same instant as any particular crank completion, since it's a
// genuinely independent free-running chain -- see the capture_buf[]
// comment above) -- capture is FIFO-paced like crank, so it only needs
// the same simple rearm, plus flagging its just-filled buffer as ready
// for the main loop to analyze while the *other* capture buffer keeps
// filling.
static void dma_irq_handler(void) {
    for (uint slot = 0; slot < NUM_BUFFERS; slot++) {
        uint chan = dma_crank_chan[slot];
        if (dma_hw->ints0 & (1u << chan)) {
            dma_hw->ints0 = 1u << chan;

            dma_channel_set_read_addr(dma_crank_chan[slot], crank_events[slot], false);
            dma_channel_set_trans_count(dma_crank_chan[slot], crank_words_total, false);

            uint other = slot ^ 1; // NUM_BUFFERS == 2
            dma_channel_set_read_addr(dma_cam_chan[other], cam_events[other], false);
            dma_channel_set_trans_count(dma_cam_chan[other], CAM_WORDS_TOTAL, true);
        }

        uint cap_chan = dma_capture_chan[slot];
        if (dma_hw->ints0 & (1u << cap_chan)) {
            dma_hw->ints0 = 1u << cap_chan;

            dma_channel_set_write_addr(dma_capture_chan[slot], live_capture_buf[slot], false);
            dma_channel_set_trans_count(dma_capture_chan[slot], live_want_samples, false);

            ready_slot = (int)slot;
            produced_cycles++;
        }
    }
}

void run_mode_live(void) {
    double rpm = select_rpm_menu();

    // --- pio0: continuous crank/cam generation, constant RPM ---
    PIO pio_gen = pio0;
    uint sm_crank = pio_claim_unused_sm(pio_gen, true);
    uint sm_cam = pio_claim_unused_sm(pio_gen, true);
    uint gen_offset = pio_add_program(pio_gen, &event_gen_program);
    float gen_clkdiv = 1.0f;
    f_pio_hz = (double)clock_get_hz(clk_sys) / gen_clkdiv;

    // Same content in both slots -- no RPM ramp in this mode, so unlike
    // mode 1 there is nothing to refill on each cycle, only DMA
    // registers to rearm.
    build_position_cycles_constant(rpm);
    build_crank_events(crank_events[0]);
    build_cam_events(cam_events[0]);
    build_crank_events(crank_events[1]);
    build_cam_events(cam_events[1]);

    event_gen_program_init(pio_gen, sm_crank, gen_offset, CRANK_PIN, gen_clkdiv);
    event_gen_program_init(pio_gen, sm_cam, gen_offset, CAM_PIN, gen_clkdiv);

    dma_crank_chan[0] = dma_claim_unused_channel(true);
    dma_crank_chan[1] = dma_claim_unused_channel(true);
    configure_ping_pong_channel(pio_gen, sm_crank, crank_events[0], crank_words_total,
                                 dma_crank_chan[0], dma_crank_chan[1]);
    configure_ping_pong_channel(pio_gen, sm_crank, crank_events[1], crank_words_total,
                                 dma_crank_chan[1], dma_crank_chan[0]);

    // Cam channels chain to themselves (disabled) -- see dma_irq_handler().
    dma_cam_chan[0] = dma_claim_unused_channel(true);
    dma_cam_chan[1] = dma_claim_unused_channel(true);
    configure_ping_pong_channel(pio_gen, sm_cam, cam_events[0], CAM_WORDS_TOTAL,
                                 dma_cam_chan[0], dma_cam_chan[0]);
    configure_ping_pong_channel(pio_gen, sm_cam, cam_events[1], CAM_WORDS_TOTAL,
                                 dma_cam_chan[1], dma_cam_chan[1]);

    // --- pio1: continuous 6-channel capture ---
    PIO pio_cap = pio1;
    uint sm_capture = pio_claim_unused_sm(pio_cap, true);
    uint cap_offset = pio_add_program(pio_cap, &capture_program);
    float cap_clkdiv = (float)(clock_get_hz(clk_sys) / (2.0 * CAPTURE_SAMPLE_HZ));
    capture_program_init(pio_cap, sm_capture, cap_offset, CAPTURE_BASE_PIN,
                          CAPTURE_PIN_COUNT, cap_clkdiv);

    double sample_period_ms = 1000.0 / CAPTURE_SAMPLE_HZ;
    double cycle_ms = REVS_PER_CYCLE * 60000.0 / rpm;
    // Nearest-integer match to one real 720deg cycle's duration, no
    // deliberate under/over margin -- capture free-runs continuously
    // (see live_capture_buf[] comment), so unlike a one-shot capture
    // there's no trailing-reference margin to reserve room for. A
    // cycle-to-cycle rounding remainder of a fraction of a sample can
    // very slowly drift capture's phase relative to generation's actual
    // cycle boundary over many minutes of continuous running; over a
    // normal test session this is imperceptible.
    live_want_samples = (uint32_t)(cycle_ms / 1000.0 * CAPTURE_SAMPLE_HZ + 0.5);
    if (live_want_samples > LIVE_MAX_CYCLE_SAMPLES) live_want_samples = LIVE_MAX_CYCLE_SAMPLES;

    dma_capture_chan[0] = dma_claim_unused_channel(true);
    dma_capture_chan[1] = dma_claim_unused_channel(true);
    configure_capture_ping_pong(pio_cap, sm_capture, live_capture_buf[0], live_want_samples,
                                 dma_capture_chan[0], dma_capture_chan[1]);
    configure_capture_ping_pong(pio_cap, sm_capture, live_capture_buf[1], live_want_samples,
                                 dma_capture_chan[1], dma_capture_chan[0]);

    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);
    dma_channel_set_irq0_enabled(dma_crank_chan[0], true);
    dma_channel_set_irq0_enabled(dma_crank_chan[1], true);
    dma_channel_set_irq0_enabled(dma_capture_chan[0], true);
    dma_channel_set_irq0_enabled(dma_capture_chan[1], true);
    irq_set_enabled(DMA_IRQ_0, true);

    printf("live capture: %.0f RPM, cycle=%.2fms, capturing %u samples/cycle (%.2fms window)\n",
           rpm, cycle_ms, live_want_samples, live_want_samples * sample_period_ms);
    printf("wire GPIO2->GPIO6 and GPIO3->GPIO7 for crank/cam loopback; ECU outputs on GPIO%d-%d.\n",
           CAPTURE_BASE_PIN + 2, CAPTURE_BASE_PIN + CAPTURE_PIN_COUNT - 1);
    printf("continuous, hardware-gapless -- crank/cam never pause between cycles.\n");
    printf("press Enter to start; press Enter again at any time to stop.\n");
    getchar();

    // Arm capture slot 0 first (margin so it can't miss the first crank
    // edge -- same rationale as mode 3's one-shot arming), then start
    // crank+cam slot 0 together, phase-locked.
    dma_channel_start(dma_capture_chan[0]);
    pio_sm_set_enabled(pio_cap, sm_capture, true);
    busy_wait_us(100);

    dma_channel_start(dma_crank_chan[0]);
    dma_channel_start(dma_cam_chan[0]);
    pio_enable_sm_mask_in_sync(pio_gen, (1u << sm_crank) | (1u << sm_cam));

    uint32_t consumed_cycles = 0;
    while (true) {
        if (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT) {
            printf("stopped after %u cycles.\n", consumed_cycles);
            break;
        }

        int slot = ready_slot;
        if (slot < 0) {
            continue;
        }
        ready_slot = -1;
        uint32_t produced_now = produced_cycles;

        capture_buf = live_capture_buf[slot];
        capture_samples_used = live_want_samples;

        double refs[MAX_REFS];
        uint n_refs;
        int rev0_window;
        bool have_refs = compute_crank_reference(sample_period_ms, refs, &n_refs, &rev0_window);

        // produced_now counts every cycle completed so far, including
        // ones whose ready_slot got overwritten before we got to them --
        // catching consumed_cycles up to it both labels this report with
        // its real cycle number and surfaces how many were skipped.
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
}
