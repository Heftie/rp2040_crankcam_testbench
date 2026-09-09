#include <stdio.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"

#include "mode_live.h"
#include "event_gen.pio.h"
#include "capture.pio.h"
#include "event_table.h"
#include "capture_analysis.h"

// RPM choices for the live-report menu; 1000 RPM is the floor (its 25%-
// margin capture window exactly fills capture_buf[]'s CAPTURE_SAMPLES
// capacity -- see run_mode_live()), 7000 RPM the requested ceiling.
static const double live_rpm_choices[] = {1000.0, 3000.0, 5000.0, 7000.0};
#define LIVE_RPM_CHOICE_COUNT (sizeof(live_rpm_choices) / sizeof(live_rpm_choices[0]))

// Headroom beyond one 720deg cycle's nominal duration so the trailing
// crank reference (see capture_analysis.c's find_crank_references header
// comment) has a real closing edge to find, same margin mode_capture.c's
// fixed 150ms/1000RPM window used.
#define LIVE_CAPTURE_MARGIN 1.25

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

void run_mode_live(void) {
    double rpm = select_rpm_menu();

    // --- pio0: crank/cam one-shot generator (same as mode 3) ---
    PIO pio_gen = pio0;
    uint sm_crank = pio_claim_unused_sm(pio_gen, true);
    uint sm_cam = pio_claim_unused_sm(pio_gen, true);
    uint gen_offset = pio_add_program(pio_gen, &event_gen_program);
    float gen_clkdiv = 1.0f;
    f_pio_hz = (double)clock_get_hz(clk_sys) / gen_clkdiv;

    build_position_cycles_constant(rpm);
    build_crank_events(crank_events[0]);
    build_cam_events(cam_events[0]);

    uint dma_crank_chan = dma_claim_unused_channel(true);
    uint dma_cam_chan = dma_claim_unused_channel(true);

    // --- pio1: 6-channel capture, sized to one 720deg cycle at this RPM ---
    PIO pio_cap = pio1;
    uint sm_capture = pio_claim_unused_sm(pio_cap, true);
    uint cap_offset = pio_add_program(pio_cap, &capture_program);
    float cap_clkdiv = (float)(clock_get_hz(clk_sys) / (2.0 * CAPTURE_SAMPLE_HZ));
    uint dma_capture_chan = dma_claim_unused_channel(true);

    double sample_period_ms = 1000.0 / CAPTURE_SAMPLE_HZ;
    double cycle_ms = REVS_PER_CYCLE * 60000.0 / rpm;
    uint32_t want_samples = (uint32_t)(cycle_ms * LIVE_CAPTURE_MARGIN / 1000.0 * CAPTURE_SAMPLE_HZ) + 1;
    if (want_samples > CAPTURE_SAMPLES) want_samples = CAPTURE_SAMPLES;

    printf("live capture: %.0f RPM, cycle=%.2fms, capturing %u samples (%.2fms window/cycle)\n",
           rpm, cycle_ms, want_samples, want_samples * sample_period_ms);
    printf("wire GPIO2->GPIO6 and GPIO3->GPIO7 for crank/cam loopback; ECU outputs on GPIO%d-%d.\n",
           CAPTURE_BASE_PIN + 2, CAPTURE_BASE_PIN + CAPTURE_PIN_COUNT - 1);
    printf("press Enter to start; press Enter again at any time to stop.\n");
    getchar();

    uint32_t cycle_count = 0;
    while (true) {
        // Non-blocking stop check: a real keypress here ends the loop
        // instead of waiting for the next cycle to finish.
        if (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT) {
            printf("stopped after %u cycles.\n", cycle_count);
            break;
        }

        // Reset generator SMs/DMA (harmless no-op on the first pass).
        pio_sm_set_enabled(pio_gen, sm_crank, false);
        pio_sm_set_enabled(pio_gen, sm_cam, false);
        dma_channel_abort(dma_crank_chan);
        dma_channel_abort(dma_cam_chan);
        pio_sm_clear_fifos(pio_gen, sm_crank);
        pio_sm_clear_fifos(pio_gen, sm_cam);
        event_gen_program_init(pio_gen, sm_crank, gen_offset, CRANK_PIN, gen_clkdiv);
        event_gen_program_init(pio_gen, sm_cam, gen_offset, CAM_PIN, gen_clkdiv);

        dma_channel_config cfg_crank = dma_channel_get_default_config(dma_crank_chan);
        channel_config_set_transfer_data_size(&cfg_crank, DMA_SIZE_32);
        channel_config_set_read_increment(&cfg_crank, true);
        channel_config_set_write_increment(&cfg_crank, false);
        channel_config_set_dreq(&cfg_crank, pio_get_dreq(pio_gen, sm_crank, true));
        dma_channel_configure(dma_crank_chan, &cfg_crank, &pio_gen->txf[sm_crank],
                               crank_events[0], crank_words_total, false);

        dma_channel_config cfg_cam = dma_channel_get_default_config(dma_cam_chan);
        channel_config_set_transfer_data_size(&cfg_cam, DMA_SIZE_32);
        channel_config_set_read_increment(&cfg_cam, true);
        channel_config_set_write_increment(&cfg_cam, false);
        channel_config_set_dreq(&cfg_cam, pio_get_dreq(pio_gen, sm_cam, true));
        dma_channel_configure(dma_cam_chan, &cfg_cam, &pio_gen->txf[sm_cam],
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
                               want_samples, false);

        // Arm capture before generation starts, same margin rationale as
        // mode_capture.c (see its header comment): don't miss the first
        // crank edge, which would corrupt reference detection below.
        dma_channel_start(dma_capture_chan);
        pio_sm_set_enabled(pio_cap, sm_capture, true);
        busy_wait_us(100);

        dma_channel_start(dma_crank_chan);
        dma_channel_start(dma_cam_chan);
        pio_enable_sm_mask_in_sync(pio_gen, (1u << sm_crank) | (1u << sm_cam));

        dma_channel_wait_for_finish_blocking(dma_capture_chan);
        pio_sm_set_enabled(pio_cap, sm_capture, false);
        capture_samples_used = want_samples;

        cycle_count++;
        double refs[MAX_REFS];
        uint n_refs;
        int rev0_window;
        bool have_refs = compute_crank_reference(sample_period_ms, refs, &n_refs, &rev0_window);
        printf("cycle %u:\n", cycle_count);
        if (have_refs) {
            report_pulse_angles(CAPTURE_PIN_COUNT, sample_period_ms, refs, n_refs, rev0_window);
        } else {
            printf("  no valid cycle reference this pass\n");
        }
    }
}
