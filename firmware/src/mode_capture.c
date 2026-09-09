#include <stdio.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"

#include "mode_capture.h"
#include "event_gen.pio.h"
#include "capture.pio.h"
#include "event_table.h"
#include "capture_analysis.h"

void run_mode_capture_test(void) {
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

    uint dma_crank_chan = dma_claim_unused_channel(true);
    uint dma_cam_chan = dma_claim_unused_channel(true);

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

        dma_channel_start(dma_crank_chan);
        dma_channel_start(dma_cam_chan);
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
