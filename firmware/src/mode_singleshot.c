#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"

#include "mode_singleshot.h"
#include "event_gen.pio.h"
#include "event_table.h"
#include "gen_fire.h"

void run_mode_singleshot(void) {
    PIO pio = pio0;
    uint sm_crank = pio_claim_unused_sm(pio, true);
    uint sm_cam = pio_claim_unused_sm(pio, true);
    uint offset = pio_add_program(pio, &event_gen_program);

    float clkdiv = 1.0f;
    f_pio_hz = (double)clock_get_hz(clk_sys) / clkdiv;

    build_position_cycles_constant(CONSTANT_RPM);
    build_crank_events(crank_events[0]);
    build_cam_events(cam_events[0]);

    uint dma_crank_chan = dma_claim_unused_channel(true);
    uint dma_cam_chan = dma_claim_unused_channel(true);

    double total_ms = (double)position_cycles[0] * positions_total / f_pio_hz * 1000.0;
    printf("singleshot: crank pin=%d, cam pin=%d, const RPM=%.0f, "
           "cycle=%.2f ms (arm scope SINGLE, then press Enter to fire)\n",
           CRANK_PIN, CAM_PIN, CONSTANT_RPM, total_ms);

    while (true) {
        int ch = getchar(); // blocks until a byte arrives over USB serial
        (void)ch;
        fire_gen_one_shot(pio, sm_crank, sm_cam, offset, clkdiv, dma_crank_chan, dma_cam_chan);
        // Runs free in hardware from here -- not blocking on completion
        // (see fire_gen_one_shot). Press Enter again any time after the
        // cycle has had time to finish (~total_ms) for a clean re-fire.
        printf("fired, runs for %.0f ms then holds. Press Enter for next shot.\n", total_ms);
    }
}
