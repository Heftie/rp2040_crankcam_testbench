#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"

#include "mode_continuous.h"
#include "event_gen.pio.h"
#include "event_table.h"
#include "gen_fire.h"

// Double buffering demo: alternate between two RPM-profile scales each
// 720deg cycle, so buffer content visibly changes while the other buffer
// plays. The switch happens only at a 720deg cycle boundary (not
// mid-cycle), matching doc section 11's "no physically impossible
// instantaneous changes" -- a full-cycle RPM step is a normal test move.
static const double cycle_rpm_scale[2] = {1.0, 1.15};

static uint dma_crank_chan[NUM_BUFFERS];
static uint dma_cam_chan[NUM_BUFFERS];
static volatile uint32_t cycles_completed = 0;

// Fires when a crank buffer finishes playing (DMA has just chained to the
// other crank buffer). The buffer that just finished is safe to refill --
// it will not be read again until the other buffer finishes its own turn,
// a full 720deg cycle away, which is orders of magnitude longer than the
// microseconds this refill takes.
//
// A finished channel's READ_ADDR/TRANS_COUNT sit at "end of buffer, 0
// remaining" -- chain_to's hardware retrigger reuses a channel's
// registers exactly as they stand, it does not restore them. Without
// re-arming crank_chan[slot] here, the *next* time it's chain-triggered
// (a full cycle later, by the other slot) it transfers zero words and
// the pin freezes -- every buffer after the first pair silently no-ops.
//
// Cam is NOT re-armed the same way -- it isn't chained to itself at all
// (see run_mode_continuous()). Cam's whole buffer (3 events, 6 words)
// fits inside the SM's 8-word FIFO, so its own DMA "transfer complete"
// fires almost instantly, long before the SM has actually played the
// buffer out -- unlike crank, whose much bigger buffer is genuinely
// FIFO-paced. Letting cam chain to itself made it retrigger far too
// early and race this handler, corrupting playback unpredictably
// (sporadic missing/misplaced cam pulses). Instead cam is driven
// explicitly, once per cycle, right here: the OTHER slot's crank buffer
// has just started playing (hardware already auto-chained it the instant
// crank_chan[slot] completed), so start that same OTHER slot's cam
// buffer now too, keeping them phase-locked to within IRQ latency.
static void dma_irq_handler(void) {
    for (uint slot = 0; slot < NUM_BUFFERS; slot++) {
        uint chan = dma_crank_chan[slot];
        if (dma_hw->ints0 & (1u << chan)) {
            dma_hw->ints0 = 1u << chan;
            cycles_completed++;
            double scale = cycle_rpm_scale[cycles_completed % 2];
            fill_buffer_slot(slot, scale);

            dma_channel_set_read_addr(dma_crank_chan[slot], crank_events[slot], false);
            dma_channel_set_trans_count(dma_crank_chan[slot], crank_words_total, false);

            uint other = slot ^ 1; // NUM_BUFFERS == 2
            dma_channel_set_read_addr(dma_cam_chan[other], cam_events[other], false);
            dma_channel_set_trans_count(dma_cam_chan[other], CAM_WORDS_TOTAL, true);
        }
    }
}

void run_mode_continuous(void) {
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
    configure_ping_pong_channel(pio, sm_crank, crank_events[0], crank_words_total,
                                 dma_crank_chan[0], dma_crank_chan[1]);
    configure_ping_pong_channel(pio, sm_crank, crank_events[1], crank_words_total,
                                 dma_crank_chan[1], dma_crank_chan[0]);

    // Cam channels chain to themselves -- pico-sdk's documented way to
    // disable hardware auto-chaining (see dma_irq_handler() above for
    // why: cam is driven explicitly from the crank IRQ instead).
    dma_cam_chan[0] = dma_claim_unused_channel(true);
    dma_cam_chan[1] = dma_claim_unused_channel(true);
    configure_ping_pong_channel(pio, sm_cam, cam_events[0], CAM_WORDS_TOTAL,
                                 dma_cam_chan[0], dma_cam_chan[0]);
    configure_ping_pong_channel(pio, sm_cam, cam_events[1], CAM_WORDS_TOTAL,
                                 dma_cam_chan[1], dma_cam_chan[1]);

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
