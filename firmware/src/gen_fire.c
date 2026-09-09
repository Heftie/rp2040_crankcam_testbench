#include "hardware/dma.h"

#include "gen_fire.h"
#include "event_gen.pio.h"
#include "event_table.h"

// Shared by modes 2 and 3 (same one-shot generator).
void fire_gen_one_shot(PIO pio, uint sm_crank, uint sm_cam, uint offset, float clkdiv,
                        uint dma_crank_chan, uint dma_cam_chan) {
    // Clean up any previous shot first: by now it has long since finished
    // and stalled on its own (see note below), so this is a harmless
    // reset, not a mid-flight interruption.
    pio_sm_set_enabled(pio, sm_crank, false);
    pio_sm_set_enabled(pio, sm_cam, false);
    dma_channel_abort(dma_crank_chan);
    dma_channel_abort(dma_cam_chan);
    pio_sm_clear_fifos(pio, sm_crank);
    pio_sm_clear_fifos(pio, sm_cam);

    // Fully re-init both SMs: resets PC to program start and clears
    // OSR/X/Y, so every shot starts from the same clean state.
    event_gen_program_init(pio, sm_crank, offset, CRANK_PIN, clkdiv);
    event_gen_program_init(pio, sm_cam, offset, CAM_PIN, clkdiv);

    dma_channel_config cfg_crank = dma_channel_get_default_config(dma_crank_chan);
    channel_config_set_transfer_data_size(&cfg_crank, DMA_SIZE_32);
    channel_config_set_read_increment(&cfg_crank, true);
    channel_config_set_write_increment(&cfg_crank, false);
    channel_config_set_dreq(&cfg_crank, pio_get_dreq(pio, sm_crank, true));
    dma_channel_configure(dma_crank_chan, &cfg_crank, &pio->txf[sm_crank],
                           crank_events[0], CRANK_WORDS_TOTAL, false);

    dma_channel_config cfg_cam = dma_channel_get_default_config(dma_cam_chan);
    channel_config_set_transfer_data_size(&cfg_cam, DMA_SIZE_32);
    channel_config_set_read_increment(&cfg_cam, true);
    channel_config_set_write_increment(&cfg_cam, false);
    channel_config_set_dreq(&cfg_cam, pio_get_dreq(pio, sm_cam, true));
    dma_channel_configure(dma_cam_chan, &cfg_cam, &pio->txf[sm_cam],
                           cam_events[0], CAM_WORDS_TOTAL, false);

    dma_channel_start(dma_crank_chan);
    dma_channel_start(dma_cam_chan);

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
