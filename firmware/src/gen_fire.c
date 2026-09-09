#include "hardware/dma.h"

#include "gen_fire.h"

void configure_ping_pong_channel(PIO pio, uint sm, uint32_t *buf, uint32_t word_count,
                                  uint this_chan, uint chain_to_chan) {
    dma_channel_config cfg = dma_channel_get_default_config(this_chan);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&cfg, true);
    channel_config_set_write_increment(&cfg, false);
    channel_config_set_dreq(&cfg, pio_get_dreq(pio, sm, true));
    channel_config_set_chain_to(&cfg, chain_to_chan);
    dma_channel_configure(this_chan, &cfg, &pio->txf[sm], buf, word_count, false);
}
