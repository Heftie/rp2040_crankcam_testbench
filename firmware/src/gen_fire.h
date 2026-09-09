// Shared TX ping-pong DMA config for continuous crank/cam generation.
#ifndef GEN_FIRE_H
#define GEN_FIRE_H

#include "pico/types.h"
#include "hardware/pio.h"

// Configures (but does not start) a DMA channel to stream `buf`
// (word_count words) to the SM's TX FIFO once, then chain-trigger
// `chain_to_chan` on completion. Passing `this_chan` as `chain_to_chan`
// disables hardware chaining (pico-sdk's documented idiom) for a channel
// meant to be driven explicitly instead of self-chaining -- see
// engine.c's cam channels for why that's sometimes necessary.
void configure_ping_pong_channel(PIO pio, uint sm, uint32_t *buf, uint32_t word_count,
                                  uint this_chan, uint chain_to_chan);

#endif
