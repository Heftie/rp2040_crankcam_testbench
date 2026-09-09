// One-shot crank/cam generator firing, shared by mode 2 (single-shot
// diagnostic) and mode 3 (capture test setup step for mode 3 lives in
// mode_capture.c since it interleaves with capture arming).
#ifndef GEN_FIRE_H
#define GEN_FIRE_H

#include "pico/types.h"
#include "hardware/pio.h"

// Resets both SMs/DMA to a clean state, rebuilds crank_events[0]/
// cam_events[0]-fed DMA channels, and fires them phase-locked.
void fire_gen_one_shot(PIO pio, uint sm_crank, uint sm_cam, uint offset, float clkdiv,
                        uint dma_crank_chan, uint dma_cam_chan);

// Configures (but does not start) a DMA channel to stream `buf`
// (word_count words) to the SM's TX FIFO once, then chain-trigger
// `chain_to_chan` on completion. Passing `this_chan` as `chain_to_chan`
// disables hardware chaining (pico-sdk's documented idiom) for a channel
// meant to be driven explicitly instead of self-chaining -- see
// mode_continuous.c's cam channels for why that's sometimes necessary.
// Shared by modes 1 and 4 (both run continuous double-buffered crank/cam
// generation).
void configure_ping_pong_channel(PIO pio, uint sm, uint32_t *buf, uint32_t word_count,
                                  uint this_chan, uint chain_to_chan);

#endif
