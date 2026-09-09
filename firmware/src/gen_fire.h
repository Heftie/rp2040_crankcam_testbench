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

#endif
