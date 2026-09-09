// Shared crank/cam geometry, event-table encoding, and RPM-profile-to-
// cycle-count conversion. Used by all three firmware modes.
#ifndef EVENT_TABLE_H
#define EVENT_TABLE_H

#include <stdint.h>
#include "pico/types.h"

// ===================== Shared crank/cam config =====================

#define CRANK_PIN 2
#define CAM_PIN 3

#define TEETH_PER_REV 60
#define MISSING_TEETH 2
#define REAL_TEETH_PER_REV (TEETH_PER_REV - MISSING_TEETH) // 58
#define REVS_PER_CYCLE 2
#define POSITIONS_PER_REV TEETH_PER_REV                      // 60
#define POSITIONS_TOTAL (POSITIONS_PER_REV * REVS_PER_CYCLE) // 120
#define DEG_PER_POSITION (360.0 / POSITIONS_PER_REV)         // 6 deg

#define CRANK_EVENTS_PER_REV (REAL_TEETH_PER_REV * 2 + 1) // 116 edges + 1 gap = 117
#define CRANK_EVENTS_TOTAL (CRANK_EVENTS_PER_REV * REVS_PER_CYCLE) // 234
#define CRANK_WORDS_TOTAL (CRANK_EVENTS_TOTAL * 2) // 468

// Cam: one pulse per 720deg cycle, 120-300deg, aligned to positions 20/50
// (120/6=20, 300/6=50) so it reuses crank's own per-position cycle counts.
#define CAM_RISE_POSITION 20
#define CAM_FALL_POSITION 50
#define CAM_EVENTS_TOTAL 3
#define CAM_WORDS_TOTAL (CAM_EVENTS_TOTAL * 2)

#define EVENT_MIN_CYCLES 3 // 2 `out` + at least 1 delay_loop iteration
#define NUM_BUFFERS 2
#define CONSTANT_RPM 1000.0 // used by modes 2 and 3

extern uint32_t crank_events[NUM_BUFFERS][CRANK_WORDS_TOTAL];
extern uint32_t cam_events[NUM_BUFFERS][CAM_WORDS_TOTAL];
extern uint32_t position_cycles[POSITIONS_TOTAL];
extern double f_pio_hz; // set by each mode from clock_get_hz(clk_sys)/clkdiv

// Fills crank_events[buf]/cam_events[buf]-sized event tables from the
// current position_cycles[] contents.
void build_crank_events(uint32_t *buf);
void build_cam_events(uint32_t *buf);

// Fills position_cycles[] with a single constant value (modes 2 and 3:
// no RPM ramp, simplest possible signal for scope/capture validation).
void build_position_cycles_constant(double rpm);

// Recomputes both crank_events[slot] and cam_events[slot] from one shared
// position_cycles[] pass (built from the section-3 RPM profile, scaled by
// `scale`), keeping crank/cam phase-locked by construction.
void fill_buffer_slot(uint slot, double scale);

#endif
