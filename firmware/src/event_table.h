// Event-table encoding and RPM-to-cycle-count conversion for the
// currently selected crank/cam trigger-wheel profile (see profiles.h).
// select_profile() must be called once, before build_position_cycles_*/
// build_crank_events/build_cam_events are used. Used by engine.c.
#ifndef EVENT_TABLE_H
#define EVENT_TABLE_H

#include <stdint.h>
#include "pico/types.h"
#include "profiles.h"

#define CRANK_PIN 2
#define CAM_PIN 3

// Largest teeth_per_rev any profiles.c entry may use -- sizes the static
// event buffers below. select_profile() asserts against this.
#define MAX_TEETH_PER_REV 60
#define REVS_PER_CYCLE 2 // 720deg four-stroke cycle = 2 crank revs

// Worst case per profile: 0 missing teeth (every position is a tooth =
// 2 edges) + 1 gap event/rev (unused but harmless when missing=0).
#define MAX_CRANK_EVENTS_PER_REV (MAX_TEETH_PER_REV * 2 + 1)
#define MAX_CRANK_EVENTS_TOTAL (MAX_CRANK_EVENTS_PER_REV * REVS_PER_CYCLE)
#define MAX_CRANK_WORDS_TOTAL (MAX_CRANK_EVENTS_TOTAL * 2)
#define MAX_POSITIONS_TOTAL (MAX_TEETH_PER_REV * REVS_PER_CYCLE)

// Largest cam_pulse_count any profiles.c entry may use -- sizes the
// static cam event buffer below. select_profile() asserts a profile's
// cam_pulse_count against it -- raise it if you add a cam wheel with more
// pulses.
#define MAX_CAM_PULSES 12

// Worst case: every pulse contributes a low-then-high event pair, plus
// one trailing low event after the last pulse.
#define MAX_CAM_EVENTS_TOTAL (MAX_CAM_PULSES * 2 + 1)
#define MAX_CAM_WORDS_TOTAL (MAX_CAM_EVENTS_TOTAL * 2)

#define EVENT_MIN_CYCLES 3 // 2 `out` + at least 1 delay_loop iteration
#define NUM_BUFFERS 2

extern uint32_t crank_events[NUM_BUFFERS][MAX_CRANK_WORDS_TOTAL];
extern uint32_t cam_events[NUM_BUFFERS][MAX_CAM_WORDS_TOTAL];
extern uint32_t position_cycles[MAX_POSITIONS_TOTAL];
extern double f_pio_hz; // set by each mode from clock_get_hz(clk_sys)/clkdiv

extern uint positions_total;    // current_profile.teeth_per_rev * REVS_PER_CYCLE
extern uint32_t crank_words_total; // word count build_crank_events wrote last
extern uint32_t cam_words_total;   // word count build_cam_events wrote last

// Selects the active trigger-wheel profile: derives positions_total,
// per-position degree pitch, and cam edge positions from it. Call once
// at boot before any build_* function.
void select_profile(const CrankCamProfile *profile);

// Fills crank_events[buf]/cam_events[buf]-sized event tables from the
// current position_cycles[] contents. build_crank_events also updates
// crank_words_total to the number of words it wrote (varies with the
// selected profile's teeth/missing-teeth count); build_cam_events
// likewise updates cam_words_total (varies with the selected profile's
// cam_pulse_count).
void build_crank_events(uint32_t *buf);
void build_cam_events(uint32_t *buf);

// Fills position_cycles[] with a single constant value -- every position
// takes the same time, i.e. steady-state RPM with no ramp.
void build_position_cycles_constant(double rpm);

// Recomputes both crank_events[slot] and cam_events[slot] from one shared
// constant-RPM position_cycles[] pass, keeping crank/cam phase-locked by
// construction. Called once per 720deg cycle boundary by engine.c's DMA
// IRQ handler, using whatever RPM is current at that moment -- this is
// what makes engine_set_rpm() take effect live.
void fill_buffer_slot_constant(uint slot, double rpm);

#endif
