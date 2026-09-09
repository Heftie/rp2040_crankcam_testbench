// Command-driven crank/cam generation + capture engine. Replaces the old
// boot-time "pick one mode, runs forever" design: generation and capture
// are independently started/stopped at any time from main.c's command
// loop, and RPM is changeable while generation is running.
//
// State machine: capture requires generation to already be running
// (there is no crank reference to measure angles against otherwise), and
// stopping generation stops capture with it. Profile changes are only
// allowed while generation is stopped (event-table sizes/geometry are
// derived from the profile once, not recomputed live).
#ifndef ENGINE_H
#define ENGINE_H

#include <stdbool.h>
#include <stdint.h>
#include "pico/types.h"
#include "profiles.h"

// Sane RPM bounds for the 'r' command. ENGINE_MAX_RPM is a generous
// sanity ceiling (event cycle counts stay far above EVENT_MIN_CYCLES up
// to this point for every current profile); CAPTURE_MIN_RPM is the floor
// live capture's fixed-size buffer supports (see LIVE_MAX_CYCLE_SAMPLES
// in engine.c) -- generation alone has no such floor.
#define ENGINE_MAX_RPM 20000u
#define CAPTURE_MIN_RPM 1000u

// One-time hardware setup: claims PIO state machines/DMA channels for
// generation (pio0) and capture (pio1), installs the shared DMA IRQ
// handler. Call once at boot before any other engine_* function.
void engine_init(void);

// Selects the active trigger-wheel profile (1-based index into
// crankcam_profiles[]). Fails (returns false, no change) if index is out
// of range or generation is currently running.
bool engine_select_profile(uint index);

// Sets the constant RPM generation runs at. If generation is running,
// the new value is picked up at the next 720deg cycle boundary (same
// buffer-refill mechanism as the RPM ramp this replaced); otherwise it
// takes effect the next time engine_start_gen() is called. Fails
// (returns false, no change) if rpm is 0 or above ENGINE_MAX_RPM.
bool engine_set_rpm(uint32_t rpm);

// Starts continuous double-buffered crank/cam generation at the current
// profile/RPM. No-op if already running.
void engine_start_gen(void);

// Stops generation. Also stops capture first if it was running, since
// capture without generation has no crank edges to measure angles from.
void engine_stop_gen(void);

// Starts the continuous capture+live-angle-report loop. Fails (returns
// false) if generation isn't running or the current RPM is below
// CAPTURE_MIN_RPM. No-op (returns true) if capture is already running.
bool engine_start_capture(void);

// Stops capture; leaves generation running.
void engine_stop_capture(void);

bool engine_gen_running(void);
bool engine_capture_running(void);
uint32_t engine_rpm(void);
const CrankCamProfile *engine_profile(void);

// Call once per main-loop iteration. Prints one "cycle N:" report (see
// capture_analysis.h's report_pulse_angles) if a capture buffer has
// finished since the last call; otherwise returns immediately. No-op if
// capture isn't running.
void engine_poll_capture(void);

#endif
