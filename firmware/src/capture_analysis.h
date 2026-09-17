// Live-capture support: per-cycle rise/fall angle reporting, used by
// engine.c. Angle reference comes from the generation engine's own
// timebase (CycleBoundary, latched by engine.c every 720deg cycle) --
// not from decoding a captured crank/cam edge, so no channel is special
// and nothing needs to be wired back to the crank/cam outputs for angle
// conversion to work.
#ifndef CAPTURE_ANALYSIS_H
#define CAPTURE_ANALYSIS_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/types.h"

#define CAPTURE_BASE_PIN 6
#define CAPTURE_PIN_COUNT 6
#define CAPTURE_SAMPLE_HZ 100000.0

// Points at whichever physical capture buffer (engine.c owns two, for
// continuous ping-pong capture) holds the data to analyze right now.
// engine.c repoints this at whichever one just finished before calling
// any function below -- no copy.
extern uint32_t *capture_buf;

// Number of capture_buf[] entries actually holding this cycle's samples.
// Every edge-scanning function below loops over this, not a fixed size,
// so a shorter capture (faster RPM) doesn't pick up a previous, longer
// capture's stale trailing samples. engine.c must set this before calling
// any function below.
extern uint32_t capture_samples_used;

// One generation-cycle boundary: engine.c latches one of these every time
// a new 720deg cycle starts playing (crank DMA completion IRQ, which
// stays paced by real playback -- see CLAUDE.md). start_us is the
// hardware timebase (time_us_64() domain) at that instant; rpm is the
// constant RPM that cycle plays at (position_cycles is rebuilt fresh per
// cycle -- event_table.c -- so RPM is exactly constant within one
// boundary's cycle, making angle a linear function of elapsed time).
typedef struct {
    uint64_t start_us;
    uint32_t rpm;
} CycleBoundary;

// Single-pulse-per-channel live report: for each of channel_count
// channels, prints the first rising and falling edge angle found (or
// "not detected" if neither is present). capture_start_us is the
// timebase instant the analyzed capture buffer's sample 0 was taken
// (engine.c latches this the same way as CycleBoundary, at capture DMA
// start/rearm); sample_period_us converts a sample index to an absolute
// timestamp. boundaries must be time-ordered oldest-first, most recent
// last. No tolerance/pass-fail -- just what was captured, for a
// per-cycle live feed.
void report_pulse_angles(uint channel_count, uint64_t capture_start_us, double sample_period_us,
                          const CycleBoundary *boundaries, uint n_boundaries);

#endif
