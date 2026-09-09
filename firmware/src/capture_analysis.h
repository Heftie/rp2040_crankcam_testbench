// Live-capture support: crank-angle reference detection (doc section 8)
// and per-cycle rise/fall angle reporting, used by engine.c.
#ifndef CAPTURE_ANALYSIS_H
#define CAPTURE_ANALYSIS_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/types.h"

#define CAPTURE_BASE_PIN 6
#define CAPTURE_PIN_COUNT 6
#define CAPTURE_SAMPLE_HZ 100000.0

#define MAX_REFS 8

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

// Doc section 8: reference = first crank tooth after the missing-tooth
// gap = angle 0deg. Finds crank reference points and identifies which
// window is rev0 (the one containing the cam pulse). Returns false if
// there weren't enough crank edges to get references.
bool compute_crank_reference(double sample_period_ms, double *refs, uint *n_refs_out,
                              int *rev0_window_out);

// Single-pulse-per-channel live report: for each of channel_count
// channels, prints the first rising and falling edge angle found (or
// "not detected" if neither is present). No tolerance/pass-fail --
// just what was captured, for a per-cycle live feed.
void report_pulse_angles(uint channel_count, double sample_period_ms,
                          const double *refs, uint n_refs, int rev0_window);

#endif
