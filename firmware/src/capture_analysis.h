// Mode 3 support: raw capture buffer, edge/angle conversion (doc section
// 8), and pass/fail evaluation against expected ECU-output windows (doc
// section 9).
#ifndef CAPTURE_ANALYSIS_H
#define CAPTURE_ANALYSIS_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/types.h"

#define CAPTURE_BASE_PIN 6
#define CAPTURE_PIN_COUNT 6
#define CAPTURE_SAMPLE_HZ 100000.0
#define CAPTURE_DURATION_MS 150.0
// Plain integer literal (not computed via a float macro): a float-cast
// expression here previously made GCC treat capture_buf as a "variably
// modified" array at file scope (UB for a static array), which silently
// crashed at boot. Keep in sync with CAPTURE_SAMPLE_HZ*CAPTURE_DURATION_MS/1000.
#define CAPTURE_SAMPLES 15000u

#define MAX_REFS 8

// Points at whichever physical buffer holds the capture to analyze right
// now. Defaults to an internal CAPTURE_SAMPLES-capacity buffer that mode
// 3 uses as-is; mode 4's continuous double-buffered capture instead owns
// two of its own physical buffers and repoints capture_buf at whichever
// one just finished before calling any function below (its DMA writes
// directly into that physical buffer -- no copy).
extern uint32_t *capture_buf;

// Number of capture_buf[] entries actually holding this capture's samples
// (<= CAPTURE_SAMPLES). Every DMA-into-capture_buf call site must set
// this to its own transfer length before calling any function below --
// they all scan capture_buf[0..capture_samples_used) rather than the
// full buffer, so a shorter capture doesn't pick up a previous, longer
// capture's stale trailing samples.
extern uint32_t capture_samples_used;

// Prints each channel's first few edges plus rising/falling edge counts.
void print_channel_edges(uint ch, double sample_period_ms);

// Doc section 8: reference = first crank tooth after the missing-tooth
// gap = angle 0deg. Finds crank reference points and identifies which
// window is rev0 (the one containing the cam pulse). Returns false if
// there weren't enough crank edges to get references.
bool compute_crank_reference(double sample_period_ms, double *refs, uint *n_refs_out,
                              int *rev0_window_out);

// Prints cam rise/fall timestamps converted to crank angle, using refs
// from compute_crank_reference.
void angle_conversion_demo(double sample_period_ms, const double *refs, uint n_refs,
                            int rev0_window);

// Doc section 9: an ECU output is a rise-then-fall pulse at known angles,
// in a known 720deg revolution, within a tolerance.
typedef struct {
    const char *name;
    uint channel;
    double rise_deg, rise_tol_deg;
    double fall_deg, fall_tol_deg;
    int expected_cycle; // 0 or 1
} EcuOutputSpec;

// Checks spec's channel against captured edges (converted via refs) and
// prints PASS/FAIL with the specific reason(s).
void evaluate_spec(const EcuOutputSpec *spec, double sample_period_ms,
                    const double *refs, uint n_refs, int rev0_window);

// Single-pulse-per-channel live report: for each of channel_count
// channels, prints the first rising and falling edge angle found (or
// "not detected" if neither is present). No tolerance/pass-fail --
// just what was captured, for a per-cycle live feed.
void report_pulse_angles(uint channel_count, double sample_period_ms,
                          const double *refs, uint n_refs, int rev0_window);

#endif
