#include <stdio.h>

#include "capture_analysis.h"

uint32_t *capture_buf = NULL; // engine.c repoints this before every use
uint32_t capture_samples_used;

// Collects up to max_out timestamps (ms) of edges on `ch` matching
// want_rising. Returns the count found (may exceed max_out; only the
// first max_out are stored).
static uint find_edge_times(uint ch, bool want_rising, double sample_period_ms,
                             double *out, uint max_out) {
    uint32_t prev = capture_buf[0] & (1u << ch);
    uint count = 0;
    for (uint32_t i = 1; i < capture_samples_used; i++) {
        uint32_t cur = capture_buf[i] & (1u << ch);
        if (cur != prev) {
            bool is_rising = (cur != 0);
            if (is_rising == want_rising && count < max_out) {
                out[count] = i * sample_period_ms;
            }
            if (is_rising == want_rising) count++;
            prev = cur;
        }
    }
    return count;
}

#define MAX_CRANK_RISING 200
#define MAX_CAM_EDGES 4

// Doc section 8: reference = first crank tooth after the missing-tooth
// gap = angle 0deg. C_rev = reference-to-reference duration (one full
// revolution, gap included). angle = 360 * C_event / C_rev, using each
// revolution's own measured C_rev rather than one fixed factor, so a
// constant-vs-varying RPM profile is handled the same way.
//
// refs[0] is just the first crank rising edge seen in this capture
// window, not a detected gap -- every later reference comes from finding
// the rising-to-rising interval that's a clear outlier (~3x a normal
// tooth period at the gap) vs. the smallest observed interval (~1 normal
// tooth). Continuous capture means there's always a next cycle's data
// following, so (unlike a one-shot capture) there's no trailing-window
// edge case to fall back for.
static uint find_crank_references(const double *rising, uint n, double *refs, uint max_refs) {
    if (n < 2 || max_refs == 0) return 0;
    double min_interval = rising[1] - rising[0];
    for (uint i = 1; i + 1 < n; i++) {
        double iv = rising[i + 1] - rising[i];
        if (iv < min_interval) min_interval = iv;
    }
    uint count = 0;
    refs[count++] = rising[0];
    for (uint i = 0; i + 1 < n && count < max_refs; i++) {
        double iv = rising[i + 1] - rising[i];
        if (iv > 2.0 * min_interval) {
            refs[count++] = rising[i + 1];
        }
    }
    return count;
}

// Converts timestamp t (ms) to an absolute crank angle (deg) using the
// reference windows in refs[0..n_refs-1]. rev0_window is the index of
// the window that contains the cam pulse (angle 0-360deg); other
// windows are numbered relative to it. Falls back to reusing the last
// full window's C_rev for a trailing timestamp with no closing reference
// -- each capture buffer covers ~1 cycle, so the final revolution's
// closing gap edge often lands just past this buffer's end.
static double convert_to_angle(double t, const double *refs, uint n_refs, int rev0_window) {
    for (uint w = 0; w + 1 < n_refs; w++) {
        bool in_window = (t >= refs[w] && t < refs[w + 1]);
        bool in_trailing = (w + 2 == n_refs && t >= refs[w + 1]);
        if (in_window || in_trailing) {
            double c_rev = refs[w + 1] - refs[w];
            double c_event = t - refs[w];
            double angle_in_rev = 360.0 * c_event / c_rev;
            int rev_index = (rev0_window >= 0) ? ((int)w - rev0_window) : 0;
            return angle_in_rev + rev_index * 360.0;
        }
    }
    return -1.0; // before refs[0] or no windows at all
}

bool compute_crank_reference(double sample_period_ms, double *refs, uint *n_refs_out,
                              int *rev0_window_out) {
    static double crank_rising[MAX_CRANK_RISING];
    uint n_rising = find_edge_times(0, true, sample_period_ms, crank_rising, MAX_CRANK_RISING);
    if (n_rising > MAX_CRANK_RISING) n_rising = MAX_CRANK_RISING;
    uint n_refs = find_crank_references(crank_rising, n_rising, refs, MAX_REFS);

    if (n_refs < 2) {
        *n_refs_out = n_refs;
        *rev0_window_out = -1;
        return false;
    }

    double cam_rise[MAX_CAM_EDGES];
    uint n_cam_rise = find_edge_times(1, true, sample_period_ms, cam_rise, MAX_CAM_EDGES);
    if (n_cam_rise > MAX_CAM_EDGES) n_cam_rise = MAX_CAM_EDGES;

    // Which reference window contains the cam pulse identifies rev 0
    // (cam is silent through rev 1 by design).
    int rev0_window = -1;
    for (uint w = 0; w + 1 < n_refs && rev0_window < 0; w++) {
        for (uint e = 0; e < n_cam_rise; e++) {
            if (cam_rise[e] >= refs[w] && cam_rise[e] < refs[w + 1]) {
                rev0_window = (int)w;
                break;
            }
        }
    }

    *n_refs_out = n_refs;
    *rev0_window_out = rev0_window;
    return true;
}

void report_pulse_angles(uint channel_count, double sample_period_ms,
                          const double *refs, uint n_refs, int rev0_window) {
    for (uint ch = 0; ch < channel_count; ch++) {
        double rise_t, fall_t;
        uint n_rise = find_edge_times(ch, true, sample_period_ms, &rise_t, 1);
        uint n_fall = find_edge_times(ch, false, sample_period_ms, &fall_t, 1);

        if (n_rise == 0 && n_fall == 0) {
            printf("  ch%u: not detected\n", ch);
            continue;
        }

        printf("  ch%u:", ch);
        if (n_rise > 0) {
            printf(" rise=%.2fdeg", convert_to_angle(rise_t, refs, n_refs, rev0_window));
        } else {
            printf(" rise=--");
        }
        if (n_fall > 0) {
            printf(" fall=%.2fdeg", convert_to_angle(fall_t, refs, n_refs, rev0_window));
        } else {
            printf(" fall=--");
        }
        printf("\n");
    }
}
