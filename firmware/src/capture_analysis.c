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

// Finds which reference window [refs[w], refs[w+1]) contains t, with a
// trailing extension past the last window for a timestamp with no
// closing reference yet -- each capture buffer covers ~1 cycle, so the
// final revolution's closing gap edge often lands just past this
// buffer's end. Returns the window index and, via *extra_revs_out, how
// many whole extra revolutions (of that window's own measured length)
// t is estimated to be past the window's start -- 0 for anything
// actually inside the window, potentially >0 in the trailing case.
// Returns -1 (before refs[0], or no windows at all) if t matches nothing.
static int find_window(double t, const double *refs, uint n_refs, int *extra_revs_out) {
    for (uint w = 0; w + 1 < n_refs; w++) {
        bool in_window = (t >= refs[w] && t < refs[w + 1]);
        bool in_trailing = (w + 2 == n_refs && t >= refs[w + 1]);
        if (in_window || in_trailing) {
            double c_rev = refs[w + 1] - refs[w];
            double c_event = t - refs[w];
            *extra_revs_out = (int)(c_event / c_rev); // 0 unless in_trailing pushed past a whole c_rev
            return (int)w;
        }
    }
    return -1;
}

// Converts timestamp t (ms) to an absolute crank angle (deg) using the
// reference windows in refs[0..n_refs-1]. rev0_window is the index of
// the window that contains the cam pulse (angle 0-360deg); other
// windows are numbered relative to it (unchanged from before). What's
// new is subtracting find_window()'s extra_revs from angle_in_rev
// itself, keeping it within its own 0-360deg span even when t is a
// trailing timestamp that's actually landed a whole revolution (or
// more) past window w's start -- without this, such a timestamp's angle
// was reported bumped up by a full 360deg per revolution it had already
// crossed (e.g. cam's own pulse, 120/300deg, reported as 480/660deg
// whenever the last captured crank reference happened to fall before
// it). rev0_window itself is computed the same way (see
// compute_crank_reference), from the exact same window index w this
// returns for that same cam edge -- so w - rev0_window is always exactly
// 0 for cam's own angle, regardless of extra_revs.
static double convert_to_angle(double t, const double *refs, uint n_refs, int rev0_window) {
    int extra_revs = 0;
    int w = find_window(t, refs, n_refs, &extra_revs);
    if (w < 0) return -1.0;

    double c_rev = refs[w + 1] - refs[w];
    double c_event = t - refs[w];
    double angle_in_rev = 360.0 * (c_event / c_rev - extra_revs);
    int rev_index = (rev0_window >= 0) ? (w - rev0_window) : 0;
    return angle_in_rev + rev_index * 360.0;
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

    // Which reference window contains the cam pulse identifies rev 0
    // (cam is silent through rev 1 by design). Uses find_window() --
    // the same lookup report_pulse_angles' later convert_to_angle() call
    // for this exact same edge will use -- so the two can never disagree
    // about which window this is; only the first rising edge is used,
    // since find_edge_times(..., max_out=1) is also what report_pulse_angles
    // uses for channel 1's own rise, so both see the identical timestamp.
    double cam_rise;
    uint n_cam_rise = find_edge_times(1, true, sample_period_ms, &cam_rise, 1);
    int rev0_window = -1;
    if (n_cam_rise > 0) {
        int extra_revs;
        rev0_window = find_window(cam_rise, refs, n_refs, &extra_revs);
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
        // convert_to_angle() returns -1.0 as a "no window contains this
        // timestamp" sentinel (e.g. an edge before refs[0] -- plausible
        // for ch0 itself if the capture buffer happens to start mid-pulse,
        // seeing a falling edge before any rising one). Print "--" rather
        // than leak that sentinel as if it were a real angle.
        if (n_rise > 0) {
            double a = convert_to_angle(rise_t, refs, n_refs, rev0_window);
            if (a < 0.0) printf(" rise=--"); else printf(" rise=%.2fdeg", a);
        } else {
            printf(" rise=--");
        }
        if (n_fall > 0) {
            double a = convert_to_angle(fall_t, refs, n_refs, rev0_window);
            if (a < 0.0) printf(" fall=--"); else printf(" fall=%.2fdeg", a);
        } else {
            printf(" fall=--");
        }
        printf("\n");
    }
}
