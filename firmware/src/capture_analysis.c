#include <stdio.h>

#include "capture_analysis.h"

uint32_t capture_buf[CAPTURE_SAMPLES];
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

void print_channel_edges(uint ch, double sample_period_ms) {
    uint32_t prev = capture_buf[0] & (1u << ch);
    uint rising = 0, falling = 0;
    uint printed = 0;
    printf("  ch%u: ", ch);
    for (uint32_t i = 1; i < capture_samples_used; i++) {
        uint32_t cur = capture_buf[i] & (1u << ch);
        if (cur != prev) {
            bool is_rising = (cur != 0);
            if (is_rising) rising++; else falling++;
            if (printed < 4) {
                printf("%s@%.3fms ", is_rising ? "R" : "F", i * sample_period_ms);
                printed++;
            }
            prev = cur;
        }
    }
    printf("(rising=%u falling=%u)\n", rising, falling);
}

#define MAX_CRANK_RISING 200
#define MAX_CAM_EDGES 4

// Doc section 8: reference = first crank tooth after the missing-tooth
// gap = angle 0deg. C_rev = reference-to-reference duration (one full
// revolution, gap included). angle = 360 * C_event / C_rev, using each
// revolution's own measured C_rev rather than one fixed factor, so a
// constant-vs-varying RPM profile is handled the same way.
//
// This test's capture window starts exactly at generation's own start
// (tooth 1 of rev 0), so refs[0] = the very first crank rising edge, not
// a detected gap. Every later reference comes from finding the
// rising-to-rising interval that's a clear outlier (~3x a normal tooth
// period at the gap) vs. the smallest observed interval (~1 normal
// tooth). The last revolution in a one-shot capture has no trailing
// reference (generation just stops there, no further gap) -- for that
// window this falls back to reusing the previous revolution's C_rev,
// noted explicitly below; a continuous double-buffered capture would not
// have this edge case.
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
// full window's C_rev for a trailing timestamp with no closing
// reference (see find_crank_references' header comment).
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

    printf("crank references (angle=0deg points) found: %u\n", n_refs);
    for (uint i = 0; i < n_refs; i++) {
        printf("  ref[%u] @ %.3fms\n", i, refs[i]);
    }
    if (n_refs < 2) {
        printf("  not enough references to compute C_rev.\n");
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
    printf("cam pulse found in reference window %d (that window = rev0/0-360deg)\n", rev0_window);

    *n_refs_out = n_refs;
    *rev0_window_out = rev0_window;
    return true;
}

void angle_conversion_demo(double sample_period_ms, const double *refs, uint n_refs,
                            int rev0_window) {
    double cam_rise[MAX_CAM_EDGES], cam_fall[MAX_CAM_EDGES];
    uint n_cam_rise = find_edge_times(1, true, sample_period_ms, cam_rise, MAX_CAM_EDGES);
    uint n_cam_fall = find_edge_times(1, false, sample_period_ms, cam_fall, MAX_CAM_EDGES);
    if (n_cam_rise > MAX_CAM_EDGES) n_cam_rise = MAX_CAM_EDGES;
    if (n_cam_fall > MAX_CAM_EDGES) n_cam_fall = MAX_CAM_EDGES;

    printf("cam angles (expect ~120deg rise, ~300deg fall, matching doc section 9):\n");
    for (uint e = 0; e < n_cam_rise; e++) {
        double angle = convert_to_angle(cam_rise[e], refs, n_refs, rev0_window);
        printf("  rise@%.3fms -> %.2f deg\n", cam_rise[e], angle);
    }
    for (uint e = 0; e < n_cam_fall; e++) {
        double angle = convert_to_angle(cam_fall[e], refs, n_refs, rev0_window);
        printf("  fall@%.3fms -> %.2f deg\n", cam_fall[e], angle);
    }
}

#define MAX_EDGES_PER_CHECK 8

// Angle-wraps a possibly-negative or out-of-range revolution index back
// into [0, 360) by construction: convert_to_angle already returns
// rev_index*360 + angle_in_rev, so recovering rev_index is just floor/360.
static int angle_to_rev_index(double angle) {
    return (angle >= 0.0) ? (int)(angle / 360.0) : -1;
}

// Among angles[0..n), returns the index of the one in [cycle*360,
// cycle*360+360) closest to target_deg, or -1 if none are in that cycle.
static int pick_in_cycle(const double *angles, uint n, int cycle, double target_deg) {
    int best = -1;
    double best_diff = 1e18;
    for (uint i = 0; i < n; i++) {
        if (angle_to_rev_index(angles[i]) != cycle) continue;
        double diff = angles[i] - target_deg;
        if (diff < 0) diff = -diff;
        if (diff < best_diff) {
            best_diff = diff;
            best = (int)i;
        }
    }
    return best;
}

static uint count_in_cycle(const double *angles, uint n, int cycle) {
    uint c = 0;
    for (uint i = 0; i < n; i++) {
        if (angle_to_rev_index(angles[i]) == cycle) c++;
    }
    return c;
}

void evaluate_spec(const EcuOutputSpec *spec, double sample_period_ms,
                    const double *refs, uint n_refs, int rev0_window) {
    double rise_t[MAX_EDGES_PER_CHECK], fall_t[MAX_EDGES_PER_CHECK];
    uint n_rise = find_edge_times(spec->channel, true, sample_period_ms, rise_t, MAX_EDGES_PER_CHECK);
    uint n_fall = find_edge_times(spec->channel, false, sample_period_ms, fall_t, MAX_EDGES_PER_CHECK);
    if (n_rise > MAX_EDGES_PER_CHECK) n_rise = MAX_EDGES_PER_CHECK;
    if (n_fall > MAX_EDGES_PER_CHECK) n_fall = MAX_EDGES_PER_CHECK;

    double rise_a[MAX_EDGES_PER_CHECK], fall_a[MAX_EDGES_PER_CHECK];
    for (uint i = 0; i < n_rise; i++) rise_a[i] = convert_to_angle(rise_t[i], refs, n_refs, rev0_window);
    for (uint i = 0; i < n_fall; i++) fall_a[i] = convert_to_angle(fall_t[i], refs, n_refs, rev0_window);

    printf("[%s] ch%u expect rise=%.1f+-%.1fdeg fall=%.1f+-%.1fdeg cycle=%d:\n",
           spec->name, spec->channel, spec->rise_deg, spec->rise_tol_deg,
           spec->fall_deg, spec->fall_tol_deg, spec->expected_cycle);

    bool pass = true;

    int ri = pick_in_cycle(rise_a, n_rise, spec->expected_cycle, spec->rise_deg);
    if (ri < 0) {
        uint elsewhere = n_rise - count_in_cycle(rise_a, n_rise, spec->expected_cycle);
        if (n_rise == 0) {
            printf("  FAIL: missing rising edge\n");
        } else if (elsewhere > 0) {
            printf("  FAIL: rising edge active in wrong revolution (expected cycle %d)\n",
                   spec->expected_cycle);
        } else {
            printf("  FAIL: missing rising edge in expected cycle\n");
        }
        pass = false;
    } else {
        double err = rise_a[ri] - spec->rise_deg;
        if (err < 0) err = -err;
        if (err > spec->rise_tol_deg) {
            printf("  FAIL: rising edge at %.2fdeg, out of tolerance\n", rise_a[ri]);
            pass = false;
        }
        if (count_in_cycle(rise_a, n_rise, spec->expected_cycle) > 1) {
            printf("  FAIL: extra rising edge(s) in expected cycle\n");
            pass = false;
        }
    }

    int fi = pick_in_cycle(fall_a, n_fall, spec->expected_cycle, spec->fall_deg);
    if (fi < 0) {
        uint elsewhere = n_fall - count_in_cycle(fall_a, n_fall, spec->expected_cycle);
        if (n_fall == 0) {
            printf("  FAIL: missing falling edge\n");
        } else if (elsewhere > 0) {
            printf("  FAIL: falling edge active in wrong revolution (expected cycle %d)\n",
                   spec->expected_cycle);
        } else {
            printf("  FAIL: missing falling edge in expected cycle\n");
        }
        pass = false;
    } else {
        double err = fall_a[fi] - spec->fall_deg;
        if (err < 0) err = -err;
        if (err > spec->fall_tol_deg) {
            printf("  FAIL: falling edge at %.2fdeg, out of tolerance\n", fall_a[fi]);
            pass = false;
        }
        if (count_in_cycle(fall_a, n_fall, spec->expected_cycle) > 1) {
            printf("  FAIL: extra falling edge(s) in expected cycle\n");
            pass = false;
        }
    }

    if (ri >= 0 && fi >= 0) {
        if (fall_a[fi] < rise_a[ri]) {
            printf("  FAIL: incorrect polarity (falling edge before rising edge)\n");
            pass = false;
        } else {
            double width = fall_a[fi] - rise_a[ri];
            double expected_width = spec->fall_deg - spec->rise_deg;
            double width_tol = spec->rise_tol_deg + spec->fall_tol_deg;
            double width_err = width - expected_width;
            if (width_err < 0) width_err = -width_err;
            if (width_err > width_tol) {
                printf("  FAIL: pulse width %.2fdeg, out of tolerance (expected %.2fdeg)\n",
                       width, expected_width);
                pass = false;
            }
        }
    }

    printf("  %s\n", pass ? "PASS" : "-> FAIL (see above)");
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
