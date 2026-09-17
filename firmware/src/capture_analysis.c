#include <stdio.h>

#include "capture_analysis.h"
#include "event_table.h" // REVS_PER_CYCLE

uint32_t *capture_buf = NULL; // engine.c repoints this before every use
uint32_t capture_samples_used;

// Finds the first sample index (1..capture_samples_used-1) where channel
// ch transitions to want_rising's polarity. Returns false if no such
// transition exists in this capture buffer.
static bool find_first_edge(uint ch, bool want_rising, uint32_t *idx_out) {
    uint32_t prev = capture_buf[0] & (1u << ch);
    for (uint32_t i = 1; i < capture_samples_used; i++) {
        uint32_t cur = capture_buf[i] & (1u << ch);
        if (cur != prev) {
            bool is_rising = (cur != 0);
            prev = cur;
            if (is_rising == want_rising) {
                *idx_out = i;
                return true;
            }
        }
    }
    return false;
}

static double cycle_duration_us(uint32_t rpm) {
    return REVS_PER_CYCLE * 60000000.0 / (double)rpm;
}

// Doc section 8, timebase version: rather than decoding a reference edge
// out of a captured crank/cam signal, the reference is the generation
// engine's own record of when each 720deg cycle started and how fast it
// was playing (CycleBoundary) -- the firmware already knows this exactly,
// since it's the one driving crank/cam out in the first place. Walks
// boundaries newest-first (the common case, t inside the current cycle,
// resolves in one step) to find the one t belongs to, then converts the
// elapsed time since that boundary into an angle. RPM is constant within
// one boundary's cycle by construction, so this is an exact linear
// mapping, not an approximation -- and it stays correct across any
// number of whole cycles elapsed since that boundary (the trailing case
// the old edge-based lookup needed special handling for), since only the
// fractional part of elapsed/duration matters.
static double convert_time_to_angle(uint64_t t_us, const CycleBoundary *boundaries, uint n_boundaries) {
    for (int i = (int)n_boundaries - 1; i >= 0; i--) {
        if (t_us >= boundaries[i].start_us) {
            double dur_us = cycle_duration_us(boundaries[i].rpm);
            double elapsed_us = (double)(t_us - boundaries[i].start_us);
            double revs_elapsed = elapsed_us / dur_us;
            double frac = revs_elapsed - (double)(int64_t)revs_elapsed;
            return frac * 360.0 * REVS_PER_CYCLE;
        }
    }
    return -1.0; // t predates the oldest known boundary -- shouldn't happen
}

void report_pulse_angles(uint channel_count, uint64_t capture_start_us, double sample_period_us,
                          const CycleBoundary *boundaries, uint n_boundaries) {
    for (uint ch = 0; ch < channel_count; ch++) {
        uint32_t rise_idx, fall_idx;
        bool have_rise = find_first_edge(ch, true, &rise_idx);
        bool have_fall = find_first_edge(ch, false, &fall_idx);

        if (!have_rise && !have_fall) {
            printf("  ch%u: not detected\n", ch);
            continue;
        }

        printf("  ch%u:", ch);
        if (have_rise) {
            uint64_t t = capture_start_us + (uint64_t)(rise_idx * sample_period_us);
            double a = convert_time_to_angle(t, boundaries, n_boundaries);
            if (a < 0.0) printf(" rise=--"); else printf(" rise=%.2fdeg", a);
        } else {
            printf(" rise=--");
        }
        if (have_fall) {
            uint64_t t = capture_start_us + (uint64_t)(fall_idx * sample_period_us);
            double a = convert_time_to_angle(t, boundaries, n_boundaries);
            if (a < 0.0) printf(" fall=--"); else printf(" fall=%.2fdeg", a);
        } else {
            printf(" fall=--");
        }
        printf("\n");
    }
}
