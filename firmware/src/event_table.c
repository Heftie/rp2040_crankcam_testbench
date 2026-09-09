#include <assert.h>

#include "event_table.h"

uint32_t crank_events[NUM_BUFFERS][MAX_CRANK_WORDS_TOTAL];
uint32_t cam_events[NUM_BUFFERS][CAM_WORDS_TOTAL];
uint32_t position_cycles[MAX_POSITIONS_TOTAL];
double f_pio_hz;

uint positions_total;
uint32_t crank_words_total;

static CrankCamProfile current_profile;
static uint positions_per_rev;
static double deg_per_position;
static uint cam_rise_position;
static uint cam_fall_position;

void select_profile(const CrankCamProfile *profile) {
    assert(profile->teeth_per_rev <= MAX_TEETH_PER_REV);
    assert(profile->missing_teeth < profile->teeth_per_rev);

    current_profile = *profile;
    positions_per_rev = profile->teeth_per_rev;
    positions_total = positions_per_rev * REVS_PER_CYCLE;
    deg_per_position = 360.0 / positions_per_rev;

    // Cam edge angles must land on a tooth-position boundary (see
    // profiles.h) -- round rather than assert so a near-miss profile
    // still runs, just with the cam edge nudged to the nearest position.
    cam_rise_position = (uint)(profile->cam_rise_deg / deg_per_position + 0.5);
    cam_fall_position = (uint)(profile->cam_fall_deg / deg_per_position + 0.5);
}

static void push_event(uint32_t *buf, uint *idx, uint32_t state, uint32_t desired_cycles) {
    // event_gen.pio: hold = 2 (`out`x2) + (D+1) (`jmp x--` loop) cycles.
    assert(desired_cycles >= EVENT_MIN_CYCLES);
    buf[(*idx)++] = state;
    buf[(*idx)++] = desired_cycles - EVENT_MIN_CYCLES;
}

void build_crank_events(uint32_t *buf) {
    uint real_teeth_per_rev = positions_per_rev - current_profile.missing_teeth;
    uint idx = 0;
    for (uint rev = 0; rev < REVS_PER_CYCLE; rev++) {
        uint32_t gap_cycles = 0;
        for (uint pos = 0; pos < positions_per_rev; pos++) {
            uint32_t c = position_cycles[rev * positions_per_rev + pos];
            if (pos < real_teeth_per_rev) {
                uint32_t half = c / 2;
                push_event(buf, &idx, 1, half);
                push_event(buf, &idx, 0, c - half);
            } else {
                gap_cycles += c; // missing-tooth position, no edge
            }
        }
        push_event(buf, &idx, 0, gap_cycles);
    }
    crank_words_total = idx;
}

static uint32_t sum_position_cycles(uint from, uint to_exclusive) {
    uint32_t sum = 0;
    for (uint pos = from; pos < to_exclusive; pos++) {
        sum += position_cycles[pos];
    }
    return sum;
}

void build_cam_events(uint32_t *buf) {
    uint idx = 0;
    push_event(buf, &idx, 0, sum_position_cycles(0, cam_rise_position));
    push_event(buf, &idx, 1, sum_position_cycles(cam_rise_position, cam_fall_position));
    push_event(buf, &idx, 0, sum_position_cycles(cam_fall_position, positions_total));
}

void build_position_cycles_constant(double rpm) {
    uint32_t c = (uint32_t)(60.0 * f_pio_hz / (rpm * positions_per_rev) + 0.5);
    for (uint pos = 0; pos < positions_total; pos++) {
        position_cycles[pos] = c;
    }
}

// =============== RPM-profile-to-cycle-count conversion (mode 1) ===============

// Section 3 profile: desired RPM at 0/180/360/540/720 deg crank angle.
typedef struct {
    double angle_deg;
    double rpm;
} RpmPoint;

static const RpmPoint base_rpm_profile[] = {
    {0.0, 1000.0},
    {180.0, 1500.0},
    {360.0, 2500.0},
    {540.0, 3500.0},
    {720.0, 4000.0},
};
#define RPM_PROFILE_POINTS (sizeof(base_rpm_profile) / sizeof(base_rpm_profile[0]))

static double interpolate_rpm(double angle_deg, double scale) {
    if (angle_deg <= base_rpm_profile[0].angle_deg) {
        return base_rpm_profile[0].rpm * scale;
    }
    for (uint i = 0; i + 1 < RPM_PROFILE_POINTS; i++) {
        const RpmPoint *a = &base_rpm_profile[i];
        const RpmPoint *b = &base_rpm_profile[i + 1];
        if (angle_deg <= b->angle_deg) {
            double frac = (angle_deg - a->angle_deg) / (b->angle_deg - a->angle_deg);
            return (a->rpm + frac * (b->rpm - a->rpm)) * scale;
        }
    }
    return base_rpm_profile[RPM_PROFILE_POINTS - 1].rpm * scale;
}

// Delta t_i = 60 / (RPM_i * N); C_i = Delta t_i * f_PIO (doc section 3).
static uint32_t cycles_for_position(double angle_deg, double scale) {
    double rpm = interpolate_rpm(angle_deg, scale);
    double dt = 60.0 / (rpm * positions_per_rev);
    double cycles = dt * f_pio_hz;
    return (uint32_t)(cycles + 0.5);
}

static void build_position_cycles_profile(double scale) {
    for (uint pos = 0; pos < positions_total; pos++) {
        double angle = pos * deg_per_position;
        position_cycles[pos] = cycles_for_position(angle, scale);
    }
}

void fill_buffer_slot(uint slot, double scale) {
    build_position_cycles_profile(scale);
    build_crank_events(crank_events[slot]);
    build_cam_events(cam_events[slot]);
}
