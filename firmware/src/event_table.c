#include <assert.h>

#include "event_table.h"

uint32_t crank_events[NUM_BUFFERS][CRANK_WORDS_TOTAL];
uint32_t cam_events[NUM_BUFFERS][CAM_WORDS_TOTAL];
uint32_t position_cycles[POSITIONS_TOTAL];
double f_pio_hz;

static void push_event(uint32_t *buf, uint *idx, uint32_t state, uint32_t desired_cycles) {
    // event_gen.pio: hold = 2 (`out`x2) + (D+1) (`jmp x--` loop) cycles.
    assert(desired_cycles >= EVENT_MIN_CYCLES);
    buf[(*idx)++] = state;
    buf[(*idx)++] = desired_cycles - EVENT_MIN_CYCLES;
}

void build_crank_events(uint32_t *buf) {
    uint idx = 0;
    for (uint rev = 0; rev < REVS_PER_CYCLE; rev++) {
        uint32_t gap_cycles = 0;
        for (uint pos = 0; pos < POSITIONS_PER_REV; pos++) {
            uint32_t c = position_cycles[rev * POSITIONS_PER_REV + pos];
            if (pos < REAL_TEETH_PER_REV) {
                uint32_t half = c / 2;
                push_event(buf, &idx, 1, half);
                push_event(buf, &idx, 0, c - half);
            } else {
                gap_cycles += c; // missing-tooth position, no edge
            }
        }
        push_event(buf, &idx, 0, gap_cycles);
    }
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
    push_event(buf, &idx, 0, sum_position_cycles(0, CAM_RISE_POSITION));
    push_event(buf, &idx, 1, sum_position_cycles(CAM_RISE_POSITION, CAM_FALL_POSITION));
    push_event(buf, &idx, 0, sum_position_cycles(CAM_FALL_POSITION, POSITIONS_TOTAL));
}

void build_position_cycles_constant(double rpm) {
    uint32_t c = (uint32_t)(60.0 * f_pio_hz / (rpm * TEETH_PER_REV) + 0.5);
    for (uint pos = 0; pos < POSITIONS_TOTAL; pos++) {
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
    double dt = 60.0 / (rpm * TEETH_PER_REV);
    double cycles = dt * f_pio_hz;
    return (uint32_t)(cycles + 0.5);
}

static void build_position_cycles_profile(double scale) {
    for (uint pos = 0; pos < POSITIONS_TOTAL; pos++) {
        double angle = pos * DEG_PER_POSITION;
        position_cycles[pos] = cycles_for_position(angle, scale);
    }
}

void fill_buffer_slot(uint slot, double scale) {
    build_position_cycles_profile(scale);
    build_crank_events(crank_events[slot]);
    build_cam_events(cam_events[slot]);
}
