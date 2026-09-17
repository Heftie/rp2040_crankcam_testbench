// Selectable crank/cam trigger-wheel profiles, ardu-stim-style: a small
// table of named wheel definitions picked at boot, instead of one
// compile-time geometry. Each profile is a missing-tooth crank wheel
// (N teeth, M missing) plus a cam pulse *list* (rise/fall angle pairs
// within the 720deg cycle). This covers both the common missing-tooth
// decoder family (60-2, 36-1, 24-1, 12-1, ...) with a single wide cam
// sync window, and multi-pulse cam wheels with uneven tooth spacing
// (e.g. iFlexAir-style 7 or 11 tooth cam wheels) via a longer pulse list.
#ifndef PROFILES_H
#define PROFILES_H

#include "pico/types.h"

typedef struct {
    double rise_deg; // pulse rise angle, 0-720deg cycle
    double fall_deg; // pulse fall angle, 0-720deg cycle
} CamPulse;

typedef struct {
    const char *name;
    uint teeth_per_rev; // wheel positions per crank revolution, missing included
    uint missing_teeth; // trailing positions per rev with no tooth (0 = none)
    const CamPulse *cam_pulses; // ascending rise_deg, non-overlapping, within 0-720
    uint cam_pulse_count; // 1 for a classic single sync window, >1 for a multi-tooth cam wheel
} CrankCamProfile;

extern const CrankCamProfile crankcam_profiles[];
extern const uint crankcam_profile_count;

#endif
