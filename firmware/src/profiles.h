// Selectable crank/cam trigger-wheel profiles, ardu-stim-style: a small
// table of named wheel definitions picked at boot, instead of one
// compile-time geometry. Each profile is a missing-tooth crank wheel
// (N teeth, M missing) plus one cam pulse (rise/fall angle within the
// 720deg cycle). This covers the common missing-tooth decoder family
// (60-2, 36-1, 24-1, 12-1, ...); a wheel with an irregular, non-missing-
// tooth-family pattern (e.g. Nissan 360, Subaru 7+1) would need a more
// general per-tooth-angle list, not implemented here.
#ifndef PROFILES_H
#define PROFILES_H

#include "pico/types.h"

typedef struct {
    const char *name;
    uint teeth_per_rev; // wheel positions per crank revolution, missing included
    uint missing_teeth; // trailing positions per rev with no tooth (0 = none)
    double cam_rise_deg; // cam pulse rise angle, 0-720deg cycle
    double cam_fall_deg; // cam pulse fall angle, 0-720deg cycle
} CrankCamProfile;

extern const CrankCamProfile crankcam_profiles[];
extern const uint crankcam_profile_count;

#endif
