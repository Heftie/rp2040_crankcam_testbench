#include "profiles.h"

// cam_rise_deg/cam_fall_deg are kept at 120/300 for every profile so the
// existing mode-3 pass/fail demo specs (see mode_capture.c) stay valid
// regardless of which profile is selected. Angles must land on an exact
// tooth-position boundary of the profile (360/teeth_per_rev divides them
// evenly) -- true for all entries below.
const CrankCamProfile crankcam_profiles[] = {
    {"60-2 (default)", 60, 2, 120.0, 300.0},
    {"36-1", 36, 1, 120.0, 300.0},
    {"24-1", 24, 1, 120.0, 300.0},
    {"12-1", 12, 1, 120.0, 300.0},
    {"60-0 (no missing tooth)", 60, 0, 120.0, 300.0},
};

const uint crankcam_profile_count = sizeof(crankcam_profiles) / sizeof(crankcam_profiles[0]);
