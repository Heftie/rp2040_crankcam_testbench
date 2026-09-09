#include "profiles.h"

// Teeth/missing-teeth counts are copied from real-world decoder patterns
// in ardu-stim's wheel_defs.h (github.com/speeduino/Ardu-Stim, GPLv3),
// picking the entries that fit this tool's model: one crank wheel with a
// trailing missing-tooth gap, plus one cam sync pulse. Only the geometry
// (teeth_per_rev/missing_teeth) is taken from ardu-stim -- our cam pulse
// is this tool's own single rise/fall sync window (120-300deg, wide and
// centered like the concept doc's example), not a copy of ardu-stim's
// often narrower/multi-blip cam signal, which this tool's rev0/rev1
// disambiguation (see capture_analysis.c) doesn't need. Comments name
// the ardu-stim WheelType enum entry each profile's numbers came from.
//
// cam_rise_deg/cam_fall_deg must land on an exact tooth-position boundary
// (360/teeth_per_rev must divide them evenly) -- true for all entries
// below except the 4-1 wheel, which uses a narrower window (90/270) sized
// to its coarser 90deg tooth pitch instead.
const CrankCamProfile crankcam_profiles[] = {
    // SIXTY_MINUS_TWO_WITH_CAM: Bosch/GM, most common automotive pattern.
    {"60-2 (Bosch/GM)", 60, 2, 120.0, 300.0},
    // THIRTY_SIX_MINUS_ONE: Ford/Mazda EDIS and common aftermarket.
    {"36-1 (Ford/Mazda EDIS)", 36, 1, 120.0, 300.0},
    // TWENTY_FOUR_MINUS_ONE.
    {"24-1", 24, 1, 120.0, 300.0},
    // TWELVE_MINUS_ONE_WITH_CAM.
    {"12-1", 12, 1, 120.0, 300.0},
    // SIX_MINUS_ONE_WITH_CAM: e.g. odd-fire V-twin motorcycle ECUs.
    {"6-1 (V-twin)", 6, 1, 120.0, 300.0},
    // FOUR_MINUS_ONE_WITH_CAM: small-engine/motorcycle pattern.
    {"4-1", 4, 1, 90.0, 270.0},
    // TWENTY_FOUR_WITH_CAM: 24 evenly spaced teeth, no missing tooth.
    {"24 even tooth (no gap)", 24, 0, 120.0, 300.0},
};

const uint crankcam_profile_count = sizeof(crankcam_profiles) / sizeof(crankcam_profiles[0]);
