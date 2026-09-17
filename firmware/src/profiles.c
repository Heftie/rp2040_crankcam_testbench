#include "profiles.h"

// Teeth/missing-teeth counts for the ardu-stim-derived entries are copied
// from real-world decoder patterns in ardu-stim's wheel_defs.h
// (github.com/speeduino/Ardu-Stim, GPLv3), picking the entries that fit
// this tool's model: one crank wheel with a trailing missing-tooth gap,
// plus a cam pulse list. Only the geometry (teeth_per_rev/missing_teeth)
// is taken from ardu-stim -- our cam pulse is this tool's own single
// rise/fall sync window (120-300deg, wide and centered like the concept
// doc's example), not a copy of ardu-stim's often narrower/multi-blip cam
// signal, which this tool's rev0/rev1 disambiguation (see
// capture_analysis.c) doesn't need. Comments name the ardu-stim WheelType
// enum entry each profile's numbers came from.
//
// cam pulse edges must land on an exact tooth-position boundary (see
// profiles.h) -- select_profile() rounds rather than asserts, so a
// near-miss profile still runs, just with the edge nudged to the nearest
// position. True for all entries below except the 4-1 wheel, which uses
// a narrower window (90/270) sized to its coarser 90deg tooth pitch.
static const CamPulse cam_default_sync[] = {{120.0, 300.0}};
static const CamPulse cam_4_1_sync[] = {{90.0, 270.0}};

// The three multi-pulse cam wheels below (iFlexAir 58/7 and 59/11
// trigger-wheel engines, see Cam_Crank_Signal_Analysis.md) are narrow
// pulse-train cam signals, not one wide sync window -- each entry in the
// source doc's "Pulse Positions"/"Cam Signal Pulse Positions" table gives
// only one edge per pulse (rising for the FAW wheels, falling for
// Perkins), not a pulse width, so a 10deg width (this tool's own choice,
// same spirit as the single-pulse profiles above) is used for every
// pulse -- narrow enough to clear the tightest gap between consecutive
// pulses in all three wheels (20deg, on the Perkins wheel). The source
// doc's angles are also relative to that engine's own TDC-referenced
// coordinate origin (and go negative); each wheel below is shifted by a
// constant so its pulses land inside this tool's own 0-720deg cycle
// convention while preserving the doc's relative pulse spacing exactly --
// the absolute phase against the crank sync gap is otherwise arbitrary,
// same as the single-pulse profiles' 120/300deg window above.

// FAW_DIESEL_58_7: iFlexAir 58/7, FAW diesel. Doc pulses (rising edges)
// -192,-162,-72,48,168,288,408deg, shifted by +192 so the first pulse
// starts the cycle.
static const CamPulse cam_faw_diesel_58_7[] = {
    {0.0, 10.0}, {30.0, 40.0}, {120.0, 130.0}, {240.0, 250.0},
    {360.0, 370.0}, {480.0, 490.0}, {600.0, 610.0},
};

// FAW_CNG_58_7: iFlexAir 58/7, FAW CNG -- same wheel geometry as the
// diesel variant, but an uneven/asymmetric cam pulse pattern. Doc pulses
// (rising edges) -42,51,81,158,278,398,518deg, shifted by +42.
static const CamPulse cam_faw_cng_58_7[] = {
    {0.0, 10.0}, {93.0, 103.0}, {123.0, 133.0}, {200.0, 210.0},
    {320.0, 330.0}, {440.0, 450.0}, {560.0, 570.0},
};

// PERKINS_59_11: iFlexAir 59/11, Perkins. Doc pulses (falling edges) PES
// 18, edges 38/98/158/218/278/338, PEA 378, edges 486/516/578/636deg,
// shifted by -8 so the first pulse's *rise* lands at 0deg (falling-edge
// referenced, unlike the two FAW wheels above, so the shift is chosen to
// keep the derived rise edge, not the doc's own falling edge, in range).
static const CamPulse cam_perkins_59_11[] = {
    {0.0, 10.0}, {20.0, 30.0}, {80.0, 90.0}, {140.0, 150.0},
    {200.0, 210.0}, {260.0, 270.0}, {320.0, 330.0}, {360.0, 370.0},
    {468.0, 478.0}, {498.0, 508.0}, {560.0, 570.0}, {618.0, 628.0},
};

#define CAM_PULSE_COUNT(arr) (sizeof(arr) / sizeof((arr)[0]))

const CrankCamProfile crankcam_profiles[] = {
    // SIXTY_MINUS_TWO_WITH_CAM: Bosch/GM, most common automotive pattern.
    {"60-2 (Bosch/GM)", 60, 2, cam_default_sync, CAM_PULSE_COUNT(cam_default_sync)},
    // THIRTY_SIX_MINUS_ONE: Ford/Mazda EDIS and common aftermarket.
    {"36-1 (Ford/Mazda EDIS)", 36, 1, cam_default_sync, CAM_PULSE_COUNT(cam_default_sync)},
    // TWENTY_FOUR_MINUS_ONE.
    {"24-1", 24, 1, cam_default_sync, CAM_PULSE_COUNT(cam_default_sync)},
    // TWELVE_MINUS_ONE_WITH_CAM.
    {"12-1", 12, 1, cam_default_sync, CAM_PULSE_COUNT(cam_default_sync)},
    // SIX_MINUS_ONE_WITH_CAM: e.g. odd-fire V-twin motorcycle ECUs.
    {"6-1 (V-twin)", 6, 1, cam_default_sync, CAM_PULSE_COUNT(cam_default_sync)},
    // FOUR_MINUS_ONE_WITH_CAM: small-engine/motorcycle pattern.
    {"4-1", 4, 1, cam_4_1_sync, CAM_PULSE_COUNT(cam_4_1_sync)},
    // TWENTY_FOUR_WITH_CAM: 24 evenly spaced teeth, no missing tooth.
    {"24 even tooth (no gap)", 24, 0, cam_default_sync, CAM_PULSE_COUNT(cam_default_sync)},
    // iFlexAir 58/7, FAW Diesel -- see Cam_Crank_Signal_Analysis.md.
    {"FAW Diesel iFlexAir 58/7", 58, 1, cam_faw_diesel_58_7, CAM_PULSE_COUNT(cam_faw_diesel_58_7)},
    // iFlexAir 58/7, FAW CNG (uneven cam pulse spacing).
    {"FAW CNG iFlexAir 58/7", 58, 1, cam_faw_cng_58_7, CAM_PULSE_COUNT(cam_faw_cng_58_7)},
    // iFlexAir 59/11, Perkins (falling-edge cylinder ID scheme).
    {"Perkins iFlexAir 59/11", 59, 1, cam_perkins_59_11, CAM_PULSE_COUNT(cam_perkins_59_11)},
};

const uint crankcam_profile_count = sizeof(crankcam_profiles) / sizeof(crankcam_profiles[0]);
