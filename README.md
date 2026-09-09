# RP2040 Crank/Cam Testbench

RP2040 firmware that generates crank/cam trigger signals and checks ECU
outputs against expected crank-angle windows. PIO + DMA do the
timing-critical generation and capture; the CPU builds event tables and
does angle conversion / pass-fail analysis. Design background:
[`rp2040_pio_dma_crank_cam_concept.md`](rp2040_pio_dma_crank_cam_concept.md).

## Signal model

- Crank: missing-tooth wheel, one selectable profile per run (see below).
  2 revs per 720° four-stroke cycle.
- Cam: one pulse per 720° cycle, rising at 120°, falling at 300° (same for
  every profile).
- Crank pin: GPIO2. Cam pin: GPIO3.

## Trigger-wheel profiles

Ardu-stim-style: a small table of named crank/cam wheel definitions
(`firmware/src/profiles.c`), picked from the boot menu before the mode
menu. Teeth/missing-tooth counts are copied from real decoder patterns in
[ardu-stim's `wheel_defs.h`](https://github.com/speeduino/Ardu-Stim/blob/master/ardustim/ardustim/wheel_defs.h)
(GPLv3) — only the entries that fit this tool's model (one crank wheel
with a trailing missing-tooth gap, plus one cam sync pulse):

| Profile | Teeth | Missing | ardu-stim source |
|---|---:|---:|---|
| 60-2 (Bosch/GM) | 60 | 2 | `SIXTY_MINUS_TWO_WITH_CAM` |
| 36-1 (Ford/Mazda EDIS) | 36 | 1 | `THIRTY_SIX_MINUS_ONE` |
| 24-1 | 24 | 1 | `TWENTY_FOUR_MINUS_ONE` |
| 12-1 | 12 | 1 | `TWELVE_MINUS_ONE_WITH_CAM` |
| 6-1 (V-twin) | 6 | 1 | `SIX_MINUS_ONE_WITH_CAM` |
| 4-1 | 4 | 1 | `FOUR_MINUS_ONE_WITH_CAM` |
| 24 even tooth (no gap) | 24 | 0 | `TWENTY_FOUR_WITH_CAM` |

The cam pulse itself (rise/fall angle) is this tool's own single sync
window, not a copy of ardu-stim's cam signal — ardu-stim's is often
narrower or multi-pulse, which this tool's rev0/rev1 disambiguation
doesn't need (see `capture_analysis.c`).

Add a profile by appending a `{name, teeth_per_rev, missing_teeth,
cam_rise_deg, cam_fall_deg}` entry to `crankcam_profiles[]` in
`firmware/src/profiles.c` — no other file needs to change. A wheel outside
the missing-tooth family (e.g. Nissan 360, Subaru 7+1, Miata 99-05's
uneven-width teeth) would need a more general per-tooth-angle pattern
description, not implemented here.

## Firmware modes

One firmware image, boot-time menu over USB serial (no reflash to switch
profile or mode; reset/replug to pick different ones):

1. **Continuous generation** — double-buffered crank/cam output following
   the 720° RPM profile from the concept doc (1000/1500/2500/3500/4000 RPM
   at 0/180/360/540/720°). Alternates a 1.0x/1.15x RPM scale each cycle to
   exercise the double-buffer refill path. Runs forever.
2. **Single-shot diagnostic** — one fixed-RPM (1000) 2-rev crank/cam burst
   per Enter keypress, for scope bring-up.
3. **Capture test** — fires one single-shot burst, captures 6 input
   channels (GPIO6–11, wire GPIO2→6 and GPIO3→7 for loopback) at 100 kHz
   for 150 ms, derives crank angle from measured tooth timing, and
   evaluates two example ECU-output specs (expected rise/fall angle,
   tolerance, expected 720° half) against the captured edges, printing
   PASS/FAIL per check.
4. **Live capture** — continuous, hardware-gapless generation *and*
   capture at a chosen constant RPM (1000/3000/5000/7000): crank/cam
   ping-pong like mode 1, capture ping-pongs the same way alongside it, so
   the simulated engine never pauses between cycles. After every
   completed cycle, prints each of the 6 channels' rise/fall angle (or
   "not detected") — no pass/fail tolerance, just what was captured.
   Analysis/printing happens in the main loop, not the DMA IRQs, so a slow
   print never stalls generation or capture — it just skips reporting that
   cycle (reported as "N cycle(s) skipped"). Press Enter to start, Enter
   again to stop.
   Known cosmetic limitation: capture free-runs on its own DMA chain,
   independent of generation's, so its buffer boundary isn't forced to
   realign with generation's cycle boundary every cycle. Occasional
   sub-sample phase drift between the two can make channel 0 (the crank
   loopback channel itself) briefly land in the wrong reference window and
   print a nonsense angle. This does not affect any other channel — cam
   and any real ECU channel are computed relative to whichever window
   actually contains them and stay correct regardless (verified on
   hardware: cam held the correct ~120°/300° angle across 175/175 cycles
   at 7000 RPM, including cycles where ch0 showed the artifact).

## Layout

```
firmware/
  src/main.c              boot menu: profile select, then mode select
  src/profiles.c/.h       selectable crank/cam trigger-wheel profiles
  src/event_table.c/.h    crank/cam geometry + event-table build (all modes)
  src/gen_fire.c/.h       one-shot generator fire (modes 2, 3)
  src/mode_continuous.c/.h  mode 1
  src/mode_singleshot.c/.h  mode 2
  src/capture_analysis.c/.h edge/angle/pass-fail/live-report logic (modes 3, 4)
  src/mode_capture.c/.h   mode 3 driver
  src/mode_live.c/.h      mode 4 driver
  src/event_gen.pio       crank/cam waveform generator (PIO), used by all modes
  src/capture.pio         6-channel edge capture (PIO), used by mode 3
  CMakeLists.txt
  pico_sdk_import.cmake
  env.sh                  sets PICO_SDK_PATH (edit for your checkout)
```

`src/crank_gen.pio` and `src/cam_gen.pio` are earlier/unused PIO programs,
not wired into the CMake build.

## Build

```sh
source firmware/env.sh   # set PICO_SDK_PATH first
cd firmware
mkdir -p build && cd build
cmake ..
make -j
```

Produces `crankcam_testbench.uf2` — flash by holding BOOTSEL and copying
it to the RP2040's mass-storage drive.
