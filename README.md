# RP2040 Crank/Cam Testbench

RP2040 firmware that generates crank/cam trigger signals and checks ECU
outputs against expected crank-angle windows. PIO + DMA do the
timing-critical generation and capture; the CPU builds event tables and
does angle conversion / pass-fail analysis. Design background:
[`rp2040_pio_dma_crank_cam_concept.md`](rp2040_pio_dma_crank_cam_concept.md).

## Signal model

- Crank: 60-tooth wheel, 2 missing teeth (58 real teeth/rev), 2 revs per
  720° four-stroke cycle.
- Cam: one pulse per 720° cycle, rising at 120°, falling at 300°.
- Crank pin: GPIO2. Cam pin: GPIO3.

## Firmware modes

One firmware image, one boot-time menu over USB serial (no reflash to
switch mode; reset/replug to pick a different one):

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

## Layout

```
firmware/
  src/main.c              boot menu only, dispatches to a mode
  src/event_table.c/.h    crank/cam geometry + event-table build (all modes)
  src/gen_fire.c/.h       one-shot generator fire (modes 2, 3)
  src/mode_continuous.c/.h  mode 1
  src/mode_singleshot.c/.h  mode 2
  src/capture_analysis.c/.h edge/angle/pass-fail logic (mode 3)
  src/mode_capture.c/.h   mode 3 driver
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
