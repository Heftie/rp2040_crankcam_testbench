# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

RP2040 firmware for a crank/cam trigger simulator and ECU-output analyzer.
PIO + DMA generate deterministic crank/cam waveforms and capture ECU output
edges; the CPU only builds event tables ahead of time and does angle
conversion / pass-fail analysis after the fact — no per-tooth CPU timing.
Design rationale and math (RPM-profile → cycle-count conversion, event-table
format, angle calculation, pass/fail window checks) are in
`rp2040_pio_dma_crank_cam_concept.md` — read it before changing the timing
model in `firmware/src/event_table.c`.

## Build

```sh
source firmware/env.sh   # sets PICO_SDK_PATH -- edit this file for your pico-sdk checkout
cd firmware
mkdir -p build && cd build
cmake ..
make -j
```

Output is `crankcam_testbench.uf2`; flash by holding BOOTSEL and copying it
to the RP2040's mass-storage drive. There is no test suite — validation is
against real hardware (scope/logic analyzer) via the firmware's own
diagnostic modes (see below).

## Architecture

`firmware/src/main.c` is only the boot menu: it reads one keypress over USB
serial and dispatches into one of three mode modules. The chosen mode then
runs forever (reset board to pick a different mode). Source layout:

- `event_table.c/.h` — crank/cam geometry macros, `crank_events[]`/
  `cam_events[]`/`position_cycles[]` buffers, and event-table building
  (`build_crank_events`, `build_cam_events`, `fill_buffer_slot`). Shared by
  all three modes — this is where the RPM-profile-to-cycle-count math
  lives.
- `gen_fire.c/.h` — `fire_gen_one_shot`, the reset-and-fire sequence for a
  single crank/cam burst. Shared by modes 2 and 3.
- `mode_continuous.c/.h` — mode 1.
- `mode_singleshot.c/.h` — mode 2.
- `capture_analysis.c/.h` — `capture_buf[]`, edge finding, angle
  conversion (`compute_crank_reference`/`convert_to_angle`), and pass/fail
  evaluation (`evaluate_spec`/`EcuOutputSpec`). Used only by mode 3.
- `mode_capture.c/.h` — mode 3 driver (PIO/DMA setup + main loop); calls
  into `capture_analysis.h`.

Mode summaries:

1. **Mode 1 — continuous double-buffered generation.** Normal operation.
   Two PIO state machines (crank on GPIO2, cam on GPIO3) are fed by
   ping-pong DMA channel pairs from `crank_events[]`/`cam_events[]`. A DMA
   IRQ handler (`dma_irq_handler` in `mode_continuous.c`) refills the
   buffer that just finished playing while its twin plays, alternating an
   RPM scale each 720° cycle. Crank and cam SMs are started
   `pio_enable_sm_mask_in_sync` so they stay phase-locked.
2. **Mode 2 — single-shot 2-rev diagnostic.** Fixed RPM, fires one burst
   per Enter keypress via `fire_gen_one_shot`, for scope bring-up.
3. **Mode 3 — capture test.** Fires one single-shot generation burst on
   `pio0` while a second PIO program (`capture.pio`) samples 6 input pins
   (GPIO6-11) at a fixed rate on `pio1` into `capture_buf[]`. CPU then
   finds edges, derives crank angle from measured tooth timing
   (`compute_crank_reference`/`convert_to_angle`), and runs pass/fail
   checks (`evaluate_spec`) against `EcuOutputSpec` windows (expected
   rise/fall angle, tolerance, expected 720° half).

Both generation and capture share one event-table convention: each event
is 2 FIFO words (pin state, delay-in-cycles), consumed by the `event_gen`
PIO program (`firmware/src/event_gen.pio`) via `out pins, 32` / `out x, 32`
/ `jmp x--` loop. The CPU always derives cycle counts from a single
`position_cycles[]` array (built either from the RPM profile or a constant
RPM) so crank and cam edges stay phase-locked by construction
(`fill_buffer_slot`).

`firmware/src/crank_gen.pio` and `firmware/src/cam_gen.pio` are earlier,
unused PIO programs — not wired into `CMakeLists.txt`. Only
`event_gen.pio` and `capture.pio` are part of the build.

## Key constants (firmware/src/event_table.h, firmware/src/capture_analysis.h)

Changing the trigger-wheel geometry, pin assignment, or capture parameters
means updating these together — they're cross-referenced throughout event
generation and angle conversion:

- `CRANK_PIN` (2), `CAM_PIN` (3) — output pins.
- `TEETH_PER_REV` (60), `MISSING_TEETH` (2), `REVS_PER_CYCLE` (2) — crank
  wheel geometry; feeds `CRANK_EVENTS_TOTAL`/`CRANK_WORDS_TOTAL` sizing.
- `CAM_RISE_POSITION`/`CAM_FALL_POSITION` — cam pulse angle, expressed as
  crank tooth-position indices (not degrees) so it reuses the same
  `position_cycles[]` timing as the crank.
- `CAPTURE_BASE_PIN` (6), `CAPTURE_PIN_COUNT` (6), `CAPTURE_SAMPLE_HZ`,
  `CAPTURE_SAMPLES` — capture window; `CAPTURE_SAMPLES` is a plain integer
  literal kept in sync with `CAPTURE_SAMPLE_HZ * CAPTURE_DURATION_MS/1000`
  by hand (a computed expression here previously made `capture_buf` a
  variably-modified file-scope array — undefined behavior, silent boot
  crash).

Mode 3's default hardware setup is loopback: wire GPIO2→GPIO6 (crank) and
GPIO3→GPIO7 (cam) so `evaluate_spec` has known-good data to check against.
