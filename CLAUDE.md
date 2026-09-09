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

`firmware/src/main.c` is only the boot menu: it reads two keypresses over
USB serial — first a crank/cam profile, then a mode — and dispatches into
one of four mode modules. The chosen profile/mode then run forever (reset
board to pick different ones). Source layout:

- `profiles.c/.h` — `crankcam_profiles[]`, the selectable trigger-wheel
  table (ardu-stim-style: name + teeth/missing-teeth + cam angles). Adding
  a wheel means adding one entry here, nothing else.
- `event_table.c/.h` — event-table building against whichever profile
  `select_profile()` last set (`build_crank_events`, `build_cam_events`,
  `fill_buffer_slot`), plus `crank_events[]`/`cam_events[]`/
  `position_cycles[]` buffers sized for the largest profile
  (`MAX_TEETH_PER_REV`). Shared by all three modes — this is where the
  RPM-profile-to-cycle-count math lives.
- `gen_fire.c/.h` — `fire_gen_one_shot`, the reset-and-fire sequence for a
  single crank/cam burst (modes 2 and 3), and `configure_ping_pong_channel`,
  the TX ping-pong DMA config shared by the two continuous modes (1 and 4).
- `mode_continuous.c/.h` — mode 1.
- `mode_singleshot.c/.h` — mode 2.
- `capture_analysis.c/.h` — `capture_buf[]` + `capture_samples_used` (how
  much of it the current capture actually filled), edge finding, angle
  conversion (`compute_crank_reference`/`convert_to_angle`), pass/fail
  evaluation (`evaluate_spec`/`EcuOutputSpec`, mode 3 only), and
  `report_pulse_angles` (single-pulse-per-channel live report, mode 4
  only).
- `mode_capture.c/.h` — mode 3 driver (PIO/DMA setup + main loop); calls
  into `capture_analysis.h`.
- `mode_live.c/.h` — mode 4 driver.

Mode summaries:

1. **Mode 1 — continuous double-buffered generation.** Normal operation.
   Two PIO state machines (crank on GPIO2, cam on GPIO3) are fed by
   ping-pong DMA channel pairs from `crank_events[]`/`cam_events[]`. A DMA
   IRQ handler (`dma_irq_handler` in `mode_continuous.c`) refills the
   buffer that just finished playing while its twin plays, alternating an
   RPM scale each 720° cycle. Crank and cam SMs are started
   `pio_enable_sm_mask_in_sync` so they stay phase-locked.
   Two real hardware bugs here, both fixed and scope-verified (see git
   history): (a) a finished DMA channel's READ_ADDR/TRANS_COUNT sit at
   "end of buffer, 0 remaining" -- `chain_to`'s hardware retrigger reuses
   those verbatim, it does not restore them, so without an explicit rearm
   every chain-triggered buffer after the first pair transferred zero
   words and the pin froze. (b) cam's whole buffer (3 events, 6 words)
   fits inside the SM's 8-word FIFO, so its own DMA completion fired
   almost instantly -- long before the SM had actually played the buffer
   out -- making cam's *own* `chain_to` retrigger far too early and race
   the rearm. Fix: crank rearms itself on its own completion IRQ (still
   self-chained, since its buffer never fits the FIFO so completion stays
   correctly paced by real playback); cam no longer self-chains at all
   (`chain_to` pointing at itself, pico-sdk's documented way to disable
   it) and is instead driven explicitly, once per cycle, from that same
   crank completion IRQ.
2. **Mode 2 — single-shot 2-rev diagnostic.** Fixed RPM, fires one burst
   per Enter keypress via `fire_gen_one_shot`, for scope bring-up.
3. **Mode 3 — capture test.** Fires one single-shot generation burst on
   `pio0` while a second PIO program (`capture.pio`) samples 6 input pins
   (GPIO6-11) at a fixed rate on `pio1` into `capture_buf[]`. CPU then
   finds edges, derives crank angle from measured tooth timing
   (`compute_crank_reference`/`convert_to_angle`), and runs pass/fail
   checks (`evaluate_spec`) against `EcuOutputSpec` windows (expected
   rise/fall angle, tolerance, expected 720° half).
4. **Mode 4 — live capture.** Continuous and hardware-gapless, unlike
   mode 3's one-shot: a constant-RPM sub-menu (1000/3000/5000/7000) picks
   the simulated RPM, then crank/cam generation runs exactly like mode 1
   (ping-pong, same rearm fix, same content in both slots since there's
   no RPM ramp here) while a second ping-pong pair on `pio1` captures
   continuously alongside it. Capture's own buffer length
   (`live_want_samples`, `mode_live.c`) is the nearest-integer sample
   count for one real 720° cycle at the chosen RPM -- no deliberate
   margin, since capture free-runs continuously rather than being
   re-armed each cycle by another channel's IRQ (unlike cam, capture's
   buffer is always far bigger than the FIFO, so it doesn't have cam's
   "completes instantly" problem and can safely self-chain with just the
   same simple rearm). `dma_irq_handler` in `mode_live.c` handles two
   independent completion events in one combined ISR: crank's (rearms
   crank/cam, same as mode 1) and capture's (rearms that capture channel,
   flags its just-filled buffer via `ready_slot`). The **main loop**, not
   either ISR, does the actual analysis/printing (`compute_crank_reference`
   + `report_pulse_angles`) picked up from `ready_slot` -- keeping
   `printf` out of interrupt context, and meaning a slow print only skips
   reporting a cycle (counted and printed as "N cycle(s) skipped"), it
   never stalls generation or capture. A non-blocking
   `getchar_timeout_us(0)` poll each loop iteration lets Enter stop the
   loop.
   Known limitation: capture's chain is independent of generation's, so
   a cycle-to-cycle rounding remainder in `live_want_samples` can slowly
   drift its buffer boundary out of phase with generation's real cycle
   boundary. This occasionally (observed ~30% of cycles at 7000 RPM)
   makes channel 0 (crank, the reference channel itself) land in the
   wrong window and print a nonsense angle -- cosmetic only: every other
   channel (cam, and any real ECU channel) is computed relative to
   whichever window actually contains it and stays correct regardless,
   confirmed on hardware across 175/175 cycles.

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

Mode 3's two `EcuOutputSpec` demo checks (`mode_capture.c`) are hardcoded
to the 120°/300° cam window, which every current profile shares — they do
not vary per profile. A profile with different cam angles would need
those checks parametrized too.

## Key constants (firmware/src/event_table.h, firmware/src/capture_analysis.h)

Changing the trigger-wheel geometry, pin assignment, or capture parameters
means updating these together — they're cross-referenced throughout event
generation and angle conversion:

- `CRANK_PIN` (2), `CAM_PIN` (3) — output pins, fixed regardless of profile.
- `REVS_PER_CYCLE` (2) — crank revs per 720° cycle, fixed regardless of
  profile (cam disambiguates the two).
- `MAX_TEETH_PER_REV` (60) — upper bound across all `profiles.c` entries;
  sizes `crank_events[]`/`position_cycles[]`. `select_profile()` asserts
  a profile's `teeth_per_rev` against it — raise it if you add a bigger
  wheel.
- `crank_words_total` (runtime, set by `build_crank_events`) — actual
  per-buffer DMA word count for the selected profile; every DMA-configure
  call site reads this instead of a compile-time size, since profiles have
  different tooth/missing-teeth counts.
- `CAM_RISE_POSITION`/`CAM_FALL_POSITION` no longer exist as macros — each
  profile's `cam_rise_deg`/`cam_fall_deg` is converted to a tooth-position
  index at `select_profile()` time.
- `CAPTURE_BASE_PIN` (6), `CAPTURE_PIN_COUNT` (6), `CAPTURE_SAMPLE_HZ`,
  `CAPTURE_SAMPLES` — capture window; `CAPTURE_SAMPLES` is a plain integer
  literal kept in sync with `CAPTURE_SAMPLE_HZ * CAPTURE_DURATION_MS/1000`
  by hand (a computed expression here previously made `capture_buf` a
  variably-modified file-scope array — undefined behavior, silent boot
  crash). `CAPTURE_SAMPLES` is also the hard cap on any single capture
  (mode 4 clamps its per-RPM sample count to it).
- `capture_buf` (runtime, `uint32_t *`) — points at whichever physical
  buffer holds the capture to analyze right now. Defaults to an internal
  CAPTURE_SAMPLES-capacity buffer mode 3 uses as-is; mode 4 owns its own
  two physical buffers (`live_capture_buf[]`, sized for continuous
  ping-pong capture -- see mode 4 above) and repoints `capture_buf` at
  whichever one just finished before calling any `capture_analysis.c`
  function, no copy.
- `capture_samples_used` (runtime) — how many of `capture_buf`'s entries
  the current capture actually holds; every edge-scanning function in
  `capture_analysis.c` loops over this, not `CAPTURE_SAMPLES`, so a call
  site that DMAs fewer samples (modes 3 and 4 both) must set it before
  calling any of them, or edge-finding will read a previous, longer
  capture's stale tail.

Mode 3's default hardware setup is loopback: wire GPIO2→GPIO6 (crank) and
GPIO3→GPIO7 (cam) so `evaluate_spec` has known-good data to check against.
