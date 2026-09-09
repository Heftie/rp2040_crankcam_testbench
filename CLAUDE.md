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
against real hardware (scope/logic analyzer) via the firmware's own live
capture/report output (see below). A Python client (`python/crankcam/`)
drives the same command protocol; see its module docstring.

## Architecture

`firmware/src/main.c` is a command loop over USB serial, not a menu: it
reads one line at a time (letter + optional argument, e.g. `r3000`,
`g1`), dispatches into `engine.c`, and prints one `OK .../ERR ...` line
per command. Generation and capture are independently started/stopped at
any time — no reset needed to change profile, RPM, or what's running.
Source layout:

- `profiles.c/.h` — `crankcam_profiles[]`, the selectable trigger-wheel
  table (ardu-stim-style: name + teeth/missing-teeth + cam angles). Adding
  a wheel means adding one entry here, nothing else.
- `event_table.c/.h` — event-table building against whichever profile
  `select_profile()` last set (`build_crank_events`, `build_cam_events`,
  `fill_buffer_slot_constant`), plus `crank_events[]`/`cam_events[]`/
  `position_cycles[]` buffers sized for the largest profile
  (`MAX_TEETH_PER_REV`). This is where the RPM-to-cycle-count math lives.
- `gen_fire.c/.h` — `configure_ping_pong_channel`, the TX ping-pong DMA
  config for continuous crank/cam generation.
- `capture_analysis.c/.h` — `capture_buf`/`capture_samples_used` (which
  physical capture buffer to analyze, and how much of it is valid), edge
  finding, angle conversion (`compute_crank_reference`/`convert_to_angle`,
  internal), and `report_pulse_angles` (per-cycle live report, one
  rise/fall angle per channel).
- `engine.c/.h` — the state machine: owns the PIO/DMA resources for both
  generation (`pio0`) and capture (`pio1`), claimed once at boot and
  reused across start/stop rather than reclaimed each time. Exposes
  `engine_select_profile`, `engine_set_rpm`, `engine_start_gen`/
  `engine_stop_gen`, `engine_start_capture`/`engine_stop_capture`, and
  `engine_poll_capture` (called every main-loop iteration; prints a
  `cycle N:` report when a capture buffer has finished).

Generation (`engine_start_gen`/`engine_stop_gen`): two PIO state machines
(crank on GPIO2, cam on GPIO3) fed by ping-pong DMA channel pairs from
`crank_events[]`/`cam_events[]`, exactly like the old continuous mode.
`dma_irq_handler` in `engine.c` refills the buffer that just finished
playing while its twin plays — but now rebuilds it from whatever
`target_rpm` is *at that instant* (`fill_buffer_slot_constant`), rather
than a fixed demo ramp, which is what makes the `r<n>` command take
effect live at the next 720° cycle boundary. Crank and cam SMs are
started with `pio_enable_sm_mask_in_sync` so they stay phase-locked.

Two real hardware bugs live in this refill path, both fixed and
scope-verified (see git history): (a) a finished DMA channel's
READ_ADDR/TRANS_COUNT sit at "end of buffer, 0 remaining" -- `chain_to`'s
hardware retrigger reuses those verbatim, it does not restore them, so
without an explicit rearm every chain-triggered buffer after the first
pair transferred zero words and the pin froze. (b) cam's whole buffer (3
events, 6 words) fits inside the SM's 8-word FIFO, so its own DMA
completion fired almost instantly -- long before the SM had actually
played the buffer out -- making cam's *own* `chain_to` retrigger far too
early and race the rearm. Fix: crank rearms itself on its own completion
IRQ (still self-chained, since its buffer never fits the FIFO so
completion stays correctly paced by real playback); cam no longer
self-chains at all (`chain_to` pointing at itself, pico-sdk's documented
way to disable it) and is instead driven explicitly, once per cycle, from
that same crank completion IRQ.

Capture (`engine_start_capture`/`engine_stop_capture`, requires
generation already running): continuous and hardware-gapless, a second
PIO program (`capture.pio`) samples 6 input pins (GPIO6-11) on `pio1`
into a ping-ponged pair of `live_capture_buf[]` buffers. Each buffer's
length is recomputed from `target_rpm` on every rearm too (same live-RPM
mechanism as generation), clamped to `LIVE_MAX_CYCLE_SAMPLES` — which
sizes the buffers for `CAPTURE_MIN_RPM`, the floor `r<n>` enforces before
`c1` will start capture. `dma_irq_handler` handles both completion events
(crank's and capture's) in one combined ISR; the **main loop**
(`engine_poll_capture`, called from `main()`'s loop, not either ISR) does
the actual analysis/printing (`compute_crank_reference` +
`report_pulse_angles`) picked up from `ready_slot` -- keeping `printf`
out of interrupt context, so a slow print only skips reporting a cycle
(counted and printed as "N cycle(s) skipped"), never stalls generation or
capture.

Known limitation, carried over unchanged from the design this replaced:
capture's chain is independent of generation's, so a cycle-to-cycle
rounding remainder in the sample count can slowly drift its buffer
boundary out of phase with generation's real cycle boundary. This
occasionally (observed ~30% of cycles at 7000 RPM) makes channel 0
(crank, the reference channel itself) land in the wrong window and print
a nonsense angle -- cosmetic only: every other channel (cam, and any real
ECU channel) is computed relative to whichever window actually contains
it and stays correct regardless, confirmed on hardware across 175/175
cycles.

Both generation and capture share one event-table convention: each event
is 2 FIFO words (pin state, delay-in-cycles), consumed by the `event_gen`
PIO program (`firmware/src/event_gen.pio`) via `out pins, 32` / `out x, 32`
/ `jmp x--` loop. The CPU always derives cycle counts from a single
`position_cycles[]` array (currently always constant-RPM; see
`build_position_cycles_constant`) so crank and cam edges stay phase-locked
by construction (`fill_buffer_slot_constant`). The angle-varying RPM
profile math in `rp2040_pio_dma_crank_cam_concept.md` section 3 is still
the reference for that conversion if a non-constant profile is added back
later — it's not currently wired into any command.

`firmware/src/crank_gen.pio` and `firmware/src/cam_gen.pio` are earlier,
unused PIO programs — not wired into `CMakeLists.txt`. Only
`event_gen.pio` and `capture.pio` are part of the build.

There is no pass/fail spec-checking in firmware anymore (the old
`EcuOutputSpec`/`evaluate_spec` one-shot check) — `report_pulse_angles`
reports rise/fall angle per channel every cycle; tolerance/pass-fail
comparison against expected angles is left to the client (e.g. the Python
API) watching that stream.

## Key constants (firmware/src/event_table.h, firmware/src/capture_analysis.h, firmware/src/engine.h)

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
- `ENGINE_MAX_RPM` (20000), `CAPTURE_MIN_RPM` (1000) — sanity ceiling for
  the `r<n>` command, and the floor `c1` enforces (below it, capture's
  fixed-size buffer can't hold a full cycle -- see `LIVE_MAX_CYCLE_SAMPLES`
  in `engine.c`). Generation alone has no floor.
- `CAPTURE_BASE_PIN` (6), `CAPTURE_PIN_COUNT` (6), `CAPTURE_SAMPLE_HZ` —
  capture pins/rate. Sample count is now computed live from RPM
  (`samples_for_rpm` in `engine.c`), not a fixed literal.
- `capture_buf` (runtime, `uint32_t *`) — points at whichever physical
  buffer holds the capture to analyze right now. `engine.c` owns the two
  physical buffers (`live_capture_buf[]`) and repoints `capture_buf` at
  whichever one just finished before calling any `capture_analysis.c`
  function, no copy.
- `capture_samples_used` (runtime) — how many of `capture_buf`'s entries
  the current capture actually holds; every edge-scanning function in
  `capture_analysis.c` loops over this, not a fixed size, so a faster-RPM
  (shorter) capture doesn't pick up a previous, longer capture's stale
  tail. `engine.c` sets this before calling any of them.

Default hardware setup is loopback: wire GPIO2→GPIO6 (crank) and
GPIO3→GPIO7 (cam) so capture has known-good data to report on.
