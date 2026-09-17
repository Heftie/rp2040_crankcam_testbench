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
to the RP2040's mass-storage drive. There is no firmware test suite —
validation is against real hardware (scope/logic analyzer) via the
firmware's own live capture/report output (see below), plus
`python/crankcam/step_test.py` (an RPM-step testbench driven over the
live serial protocol; see its module docstring) for repeatable hardware
runs. A Python client (`python/crankcam/`) drives the same command
protocol; see its module docstring. That client's own protocol-parsing
logic (not the firmware) has an offline, hardware-free unittest suite at
`python/tests/test_board.py` — run with `cd python && python -m
unittest discover -s tests`. That same `python/tests/` directory also
has `test_hardware.py`, a unittest suite that drives a REAL board over
this same API to check protocol-level behavior (profile/RPM bounds,
gen/capture lifecycle rules, cycle-report structure) -- it auto-skips
cleanly if no board answers on `CRANKCAM_PORT` (default
`/dev/ttyACM0`), so `python -m unittest discover -s tests` is always
safe to run with or without hardware attached; see its module docstring
for why it deliberately doesn't assert on the two known-open bugs below.

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
the actual analysis/printing (`report_pulse_angles`) picked up from
`ready_slot` -- keeping `printf` out of interrupt context, so a slow
print only skips reporting a cycle (counted and printed as "N cycle(s)
skipped"), never stalls generation or capture.

Angle reference is the generation engine's own timebase, not a decoded
capture-side edge: `engine.c` is the thing driving crank/cam out in the
first place, so it already knows exactly when each 720° cycle starts and
how fast it's playing. `dma_irq_handler`'s crank branch latches a
`CycleBoundary` (`capture_analysis.h`: a `time_us_64()` instant + the
constant RPM that cycle plays at) every time a cycle boundary fires, into
a small ring buffer (`cycle_history[CYCLE_HISTORY]` in `engine.c`) --
`engine_start_gen()` latches the first one directly, since slot 0's first
cycle starts via `dma_channel_start`/`pio_enable_sm_mask_in_sync`, not a
chain_to completion. Capture buffers get the same treatment
(`cap_slot_start_us[]`), latched when the *other* slot's DMA starts via
chain_to (or directly, for slot 0, in `engine_start_capture()`).
`engine_poll_capture()` snapshots both (briefly masking `DMA_IRQ_0`, the
same pattern used elsewhere in this file, so a boundary push mid-copy
can't be read half-written) and hands them to `capture_analysis.c`'s
`report_pulse_angles`, which converts each edge's capture-sample index to
an absolute timestamp and then to an angle via `convert_time_to_angle` --
a linear elapsed-time/cycle-duration calculation, exact because RPM is
constant within one cycle by construction (`fill_buffer_slot_constant`).
No capture channel is special and nothing needs to be wired back to the
crank/cam outputs for this to work; keeping a few cycle boundaries, not
just the latest, covers a capture buffer whose independent DMA chain
happens to straddle a boundary.

Another real hardware bug, found and fixed while bringing up the timebase
change above: latching a cycle boundary's timestamp at the instant crank's
DMA channel reports "complete" is too early, and by a constant amount, not
jitter. `event_gen_program_init()` joins each generation SM's TX FIFO
(`PIO_FIFO_JOIN_TX`), doubling it to 8 words; the DMA channel's dreq only
fires on FIFO space, so in steady playback the FIFO sits essentially
always full, and "DMA complete" only means the *last word was accepted
into the FIFO*, not that the SM has actually shifted it out to the pin
yet. Reproduced on hardware: cam's known-true 120°/300° pulse (this
firmware generates cam itself, so its real angles are known independent
of any capture) read back a constant +24.2° at 3000 RPM on the 60-2
profile, stable cycle to cycle, present from the second reported cycle
onward (the very first cycle is latched directly at
`pio_enable_sm_mask_in_sync`, not via a DMA completion, so it alone was
unaffected). Root-caused by walking the buffer's actual event structure:
DMA finishes pushing the last event once the SM has *started* consuming
the event 4 positions earlier (FIFO capacity in events), so at completion
time 5 events' worth of playback -- the 4 not-yet-touched plus the one the
SM just began -- are still queued ahead of the true cycle boundary, not
4. `crank_tail_backlog_us()` in `engine.c` sums those last 5 events' real
durations (`CRANK_FIFO_BACKLOG_WORDS` = 10 words) from `crank_events[slot]`
-- read before `fill_buffer_slot_constant()` overwrites it -- and adds
that to the latched timestamp. Verified on hardware after the fix: cam
read within ~0.5° of 120°/300° across 1000-7000 RPM on the 60-2 profile,
consistent with `CAPTURE_SAMPLE_HZ` quantization noise (10us/sample), not
a residual systematic offset.

Two more real hardware bugs, both in `engine_start_capture`/
`engine_start_gen`, fixed and scope-verified (see git history): (a) the
RP2040's per-channel `dma_hw->ints0` completion flag is sticky --
`dma_channel_set_irq0_enabled()` only gates whether that channel's
completion *interrupts* the CPU, it does not clear the flag itself.
Every capture session's channels complete (ping-ponging) many times
before being stopped, so the flag is essentially always left set; without
clearing it, the very next crank-triggered `dma_irq_handler()` call after
a restart -- often before the channel had even started for real that
session -- misread it as a genuine completion, corrupting
`ready_slot`/`produced_cycles` and the DMA's own live write address right
as the session began. Reproduced on hardware: repeated `c1`/`c0` restarts
within one still-running `g1` session produced garbage-looking angle
reports almost every time; fixed by explicitly clearing both capture
channels' `ints0` bits before re-enabling their IRQs. (b)
`dma_irq_handler`'s capture branch is unconditional (doesn't check
`capture_running`), so it runs on every crank-completion IRQ regardless
-- a routine, unrelated IRQ landing mid-setup could read/rearm a capture
channel's registers concurrently with `configure_capture_ping_pong()`, a
genuine foreground/ISR register race. Fixed by masking `DMA_IRQ_0` for
the duration of `engine_start_capture`'s (and, defensively,
`engine_start_gen`'s) setup.

Historical hardware finding, from before the timebase change above, kept
for anyone chasing crank/cam signal integrity on the generation side
itself: a fresh `g1` immediately followed by loopback-wired capture, in
roughly half of sessions, found the crank *input* pin (GPIO2 looped back,
polled directly with `gpio_get()`, bypassing PIO/DMA entirely) stuck at a
constant level for the whole session, while the crank *output* pad
itself, polled the same way right at the source, always toggled
correctly, and capture's DMA channel always armed and progressed
normally. Swapping the loopback wiring (crank moved from its default
GPIO2->GPIO6 to GPIO2->GPIO7, cam from GPIO3->GPIO7 to GPIO3->GPIO6)
moved the same symptom onto GPIO7 with crank, while cam stayed rock solid
on GPIO6 -- ruling out a GPIO6-specific pad/pull/wire fault and pointing
at something in crank's own generation path (`engine_start_gen`'s crank
SM setup, or `event_gen.pio`'s output config, e.g. `pio_gpio_init`/
`pio_sm_set_consecutive_pindirs` ordering vs. `pio_sm_init`/
`pio_enable_sm_mask_in_sync`) that occasionally leaves the driven signal
unable to properly reach a downstream receiver despite reading correctly
at its own source pad. This no longer affects capture's angle reference
(see above -- it doesn't depend on any loopback signal anymore, decoded
or otherwise), but the same drive issue, if still present, would still
affect a real ECU wired to GPIO2/GPIO3. Not re-investigated since
removing capture's dependency on it; needs a scope on the output pin
itself (not a same-chip `gpio_get()`) during a failing session to confirm
whether it's still there.

The old edge-decoding scheme's window-drift limitation is gone along with
the scheme itself: since angle is now computed directly from elapsed time
against `cycle_history`, not a window found by decoding captured crank
edges, there's no more discrete window for a timestamp to miss.
`convert_time_to_angle` only returns the "--"/no-angle sentinel if a
sample's timestamp predates every known `CycleBoundary`, which shouldn't
happen once generation has completed even its first cycle boundary latch
(both `engine_start_gen()` and `engine_start_capture()` latch one
directly, before any capture data can exist). `python/crankcam/board.py`'s
`ChannelReport.detected` still distinguishes "no edge at all" (a channel
that's genuinely unwired) from "an edge was found but its angle came back
as the sentinel" -- now expected to be vanishingly rare rather than a
routine, session-dependent artifact.

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

Loopback wiring (GPIO2→GPIO6, GPIO3→GPIO7) is optional, useful as a
self-test (two channels with known-good signal to check against) -- it's
not required for anything. Capture's angle reference no longer depends on
any particular channel being wired to crank/cam; all 6 capture channels
are plain, equivalent ECU-output channels.
