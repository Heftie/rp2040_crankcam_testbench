# RP2040 Crank/Cam Testbench

RP2040 firmware that generates crank/cam trigger signals and reports
ECU-output edges as crank angle, command-driven over USB serial. PIO + DMA
do the timing-critical generation and capture; the CPU builds event
tables and does angle conversion. Design background:
[`rp2040_pio_dma_crank_cam_concept.md`](rp2040_pio_dma_crank_cam_concept.md).

## Signal model

- Crank: missing-tooth wheel, one selectable profile per run (see below).
  2 revs per 720° four-stroke cycle.
- Cam: one pulse per 720° cycle, rising at 120°, falling at 300° (same for
  every profile).
- Crank pin: GPIO2. Cam pin: GPIO3.

## Trigger-wheel profiles

Ardu-stim-style: a small table of named crank/cam wheel definitions
(`firmware/src/profiles.c`), selected at runtime with the `p<n>` command
(see below). Teeth/missing-tooth counts are copied from real decoder patterns in
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

## Firmware command protocol

One firmware image, always ready — no boot menu, no reset needed to
change anything. It's a one-line-command protocol over USB serial: a
letter plus an optional argument, no separator (e.g. `r3000`). Every
command gets exactly one `OK ...`/`ERR ...` response line.

| Command | Effect |
|---|---|
| `p<n>` | Select trigger-wheel profile `n` (1-based, see `l`). Requires generation stopped. |
| `r<n>` | Set RPM to `n`. Live if generation is running — takes effect at the next 720° cycle boundary. |
| `g1` / `g0` | Start/stop crank+cam generation. Stopping generation also stops capture. |
| `c1` / `c0` | Start/stop capture + live per-cycle angle report. Requires generation running. |
| `l` | List trigger-wheel profiles. |
| `?` | Print status (`STATUS profile=... rpm=... gen=0/1 capture=0/1`). |
| `h` | Print command help. |

Generation is continuous double-buffered crank/cam output at a constant,
live-adjustable RPM (two real PIO/DMA hardware bugs around this refill
path are documented in `CLAUDE.md`). Capture is continuous and
hardware-gapless: a second PIO program samples 6 input channels
(GPIO6–11, wire GPIO2→6 and GPIO3→7 for loopback) alongside generation,
and after every completed 720° cycle prints each channel's rise/fall
angle (or "not detected") — no pass/fail tolerance in firmware; that's
left to a client watching the stream (see the Python client below).
Printing happens in the main loop, not the DMA IRQs, so a slow print
never stalls generation or capture — it just skips reporting that cycle
("N cycle(s) skipped").

Known cosmetic limitation, unchanged from the design this replaced:
capture free-runs on its own DMA chain, independent of generation's, so
its buffer boundary isn't forced to realign with generation's cycle
boundary every cycle. Occasional sub-sample phase drift between the two
can make channel 0 (the crank loopback channel itself) briefly land in
the wrong reference window and print a nonsense angle. This does not
affect any other channel — cam and any real ECU channel are computed
relative to whichever window actually contains them and stay correct
regardless (verified on hardware: cam held the correct ~120°/300° angle
across 175/175 cycles at 7000 RPM, including cycles where ch0 showed the
artifact).

A related bug — a timestamp with no closing crank reference yet (the
common case for the last-captured revolution in a buffer) could get
misread as one whole revolution further along than it really was,
reporting e.g. cam's own 120°/300° pulse as 480°/660° — was fixed in
`capture_analysis.c`'s `convert_to_angle`/`find_window`. Re-verified on
hardware: 150/150 cycles clean across 1000–7000 RPM with no 360°-shifted
reading.

## Python client

`python/crankcam/` is a thin client for the protocol above (`pip install
-e python/`, needs `pyserial`):

```python
from crankcam import CrankCamBoard

with CrankCamBoard("/dev/ttyACM0") as board:
    board.select_profile(1)
    board.set_rpm(3000)
    board.start_gen()
    board.start_capture()
    for _ in range(10):
        print(board.read_cycle())
    board.stop_capture()
    board.stop_gen()
```

Verified against real hardware: connect, profile select, gen/capture
start-stop, live RPM change, and error paths all confirmed working.

### GUI

`crankcam-gui` (installed by the `pip install -e python/` above) is a
small Tk app: port/connect, profile/RPM/gen/capture controls, and a
0-720° timeline canvas that redraws with each cycle's rise/fall angles.
It runs one dedicated thread owning all serial I/O — the GUI thread only
exchanges queued messages with it, since the board client isn't safe to
call from two threads at once. Run with `crankcam-gui` or
`python -m crankcam.gui`; verified against real hardware.

## Layout

```
firmware/
  src/main.c              command loop over USB serial (see protocol table above)
  src/profiles.c/.h       selectable crank/cam trigger-wheel profiles
  src/event_table.c/.h    crank/cam geometry + event-table build
  src/gen_fire.c/.h       ping-pong DMA config for continuous generation
  src/capture_analysis.c/.h  edge/angle/live-report logic
  src/engine.c/.h         generation+capture state machine, DMA IRQ handler
  src/event_gen.pio       crank/cam waveform generator (PIO)
  src/capture.pio         6-channel edge capture (PIO)
  CMakeLists.txt
  pico_sdk_import.cmake
  env.sh                  sets PICO_SDK_PATH (edit for your checkout)
python/
  crankcam/board.py       Python client for the command protocol
  crankcam/gui.py         Tk GUI (crankcam-gui)
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
