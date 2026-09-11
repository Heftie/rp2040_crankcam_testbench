"""RPM-step testbench: drives the board across a range of RPM steps and
verifies the live capture report stream stays parseable and consistent at
each step -- the same board.read_cycle() path the GUI's Worker thread
uses, hammered in a tight loop (no artificial per-iteration delay), so
this reproduces at the protocol level the drain-rate bug the GUI Worker
had (see git history: Worker.run() throttling reads to ~20/s via
cmd_q.get(timeout=0.05), starving read_cycle() above ~2400 RPM).

Per step, holds RPM for --duration seconds and counts:
  - cycles seen, skipped cycles (firmware-reported), cycle-number gaps
    beyond the reported skip count (protocol desync)
  - parse errors (ProtocolError from read_cycle -- corrupted/misaligned
    line, the failure mode the GUI bug produced)
  - cycles with no valid reference, and per-channel "not detected" counts

Pass/fail only checks ch0/ch1 (crank/cam, the two channels the default
loopback wiring drives -- GPIO2->6, GPIO3->7, per CLAUDE.md). ch2-5 are
unwired by default and always read "not detected", so they're reported
but never gate pass/fail; pass --channels to change which ones do.

Default RPM range stops at 7000 -- above that ch0 (crank, the reference
channel) is known-cosmetic-flaky (window-matching drift, see CLAUDE.md)
and would fail here even though cam/ECU channels stay correct. Raise
--stop deliberately if you want to characterize that, not by default.

Run with: python -m crankcam.step_test [--port /dev/ttyACM0] [--profile 1]
          [--start 1000] [--stop 7000] [--step 1000] [--duration 3]
          [--channels 0,1]
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass, field
from typing import List

from .board import CAPTURE_PIN_COUNT, CrankCamBoard, ProtocolError

CAPTURE_MIN_RPM = 1000  # mirrors firmware/src/event_table.h


@dataclass
class StepResult:
    rpm: int
    channels: List[int]
    cycles: int = 0
    skipped: int = 0
    cycle_gaps: int = 0
    parse_errors: int = 0
    no_ref: int = 0
    channel_missing: List[int] = field(default_factory=lambda: [0] * CAPTURE_PIN_COUNT)

    @property
    def ok(self) -> bool:
        if self.cycles == 0 or self.parse_errors or self.cycle_gaps:
            return False
        return all(self.channel_missing[ch] == 0 for ch in self.channels) and self.no_ref == 0


def run_step(board: CrankCamBoard, rpm: int, duration_s: float, channels: List[int]) -> StepResult:
    board.set_rpm(rpm)
    result = StepResult(rpm=rpm, channels=channels)
    deadline = time.monotonic() + duration_s
    last_cycle = None
    while time.monotonic() < deadline:
        try:
            report = board.read_cycle()
        except ProtocolError:
            result.parse_errors += 1
            continue
        result.cycles += 1
        result.skipped += report.skipped
        if last_cycle is not None and report.cycle != last_cycle + 1 + report.skipped:
            result.cycle_gaps += 1
        last_cycle = report.cycle
        if not report.have_refs:
            result.no_ref += 1
            continue
        for ch in range(CAPTURE_PIN_COUNT):
            rise, fall = report.channels.get(ch, (None, None))
            if rise is None and fall is None:
                result.channel_missing[ch] += 1
    return result


def _print_step(r: StepResult) -> None:
    ch_str = " ".join(f"ch{c}={r.channel_missing[c]}" for c in r.channels)
    status = "OK" if r.ok else "FAIL"
    print(
        f"[{status}] rpm={r.rpm:6d}  cycles={r.cycles:4d}  skipped={r.skipped:4d}  "
        f"gaps={r.cycle_gaps}  parse_err={r.parse_errors}  no_ref={r.no_ref}  "
        f"not_detected: {ch_str}"
    )


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--profile", type=int, default=1, help="1-based profile index")
    ap.add_argument("--start", type=int, default=CAPTURE_MIN_RPM)
    ap.add_argument("--stop", type=int, default=7000, help="known-good ch0 range; see module docstring")
    ap.add_argument("--step", type=int, default=1000)
    ap.add_argument("--duration", type=float, default=3.0, help="seconds held per RPM step")
    ap.add_argument("--channels", default="0,1", help="comma-separated channel indices to gate pass/fail on")
    args = ap.parse_args(argv)

    if args.start < CAPTURE_MIN_RPM:
        print(f"--start below CAPTURE_MIN_RPM ({CAPTURE_MIN_RPM}); capture won't start", file=sys.stderr)
        return 2

    channels = [int(c) for c in args.channels.split(",")]
    for ch in channels:
        if not 0 <= ch < CAPTURE_PIN_COUNT:
            print(f"--channels: {ch} out of range 0..{CAPTURE_PIN_COUNT - 1}", file=sys.stderr)
            return 2

    results: List[StepResult] = []
    with CrankCamBoard(args.port) as board:
        board.select_profile(args.profile)
        board.start_gen()
        board.set_rpm(args.start)
        board.start_capture()
        try:
            for rpm in range(args.start, args.stop + 1, args.step):
                r = run_step(board, rpm, args.duration, channels)
                results.append(r)
                _print_step(r)
        finally:
            board.stop_capture()
            board.stop_gen()

    failed = [r for r in results if not r.ok]
    print()
    print(f"{len(results)} steps, {len(failed)} failed")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
