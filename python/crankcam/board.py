"""Python client for the RP2040 crank/cam trigger simulator's USB-serial
command protocol.

The firmware (see firmware/src/main.c, firmware/src/engine.c) has no boot
menu -- it's always ready for one-line commands over USB CDC serial:

    p<n>   select trigger-wheel profile n (1-based, see list_profiles())
    r<n>   set RPM to n; live if generation is running
    g1/g0  start/stop crank+cam generation
    c1/c0  start/stop capture + live per-cycle angle report (needs gen=1)
    l      list trigger-wheel profiles
    ?      status
    h      help

Every command gets exactly one "OK ..."/"ERR ..." response line (list
profiles are printed before that line; status is its own "STATUS ..."
line). While capture is running, "cycle N:" report blocks can appear on
the wire at any time, interleaved with command responses -- this client
transparently pulls those into a queue so command calls never see them,
and read_cycle() drains that queue.

Typical usage::

    from crankcam import CrankCamBoard

    with CrankCamBoard("/dev/ttyACM0") as board:
        board.select_profile(1)      # "60-2 (Bosch/GM)"
        board.set_rpm(3000)
        board.start_gen()
        board.set_rpm(3500)          # takes effect at the next cycle boundary
        board.start_capture()
        for _ in range(10):
            print(board.read_cycle())
        board.stop_capture()
        board.stop_gen()
"""

from __future__ import annotations

import re
from collections import deque
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import serial

DEFAULT_BAUDRATE = 115200

# Mirrors firmware/src/capture_analysis.h CAPTURE_PIN_COUNT. report_pulse_angles()
# prints exactly this many "  chN: ..." lines per cycle.
CAPTURE_PIN_COUNT = 6


class ProtocolError(RuntimeError):
    """The board's serial output didn't match the expected protocol, or a
    command got an ERR response."""


@dataclass
class Profile:
    index: int  # 1-based, as used by select_profile()
    name: str


@dataclass
class Status:
    profile: str
    rpm: int
    gen: bool
    capture: bool


@dataclass
class ChannelReport:
    # False means the firmware found no rise or fall edge at all on this
    # channel this cycle ("not detected": e.g. an unwired pin). True
    # means at least one edge was found, but rise_deg/fall_deg can still
    # individually be None if that particular edge's timestamp didn't
    # fall inside any known crank-reference window ("--" on the wire) --
    # a different condition (see CLAUDE.md's ch0 window-drift note),
    # not "no signal".
    detected: bool
    rise_deg: Optional[float]
    fall_deg: Optional[float]


@dataclass
class CycleReport:
    cycle: int
    skipped: int
    have_refs: bool
    # channel -> ChannelReport; empty when have_refs is False.
    channels: Dict[int, ChannelReport] = field(default_factory=dict)


_STATUS_RE = re.compile(r"STATUS profile=(.+) rpm=(\d+) gen=([01]) capture=([01])")
_PROFILE_LINE_RE = re.compile(r"\s*(\d+)\)\s*(.+)")
_CYCLE_HEADER_RE = re.compile(r"cycle (\d+):")
_SKIPPED_RE = re.compile(r"\s*\((\d+) earlier cycle")
_CHANNEL_RE = re.compile(r"\s*ch(\d+): rise=(--|[\d.]+deg) fall=(--|[\d.]+deg)")
_CHANNEL_NOT_DETECTED_RE = re.compile(r"\s*ch(\d+): not detected")


def _parse_status(line: str) -> Status:
    m = _STATUS_RE.match(line)
    if not m:
        raise ProtocolError(f"unexpected status line: {line!r}")
    return Status(m.group(1), int(m.group(2)), m.group(3) == "1", m.group(4) == "1")


class CrankCamBoard:
    def __init__(self, port: str, baudrate: int = DEFAULT_BAUDRATE, timeout: float = 5.0):
        self._ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        self._cycle_queue: deque = deque()

    def close(self) -> None:
        self._ser.close()

    def __enter__(self) -> "CrankCamBoard":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # -- low-level I/O --

    def _readline(self) -> str:
        raw = self._ser.readline()
        if not raw:
            raise ProtocolError("timed out waiting for a line from the board")
        return raw.decode(errors="replace").rstrip("\r\n")

    def _read_cycle_block(self, header_line: str) -> CycleReport:
        m = _CYCLE_HEADER_RE.match(header_line)
        if not m:
            raise ProtocolError(f"expected cycle header, got: {header_line!r}")
        cycle = int(m.group(1))

        skipped = 0
        line = self._readline()
        sm = _SKIPPED_RE.match(line)
        if sm:
            skipped = int(sm.group(1))
            line = self._readline()

        if line.strip() == "no valid cycle reference this pass":
            return CycleReport(cycle, skipped, have_refs=False)

        channels: Dict[int, ChannelReport] = {}
        for i in range(CAPTURE_PIN_COUNT):
            cm = _CHANNEL_RE.match(line)
            if cm:
                ch = int(cm.group(1))
                rise = None if cm.group(2) == "--" else float(cm.group(2)[:-3])
                fall = None if cm.group(3) == "--" else float(cm.group(3)[:-3])
                channels[ch] = ChannelReport(detected=True, rise_deg=rise, fall_deg=fall)
            else:
                nm = _CHANNEL_NOT_DETECTED_RE.match(line)
                if not nm:
                    raise ProtocolError(f"expected channel line, got: {line!r}")
                ch = int(nm.group(1))
                channels[ch] = ChannelReport(detected=False, rise_deg=None, fall_deg=None)
            if i < CAPTURE_PIN_COUNT - 1:
                line = self._readline()
        return CycleReport(cycle, skipped, have_refs=True, channels=channels)

    def _transact(self, cmd: str) -> Tuple[str, List[str]]:
        """Sends one command line and reads until its OK/ERR/STATUS
        response, transparently queuing any cycle-report blocks that
        arrive first. Returns (response_line, other_lines_seen)."""
        self._ser.write((cmd + "\n").encode("ascii"))
        extra: List[str] = []
        while True:
            line = self._readline()
            if line.startswith("cycle "):
                self._cycle_queue.append(self._read_cycle_block(line))
                continue
            if line.startswith(("OK", "ERR", "STATUS")):
                return line, extra
            extra.append(line)

    def _transact_ok(self, cmd: str) -> None:
        result, _ = self._transact(cmd)
        if not result.startswith("OK"):
            raise ProtocolError(result)

    # -- connection --

    def connect(self) -> Status:
        """Queries status to confirm the board is alive and responding.
        There's no menu or handshake to drive -- the board is ready for
        commands as soon as it's plugged in, whether freshly reset or
        already running -- so this is just status() under a more
        discoverable name for "first call after opening the port"."""
        return self.status()

    # -- commands --

    def status(self) -> Status:
        result, _ = self._transact("?")
        return _parse_status(result)

    def list_profiles(self) -> List[Profile]:
        result, extra = self._transact("l")
        if not result.startswith("OK"):
            raise ProtocolError(result)
        profiles = []
        for line in extra:
            m = _PROFILE_LINE_RE.match(line)
            if m:
                profiles.append(Profile(int(m.group(1)), m.group(2)))
        return profiles

    def help(self) -> str:
        _, extra = self._transact("h")
        return "\n".join(extra)

    def select_profile(self, index: int) -> None:
        """index is 1-based, as shown by list_profiles(). Requires
        generation to be stopped."""
        self._transact_ok(f"p{index}")

    def set_rpm(self, rpm: int) -> None:
        """Takes effect at the next 720deg cycle boundary if generation
        is already running."""
        self._transact_ok(f"r{rpm}")

    def start_gen(self) -> None:
        self._transact_ok("g1")

    def stop_gen(self) -> None:
        """Also stops capture, if it was running."""
        self._transact_ok("g0")

    def start_capture(self) -> None:
        """Requires generation to already be running."""
        self._transact_ok("c1")

    def stop_capture(self) -> None:
        self._transact_ok("c0")

    def read_cycle(self) -> CycleReport:
        """Blocks for the next capture cycle report (drains any queued up
        while a command was in flight first)."""
        while True:
            if self._cycle_queue:
                return self._cycle_queue.popleft()
            line = self._readline()
            if line.startswith("cycle "):
                return self._read_cycle_block(line)
            # a stray OK/ERR/STATUS/help line with no command in flight
            # shouldn't happen; ignore rather than fail a live poll loop
