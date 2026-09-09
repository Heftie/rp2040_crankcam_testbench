"""Python client for the RP2040 crank/cam trigger simulator's USB-serial
boot menu and per-mode text protocol.

The firmware (see firmware/src/main.c) has no binary command protocol --
it is a plain-text menu read a keypress at a time over USB CDC, then one
of four modes that runs forever printing human-readable reports. This
client drives that same menu and parses those reports, so nothing on the
firmware side needs to change.

Typical usage::

    from crankcam import CrankCamBoard

    with CrankCamBoard("/dev/ttyACM0") as board:
        profiles = board.connect()
        board.select_profile(1)          # "60-2 (Bosch/GM)"
        live = board.select_mode(4)      # returns a LiveMode
        live.select_rpm(1)               # 1000 RPM
        live.start()
        for _ in range(10):
            print(live.read_cycle())
        live.stop()

Each mode is exclusive for the lifetime of one board session: like the
firmware itself, picking a different profile or mode means resetting the
board and reconnecting.
"""

from __future__ import annotations

import re
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import serial

DEFAULT_BAUDRATE = 115200

# Mirrors firmware/src/capture_analysis.h CAPTURE_PIN_COUNT. Keep in sync
# if that constant ever changes -- report_pulse_angles() prints exactly
# this many "  chN: ..." lines per cycle, and there is no other marker of
# where one live-mode cycle report ends.
CAPTURE_PIN_COUNT = 6


class ProtocolError(RuntimeError):
    """The board's serial output didn't match the expected menu/report text."""


@dataclass
class Profile:
    index: int  # 1-based, as shown (and sent back) in the firmware's menu
    name: str


@dataclass
class ChannelEdges:
    rising: int
    falling: int
    edges: List[Tuple[bool, float]]  # (is_rising, time_ms), first few only


@dataclass
class SpecCheck:
    name: str
    channel: int
    passed: bool
    failures: List[str]


@dataclass
class CaptureResult:
    edges: Dict[int, ChannelEdges]
    refs_ms: List[float]
    rev0_window: Optional[int]
    have_refs: bool
    cam_angles: List[Tuple[str, float, float]]  # ("rise"/"fall", time_ms, angle_deg)
    checks: List[SpecCheck]


@dataclass
class CycleReport:
    cycle: int
    skipped: int
    have_refs: bool
    # channel -> (rise_deg, fall_deg); either half is None if that edge
    # wasn't found, and the dict is empty when have_refs is False.
    channels: Dict[int, Tuple[Optional[float], Optional[float]]] = field(default_factory=dict)


class _SerialLink:
    """Thin wrapper around the serial port with the two read primitives
    every mode needs: newline-terminated report lines, and the three
    fixed prompts the firmware prints without a trailing newline
    ("profile: ", "select mode ...: ", "RPM: ")."""

    def __init__(self, ser: serial.Serial):
        self._ser = ser

    def send_key(self, ch: str) -> None:
        self._ser.write((ch + "\n").encode("ascii"))

    def press_enter(self) -> None:
        self._ser.write(b"\n")

    def readline(self) -> str:
        raw = self._ser.readline()
        if not raw:
            raise ProtocolError("timed out waiting for a line from the board")
        return raw.decode(errors="replace").rstrip("\r\n")

    def read_until_text(self, substr: str, overall_timeout: float = 5.0) -> List[str]:
        """Reads raw bytes until substr appears anywhere in the accumulated
        buffer, then returns it split into lines. Used only for the three
        prompts above, since readline() would otherwise block for the
        full serial timeout waiting for a newline that never comes."""
        deadline = time.monotonic() + overall_timeout
        buf = ""
        while substr not in buf:
            if time.monotonic() > deadline:
                raise ProtocolError(
                    f"timed out waiting for {substr!r} in board output; got so far: {buf!r}"
                )
            chunk = self._ser.read(max(1, self._ser.in_waiting or 1))
            if chunk:
                buf += chunk.decode(errors="replace")
        return buf.splitlines()


class CrankCamBoard:
    def __init__(self, port: str, baudrate: int = DEFAULT_BAUDRATE, timeout: float = 5.0):
        self._ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        self._link = _SerialLink(self._ser)
        self.profiles: List[Profile] = []

    def close(self) -> None:
        self._ser.close()

    def __enter__(self) -> "CrankCamBoard":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def connect(self) -> List[Profile]:
        """Reads the boot-time profile menu. Call this once, right after
        the board has been reset/replugged (main() sleeps 1.5s before
        printing it, so opening the port promptly is fine)."""
        lines = self._link.read_until_text("profile: ")
        profiles = []
        for line in lines:
            m = re.match(r"\s*(\d+)\)\s*(.+)", line)
            if m:
                profiles.append(Profile(int(m.group(1)), m.group(2)))
        if not profiles:
            raise ProtocolError(
                "no profile menu found -- reset the board and call connect() again"
            )
        self.profiles = profiles
        return profiles

    def select_profile(self, index: int) -> None:
        """index is the 1-based number shown by connect()'s profile menu."""
        valid = {p.index for p in self.profiles}
        if index not in valid:
            raise ValueError(f"profile {index} not in {sorted(valid)}; call connect() first")
        self._link.send_key(str(index))
        echo = self._link.readline()
        if echo.strip() != str(index):
            raise ProtocolError(f"expected profile echo {index!r}, got {echo!r}")
        # Mode menu follows immediately; leave it for select_mode() to read
        # so its own prompt text stays close to the code that consumes it.
        self._link.read_until_text("select mode")

    def select_mode(self, mode: int):
        """Sends the mode digit and returns a controller for that mode:
        1 -> ContinuousMode, 2 -> SingleshotMode,
        3 -> CaptureTestMode, 4 -> LiveMode."""
        if mode not in (1, 2, 3, 4):
            raise ValueError("mode must be 1, 2, 3, or 4")
        self._link.send_key(str(mode))
        echo = self._link.readline()
        if echo.strip() != str(mode):
            raise ProtocolError(f"expected mode echo {mode!r}, got {echo!r}")
        if mode == 1:
            return ContinuousMode(self._link)
        if mode == 2:
            return SingleshotMode(self._link)
        if mode == 3:
            return CaptureTestMode(self._link)
        return LiveMode(self._link)


class ContinuousMode:
    """Mode 1: continuous double-buffered generation. Runs forever with no
    further input or per-cycle output -- there is nothing to poll."""

    _BANNER_RE = re.compile(
        r"event_gen double-buffered: crank pin=(\d+), cam pin=(\d+), f_pio=([\d.]+) Hz"
    )

    def __init__(self, link: _SerialLink):
        line = link.readline()
        m = self._BANNER_RE.search(line)
        if not m:
            raise ProtocolError(f"unexpected mode-1 banner: {line!r}")
        self.crank_pin = int(m.group(1))
        self.cam_pin = int(m.group(2))
        self.f_pio_hz = float(m.group(3))


class SingleshotMode:
    """Mode 2: single-shot 2-rev diagnostic. Fires one burst per fire() call."""

    _BANNER_RE = re.compile(
        r"singleshot: crank pin=(\d+), cam pin=(\d+), const RPM=([\d.]+), "
        r"cycle=([\d.]+) ms"
    )
    _FIRED_RE = re.compile(r"fired, runs for ([\d.]+) ms")

    def __init__(self, link: _SerialLink):
        self._link = link
        line = link.readline()
        m = self._BANNER_RE.search(line)
        if not m:
            raise ProtocolError(f"unexpected mode-2 banner: {line!r}")
        self.crank_pin = int(m.group(1))
        self.cam_pin = int(m.group(2))
        self.const_rpm = float(m.group(3))
        self.cycle_ms = float(m.group(4))

    def fire(self) -> float:
        """Fires one burst; returns how many ms it runs for before holding."""
        self._link.press_enter()
        line = self._link.readline()
        m = self._FIRED_RE.search(line)
        if not m:
            raise ProtocolError(f"unexpected mode-2 fire response: {line!r}")
        return float(m.group(1))


_EDGE_LINE_RE = re.compile(
    r"\s*ch(\d+): ((?:[RF]@[\d.]+ms )*)\(rising=(\d+) falling=(\d+)\)"
)
_EDGE_SAMPLE_RE = re.compile(r"([RF])@([\d.]+)ms")
_REF_COUNT_RE = re.compile(r"crank references \(angle=0deg points\) found: (\d+)")
_REF_LINE_RE = re.compile(r"\s*ref\[(\d+)\] @ ([\d.]+)ms")
_REV0_RE = re.compile(r"cam pulse found in reference window (-?\d+)")
_CAM_ANGLE_RE = re.compile(r"\s*(rise|fall)@([\d.]+)ms -> (-?[\d.]+) deg")
_SPEC_HEADER_RE = re.compile(r"\[(.+)\] ch(\d+) expect")
_SPEC_FAIL_RE = re.compile(r"\s*FAIL: (.+)")


class CaptureTestMode:
    """Mode 3: one-shot generation + 6-channel capture + angle + pass/fail,
    fired once per run_capture() call."""

    _BANNER_RE = re.compile(
        r"capture_test: crank pin=(\d+), cam pin=(\d+), capture base=(\d+) \((\d+) ch\), "
        r"sample=([\d.]+) Hz, window=([\d.]+) ms \((\d+) samples\)"
    )

    def __init__(self, link: _SerialLink):
        self._link = link
        line = link.readline()
        m = self._BANNER_RE.search(line)
        if not m:
            raise ProtocolError(f"unexpected mode-3 banner: {line!r}")
        self.crank_pin = int(m.group(1))
        self.cam_pin = int(m.group(2))
        self.capture_base_pin = int(m.group(3))
        self.capture_pin_count = int(m.group(4))
        self.sample_hz = float(m.group(5))
        self.window_ms = float(m.group(6))
        self.capture_samples = int(m.group(7))
        link.readline()  # "wire GPIO2->GPIO6 ..., then press Enter to capture."

    def run_capture(self) -> CaptureResult:
        """Fires one capture burst and returns the parsed report. Blocks
        until the firmware finishes (window_ms plus analysis, well under
        the link's default timeout for a normal-size capture)."""
        self._link.press_enter()
        self._link.readline()  # "capturing..."
        self._link.readline()  # "done. Edges found:"

        edges: Dict[int, ChannelEdges] = {}
        for _ in range(self.capture_pin_count):
            line = self._link.readline()
            m = _EDGE_LINE_RE.match(line)
            if not m:
                raise ProtocolError(f"unexpected edge line: {line!r}")
            ch = int(m.group(1))
            samples = [(kind == "R", float(t)) for kind, t in _EDGE_SAMPLE_RE.findall(m.group(2))]
            edges[ch] = ChannelEdges(int(m.group(3)), int(m.group(4)), samples)

        line = self._link.readline()
        m = _REF_COUNT_RE.match(line)
        if not m:
            raise ProtocolError(f"expected crank-reference count, got: {line!r}")
        n_refs = int(m.group(1))
        refs_ms = []
        for _ in range(n_refs):
            line = self._link.readline()
            m = _REF_LINE_RE.match(line)
            if not m:
                raise ProtocolError(f"expected a ref[] line, got: {line!r}")
            refs_ms.append(float(m.group(2)))

        have_refs = n_refs >= 2
        rev0_window = None
        cam_angles: List[Tuple[str, float, float]] = []
        checks: List[SpecCheck] = []

        if not have_refs:
            self._link.readline()  # "  not enough references to compute C_rev."
        else:
            line = self._link.readline()
            m = _REV0_RE.match(line)
            if not m:
                raise ProtocolError(f"expected rev0-window line, got: {line!r}")
            rev0_window = int(m.group(1))

            self._link.readline()  # "cam angles (expect ~120deg rise, ~300deg fall, ...):"
            # Cam has exactly one rise and one fall edge by profile design;
            # read that many lines as long as they parse as angle lines.
            while True:
                line = self._link.readline()
                m = _CAM_ANGLE_RE.match(line)
                if not m:
                    break
                cam_angles.append((m.group(1), float(m.group(2)), float(m.group(3))))
            # `line` now holds the first non-angle line: "pass/fail evaluation:"
            for _ in range(2):  # two EcuOutputSpec checks, hardcoded in mode_capture.c
                header = self._link.readline()
                hm = _SPEC_HEADER_RE.match(header)
                if not hm:
                    raise ProtocolError(f"expected spec header, got: {header!r}")
                failures = []
                while True:
                    line = self._link.readline()
                    fm = _SPEC_FAIL_RE.match(line)
                    if fm:
                        failures.append(fm.group(1))
                        continue
                    break
                # `line` is now the final PASS / "-> FAIL (see above)" line.
                passed = line.strip() == "PASS"
                checks.append(SpecCheck(hm.group(1), int(hm.group(2)), passed, failures))

        self._link.readline()  # "Press Enter to capture again."

        return CaptureResult(edges, refs_ms, rev0_window, have_refs, cam_angles, checks)


_LIVE_RPM_CHOICES = (1000.0, 3000.0, 5000.0, 7000.0)
_LIVE_SETUP_RE = re.compile(
    r"live capture: ([\d.]+) RPM, cycle=([\d.]+)ms, capturing (\d+) samples/cycle"
)
_CYCLE_HEADER_RE = re.compile(r"cycle (\d+):")
_SKIPPED_RE = re.compile(r"\s*\((\d+) earlier cycle")
_CHANNEL_RE = re.compile(
    r"\s*ch(\d+): rise=(--|[\d.]+deg) fall=(--|[\d.]+deg)"
)
_CHANNEL_NOT_DETECTED_RE = re.compile(r"\s*ch(\d+): not detected")
_STOPPED_RE = re.compile(r"stopped after (\d+) cycles\.")


class LiveMode:
    """Mode 4: continuous per-cycle live capture. select_rpm() then start()
    before read_cycle(); stop() ends the report loop."""

    def __init__(self, link: _SerialLink):
        self._link = link
        self._started = False

    def select_rpm(self, choice: int) -> float:
        """choice is 1..4 for {1000, 3000, 5000, 7000} RPM, matching the
        firmware's fixed menu. Returns the resulting RPM."""
        if choice not in (1, 2, 3, 4):
            raise ValueError("choice must be 1, 2, 3, or 4")
        lines = self._link.read_until_text("RPM: ")
        if not any(str(choice) + ")" in l for l in lines):
            raise ProtocolError(f"RPM menu missing option {choice}: {lines!r}")
        self._link.send_key(str(choice))
        echo = self._link.readline()
        if echo.strip() != str(choice):
            raise ProtocolError(f"expected RPM echo {choice!r}, got {echo!r}")

        line = self._link.readline()
        m = _LIVE_SETUP_RE.search(line)
        if not m:
            raise ProtocolError(f"unexpected mode-4 setup line: {line!r}")
        self.rpm = float(m.group(1))
        self.cycle_ms = float(m.group(2))
        self.samples_per_cycle = int(m.group(3))
        for _ in range(3):  # wiring line, "gapless" line, "press Enter..." line
            self._link.readline()
        return self.rpm

    def start(self) -> None:
        if self._started:
            raise ProtocolError("already started")
        self._link.press_enter()
        self._started = True

    def read_cycle(self) -> CycleReport:
        """Blocks for one cycle's report. Call stop() instead once done --
        do not call this again after stop()."""
        if not self._started:
            raise ProtocolError("call start() first")
        line = self._link.readline()
        m = _CYCLE_HEADER_RE.match(line)
        if not m:
            raise ProtocolError(f"expected cycle header, got: {line!r}")
        cycle = int(m.group(1))

        skipped = 0
        line = self._link.readline()
        sm = _SKIPPED_RE.match(line)
        if sm:
            skipped = int(sm.group(1))
            line = self._link.readline()

        if line.strip() == "no valid cycle reference this pass":
            return CycleReport(cycle, skipped, have_refs=False)

        channels: Dict[int, Tuple[Optional[float], Optional[float]]] = {}
        for i in range(CAPTURE_PIN_COUNT):
            cm = _CHANNEL_RE.match(line)
            if cm:
                ch = int(cm.group(1))
                rise = None if cm.group(2) == "--" else float(cm.group(2)[:-3])
                fall = None if cm.group(3) == "--" else float(cm.group(3)[:-3])
            else:
                nm = _CHANNEL_NOT_DETECTED_RE.match(line)
                if not nm:
                    raise ProtocolError(f"expected channel line, got: {line!r}")
                ch = int(nm.group(1))
                rise = fall = None
            channels[ch] = (rise, fall)
            if i < CAPTURE_PIN_COUNT - 1:
                line = self._link.readline()
        return CycleReport(cycle, skipped, have_refs=True, channels=channels)

    def stop(self) -> int:
        """Signals the board to stop; returns the number of cycles run.
        Discards any cycle reports still buffered ahead of the "stopped"
        line -- the board keeps producing them until it next checks for
        this keypress, which happens once per completed cycle."""
        self._link.press_enter()
        while True:
            line = self._link.readline()
            m = _STOPPED_RE.match(line)
            if m:
                self._started = False
                return int(m.group(1))
