"""Hardware-in-the-loop unittest suite: drives a REAL board over the
Python API (crankcam.board.CrankCamBoard) and checks protocol-level
behavior -- profile selection rules, RPM bounds, gen/capture lifecycle
rules, cycle-report structure. Needs an actual board connected; skips
cleanly (with a clear reason) if one isn't reachable, so it's safe to
run alongside the offline test_board.py suite in any environment.

Two already-known, unfixed hardware issues (see CLAUDE.md) mean this
suite deliberately does NOT assert on:
  - read_cycle() always having have_refs=True right after a fresh
    start_capture() -- roughly half of fresh sessions never find a
    valid crank reference at all, for the whole session (a real, open
    bug, not a test flake)
  - ch0's rise/fall angles always being present -- window-drift
    cosmetic issue, frequency highly session-dependent, not RPM-tied
Both are exercised structurally (type/shape checks on whatever comes
back) rather than for a specific value, so this suite stays a reliable
regression check against NEW breakage instead of re-flagging those two
known issues on every run. ch1 (cam) is unaffected by either and IS
asserted on -- a ch1 failure here is a real regression.

Run with:
    cd python && python -m unittest discover -s tests -p 'test_hardware.py' -v
Point at a specific port with the CRANKCAM_PORT environment variable
(default /dev/ttyACM0). Talks to whatever profile/RPM/gen/capture state
the board is already in -- each test resets to a known idle state
(profile 1, 1000 RPM, gen=0, capture=0) in setUp/addCleanup, so tests
don't depend on run order, but do expect exclusive use of the board
while this suite runs.
"""

from __future__ import annotations

import os
import unittest

from crankcam import CrankCamBoard, ProtocolError, Status

PORT = os.environ.get("CRANKCAM_PORT", "/dev/ttyACM0")

# Mirror firmware/src/event_table.h -- see CLAUDE.md's "Key constants".
ENGINE_MAX_RPM = 20000
CAPTURE_MIN_RPM = 1000


def _reachable() -> str:
    """Returns "" if a board answers on PORT, else a human-readable
    reason (used directly as the unittest skip message)."""
    try:
        with CrankCamBoard(PORT, timeout=1.0) as board:
            board.status()
        return ""
    except Exception as e:  # noqa: BLE001 -- any failure just means "skip"
        return f"no crankcam board reachable on {PORT} ({type(e).__name__}: {e}); set CRANKCAM_PORT"


_SKIP_REASON = _reachable()


@unittest.skipIf(bool(_SKIP_REASON), _SKIP_REASON)
class HardwareTestCase(unittest.TestCase):
    """Base class: one board connection shared by the whole test class,
    reset to a known idle state before and after every test."""

    board: CrankCamBoard

    @classmethod
    def setUpClass(cls):
        cls.board = CrankCamBoard(PORT, timeout=2.0)

    @classmethod
    def tearDownClass(cls):
        cls.board.close()

    def setUp(self):
        self._reset_idle()
        self.addCleanup(self._reset_idle)

    def _reset_idle(self):
        status = self.board.status()
        if status.capture:
            self.board.stop_capture()
        if status.gen:
            self.board.stop_gen()
        self.board.select_profile(1)
        self.board.set_rpm(1000)


class StatusAndProfileTests(HardwareTestCase):
    def test_status_reflects_idle_state(self):
        status = self.board.status()
        self.assertIsInstance(status, Status)
        self.assertFalse(status.gen)
        self.assertFalse(status.capture)

    def test_list_profiles_nonempty_and_1_indexed(self):
        profiles = self.board.list_profiles()
        self.assertGreaterEqual(len(profiles), 1)
        indices = [p.index for p in profiles]
        self.assertEqual(indices, sorted(set(indices)), "profile indices should be sorted, no dupes")
        self.assertEqual(indices[0], 1)

    def test_select_each_listed_profile(self):
        for p in self.board.list_profiles():
            self.board.select_profile(p.index)
            self.assertEqual(self.board.status().profile, p.name)

    def test_select_profile_out_of_range_rejected(self):
        profiles = self.board.list_profiles()
        with self.assertRaises(ProtocolError):
            self.board.select_profile(len(profiles) + 1)

    def test_select_profile_while_gen_running_rejected(self):
        self.board.start_gen()
        with self.assertRaises(ProtocolError):
            self.board.select_profile(1)


class RpmTests(HardwareTestCase):
    def test_set_rpm_accepted_in_range(self):
        self.board.set_rpm(2500)
        self.assertEqual(self.board.status().rpm, 2500)

    def test_set_rpm_zero_rejected(self):
        with self.assertRaises(ProtocolError):
            self.board.set_rpm(0)

    def test_set_rpm_above_ceiling_rejected(self):
        with self.assertRaises(ProtocolError):
            self.board.set_rpm(ENGINE_MAX_RPM + 1)

    def test_set_rpm_at_ceiling_accepted(self):
        self.board.set_rpm(ENGINE_MAX_RPM)
        self.assertEqual(self.board.status().rpm, ENGINE_MAX_RPM)

    def test_set_rpm_live_while_gen_running(self):
        self.board.start_gen()
        self.board.set_rpm(3000)
        self.assertEqual(self.board.status().rpm, 3000)


class GenCaptureLifecycleTests(HardwareTestCase):
    def test_capture_requires_gen_running(self):
        with self.assertRaises(ProtocolError):
            self.board.start_capture()

    def test_start_gen_reflected_in_status(self):
        self.board.start_gen()
        self.assertTrue(self.board.status().gen)

    def test_start_capture_reflected_in_status(self):
        self.board.start_gen()
        self.board.start_capture()
        self.assertTrue(self.board.status().capture)

    def test_capture_below_min_rpm_rejected(self):
        self.board.set_rpm(CAPTURE_MIN_RPM - 1)
        self.board.start_gen()
        with self.assertRaises(ProtocolError):
            self.board.start_capture()

    def test_capture_at_min_rpm_accepted(self):
        self.board.set_rpm(CAPTURE_MIN_RPM)
        self.board.start_gen()
        self.board.start_capture()
        self.assertTrue(self.board.status().capture)

    def test_stop_gen_also_stops_capture(self):
        self.board.start_gen()
        self.board.start_capture()
        self.board.stop_gen()
        status = self.board.status()
        self.assertFalse(status.gen)
        self.assertFalse(status.capture)

    def test_repeated_start_capture_is_idempotent(self):
        self.board.start_gen()
        self.board.start_capture()
        self.board.start_capture()  # should not raise
        self.assertTrue(self.board.status().capture)

    def test_repeated_stop_gen_is_idempotent(self):
        self.board.stop_gen()  # already idle from setUp; should not raise
        self.assertFalse(self.board.status().gen)


class CycleReportTests(HardwareTestCase):
    def test_read_cycle_returns_structurally_valid_reports(self):
        self.board.start_gen()
        self.board.start_capture()
        for _ in range(5):
            report = self.board.read_cycle()
            self.assertIsInstance(report.cycle, int)
            self.assertGreaterEqual(report.cycle, 1)
            self.assertIsInstance(report.skipped, int)
            self.assertGreaterEqual(report.skipped, 0)
            self.assertIsInstance(report.have_refs, bool)
            if report.have_refs:
                self.assertGreater(len(report.channels), 0)
                for ch, cr in report.channels.items():
                    self.assertIsInstance(ch, int)
                    self.assertIsInstance(cr.detected, bool)
                    if not cr.detected:
                        self.assertIsNone(cr.rise_deg)
                        self.assertIsNone(cr.fall_deg)
            else:
                self.assertEqual(report.channels, {})
            # not asserting have_refs is ever True -- see module docstring

    def test_cycle_numbers_increase_monotonically(self):
        self.board.start_gen()
        self.board.start_capture()
        last = None
        for _ in range(5):
            report = self.board.read_cycle()
            if last is not None:
                self.assertGreater(report.cycle, last)
            last = report.cycle

    def test_default_wiring_channel1_cam_detected(self):
        # ch1 (cam) itself is unaffected by both known open bugs (crank-
        # reference loss and ch0 window-drift are crank-specific) -- but
        # when have_refs is False, the firmware reports NO channel data
        # at all that cycle, ch1 included, so only cycles that actually
        # got a reference are meaningful here. A session that never gets
        # one (the other known open bug -- see CLAUDE.md) makes this
        # inconclusive, not a ch1 regression.
        self.board.start_gen()
        self.board.start_capture()
        found = False
        saw_have_refs = False
        for _ in range(20):
            report = self.board.read_cycle()
            if not report.have_refs:
                continue
            saw_have_refs = True
            cr = report.channels.get(1)
            if cr is not None and cr.detected:
                found = True
                break
        if not saw_have_refs:
            self.skipTest(
                "have_refs was never True in 20 cycles -- known open crank-reference "
                "bug (see CLAUDE.md), not a ch1 regression; rerun for a fresh session"
            )
        self.assertTrue(found, "ch1 (cam) never detected across cycles with have_refs=True")


if __name__ == "__main__":
    unittest.main()
