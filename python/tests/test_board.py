"""Offline unit tests for crankcam.board's protocol client.

Pure protocol-parsing logic (no hardware needed): patches
crankcam.board.serial.Serial with FakeSerial, a canned line-by-line
stand-in, so CrankCamBoard can be driven exactly as it would be against
a real board without a port ever being opened.

Run with:
    cd python && python -m unittest discover -s tests
or:
    cd python && python -m unittest tests.test_board -v
"""

from __future__ import annotations

import unittest
from unittest.mock import patch

from crankcam.board import (
    ChannelReport,
    CrankCamBoard,
    CycleReport,
    Profile,
    ProtocolError,
    Status,
)


class FakeSerial:
    """Stands in for serial.Serial: readline() returns preset lines in
    order (as bytes, matching pyserial's contract), write() just records
    what was sent. An empty response list makes readline() return b""
    (pyserial's own signal for "timed out"), matching a real timeout."""

    def __init__(self, port=None, baudrate=None, timeout=None):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.lines: list = []
        self.written: list = []
        self.closed = False

    def queue(self, *lines: str) -> "FakeSerial":
        self.lines.extend(lines)
        return self

    def write(self, data: bytes) -> None:
        self.written.append(data)

    def readline(self) -> bytes:
        if not self.lines:
            return b""
        return (self.lines.pop(0) + "\n").encode()

    def close(self) -> None:
        self.closed = True


def make_board(*lines: str) -> tuple[CrankCamBoard, FakeSerial]:
    """Builds a CrankCamBoard backed by a FakeSerial preloaded with
    `lines`. The patch only needs to live for the constructor call --
    CrankCamBoard keeps its own reference to the fake afterward."""
    with patch("crankcam.board.serial.Serial") as ctor:
        fake = FakeSerial()
        fake.queue(*lines)
        ctor.return_value = fake
        board = CrankCamBoard("/dev/fake")
    return board, fake


class StatusTests(unittest.TestCase):
    def test_parses_status_line(self):
        board, _ = make_board("STATUS profile=60-2 (Bosch/GM) rpm=3000 gen=1 capture=0")
        self.assertEqual(
            board.status(),
            Status(profile="60-2 (Bosch/GM)", rpm=3000, gen=True, capture=False),
        )

    def test_malformed_status_raises(self):
        board, _ = make_board("not a status line")
        with self.assertRaises(ProtocolError):
            board.status()

    def test_connect_is_status(self):
        board, _ = make_board("STATUS profile=60-2 (Bosch/GM) rpm=1000 gen=0 capture=0")
        self.assertEqual(board.connect().rpm, 1000)


class ProfileTests(unittest.TestCase):
    def test_lists_profiles(self):
        board, _ = make_board(
            "1) 60-2 (Bosch/GM)",
            "2) 36-1 (Ford/Mazda EDIS)",
            "7) 24 even tooth (no gap)",
            "OK",
        )
        self.assertEqual(
            board.list_profiles(),
            [
                Profile(1, "60-2 (Bosch/GM)"),
                Profile(2, "36-1 (Ford/Mazda EDIS)"),
                Profile(7, "24 even tooth (no gap)"),
            ],
        )

    def test_list_profiles_err_raises(self):
        board, _ = make_board("ERR unexpected")
        with self.assertRaises(ProtocolError):
            board.list_profiles()

    def test_help_joins_extra_lines(self):
        board, _ = make_board("p<n>   select profile", "r<n>   set RPM", "OK")
        self.assertEqual(board.help(), "p<n>   select profile\nr<n>   set RPM")


class CommandTests(unittest.TestCase):
    def test_select_profile_sends_command_and_accepts_ok(self):
        board, fake = make_board("OK profile=60-2 (Bosch/GM)")
        board.select_profile(1)
        self.assertEqual(fake.written[-1], b"p1\n")

    def test_select_profile_err_raises_protocol_error(self):
        board, _ = make_board("ERR generation running")
        with self.assertRaises(ProtocolError):
            board.select_profile(1)

    def test_set_rpm_sends_formatted_command(self):
        board, fake = make_board("OK rpm=3500")
        board.set_rpm(3500)
        self.assertEqual(fake.written[-1], b"r3500\n")

    def test_start_stop_gen_and_capture(self):
        board, fake = make_board("OK gen=1", "OK capture=1", "OK capture=0", "OK gen=0")
        board.start_gen()
        board.start_capture()
        board.stop_capture()
        board.stop_gen()
        self.assertEqual(fake.written, [b"g1\n", b"c1\n", b"c0\n", b"g0\n"])


class CycleBlockTests(unittest.TestCase):
    def test_parses_full_cycle_with_all_channel_kinds(self):
        # ch0: a real edge pair; ch1: an edge found but outside any known
        # window (the ch0 window-drift cosmetic condition, see CLAUDE.md);
        # ch2: no edge at all ("not detected"); ch3-5: not detected too.
        board, _ = make_board(
            "cycle 42:",
            "  ch0: rise=0.00deg fall=3.05deg",
            "  ch1: rise=-- fall=--",
            "  ch2: not detected",
            "  ch3: not detected",
            "  ch4: not detected",
            "  ch5: not detected",
        )
        report = board.read_cycle()
        self.assertEqual(report.cycle, 42)
        self.assertEqual(report.skipped, 0)
        self.assertTrue(report.have_refs)
        self.assertEqual(
            report.channels[0], ChannelReport(detected=True, rise_deg=0.00, fall_deg=3.05)
        )
        self.assertEqual(
            report.channels[1], ChannelReport(detected=True, rise_deg=None, fall_deg=None)
        )
        for ch in (2, 3, 4, 5):
            self.assertEqual(
                report.channels[ch], ChannelReport(detected=False, rise_deg=None, fall_deg=None)
            )

    def test_not_detected_and_no_window_are_distinguishable(self):
        # Regression test: these two conditions used to collapse to the
        # same (None, None) tuple in older client code, making a channel
        # with no signal indistinguishable from one whose edge just fell
        # outside a known window (a real signal). See git history and
        # CLAUDE.md's window-drift note.
        board, _ = make_board(
            "cycle 1:",
            "  ch0: not detected",
            "  ch1: rise=-- fall=--",
            "  ch2: not detected",
            "  ch3: not detected",
            "  ch4: not detected",
            "  ch5: not detected",
        )
        report = board.read_cycle()
        self.assertFalse(report.channels[0].detected)
        self.assertTrue(report.channels[1].detected)
        self.assertIsNone(report.channels[1].rise_deg)
        self.assertIsNone(report.channels[1].fall_deg)
        self.assertNotEqual(report.channels[0], report.channels[1])

    def test_partial_rise_only_edge(self):
        board, _ = make_board(
            "cycle 3:",
            "  ch0: rise=10.00deg fall=--",
            "  ch1: not detected",
            "  ch2: not detected",
            "  ch3: not detected",
            "  ch4: not detected",
            "  ch5: not detected",
        )
        report = board.read_cycle()
        ch0 = report.channels[0]
        self.assertTrue(ch0.detected)
        self.assertEqual(ch0.rise_deg, 10.00)
        self.assertIsNone(ch0.fall_deg)

    def test_no_valid_cycle_reference(self):
        board, _ = make_board("cycle 5:", "no valid cycle reference this pass")
        report = board.read_cycle()
        self.assertEqual(report, CycleReport(cycle=5, skipped=0, have_refs=False))

    def test_skipped_cycles_counted(self):
        board, _ = make_board(
            "cycle 9:",
            "  (3 earlier cycle(s) skipped -- printing fell behind; generation kept running)",
            "  ch0: rise=0.00deg fall=3.05deg",
            "  ch1: not detected",
            "  ch2: not detected",
            "  ch3: not detected",
            "  ch4: not detected",
            "  ch5: not detected",
        )
        report = board.read_cycle()
        self.assertEqual(report.skipped, 3)
        self.assertEqual(report.cycle, 9)

    def test_malformed_channel_line_raises(self):
        board, _ = make_board("cycle 1:", "  garbage line")
        with self.assertRaises(ProtocolError):
            board.read_cycle()

    def test_malformed_cycle_header_raises(self):
        board, _ = make_board("not a cycle header")
        with self.assertRaises(ProtocolError):
            board._read_cycle_block("not a cycle header")


class InterleavingTests(unittest.TestCase):
    def test_cycle_block_arriving_during_a_command_is_queued(self):
        # While capture is running, "cycle N:" blocks can land on the wire
        # at any time, including in the middle of an unrelated command's
        # response -- the client must transparently queue them rather
        # than choke on them mid-command.
        board, _ = make_board(
            "cycle 1:",
            "  ch0: rise=0.00deg fall=3.05deg",
            "  ch1: not detected",
            "  ch2: not detected",
            "  ch3: not detected",
            "  ch4: not detected",
            "  ch5: not detected",
            "STATUS profile=60-2 (Bosch/GM) rpm=1000 gen=1 capture=1",
        )
        status = board.status()
        self.assertTrue(status.capture)
        # the cycle block that arrived first should now be queued, and
        # read_cycle() drains the queue before touching the wire again
        report = board.read_cycle()
        self.assertEqual(report.cycle, 1)

    def test_multiple_queued_cycle_blocks_drain_in_order(self):
        board, _ = make_board(
            "cycle 1:",
            "  ch0: rise=0.00deg fall=3.05deg",
            "  ch1: not detected",
            "  ch2: not detected",
            "  ch3: not detected",
            "  ch4: not detected",
            "  ch5: not detected",
            "cycle 2:",
            "  ch0: rise=0.00deg fall=3.05deg",
            "  ch1: not detected",
            "  ch2: not detected",
            "  ch3: not detected",
            "  ch4: not detected",
            "  ch5: not detected",
            "OK capture=0",
        )
        board.stop_capture()
        self.assertEqual(board.read_cycle().cycle, 1)
        self.assertEqual(board.read_cycle().cycle, 2)


class TimeoutTests(unittest.TestCase):
    def test_readline_timeout_raises_protocol_error(self):
        board, _ = make_board()  # no lines queued at all
        with self.assertRaises(ProtocolError):
            board.status()

    def test_read_cycle_timeout_raises_protocol_error(self):
        board, _ = make_board()
        with self.assertRaises(ProtocolError):
            board.read_cycle()


if __name__ == "__main__":
    unittest.main()
