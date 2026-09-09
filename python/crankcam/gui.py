"""Small Tk GUI for the crank/cam testbench: connect to a port, drive the
engine (profile/RPM/gen/capture), and watch the live per-cycle capture
report as a 0-720deg timeline.

All serial I/O happens on one dedicated worker thread, which owns the
CrankCamBoard instance exclusively -- the GUI thread never touches it
directly, only a pair of queues (commands in, events out), since
CrankCamBoard/pyserial aren't safe to call from two threads at once (a
command's response line and a capture-report line could otherwise race
on the same read). Run with: python -m crankcam.gui
"""

from __future__ import annotations

import queue
import threading
import tkinter as tk
from tkinter import ttk
from typing import Optional

import serial

from .board import CAPTURE_PIN_COUNT, CrankCamBoard, CycleReport, Profile, ProtocolError, Status

CHANNEL_LABELS = ["ch0 (crank)", "ch1 (cam)", "ch2", "ch3", "ch4", "ch5"]


class Worker(threading.Thread):
    """Owns the CrankCamBoard and all serial I/O. Commands come in via
    cmd_q as (name, args) tuples; results/errors/cycle reports go out via
    event_q as (kind, payload) tuples."""

    def __init__(self, cmd_q: "queue.Queue", event_q: "queue.Queue"):
        super().__init__(daemon=True)
        self.cmd_q = cmd_q
        self.event_q = event_q
        self._stop = threading.Event()
        self.board: Optional[CrankCamBoard] = None
        self._capturing = False

    def stop(self):
        self._stop.set()

    def run(self):
        while not self._stop.is_set():
            try:
                name, args = self.cmd_q.get(timeout=0.05)
                self._handle(name, args)
            except queue.Empty:
                pass

            if self.board is not None and self._capturing:
                try:
                    report = self.board.read_cycle()
                    self.event_q.put(("cycle", report))
                except ProtocolError:
                    pass  # just a read timeout while polling; try again

        if self.board is not None:
            self.board.close()

    def _handle(self, name: str, args: tuple):
        try:
            if name == "connect":
                (port,) = args
                board = CrankCamBoard(port, timeout=0.2)
                status = board.connect()
                self.board = board
                self._capturing = status.capture
                self.event_q.put(("connected", status))
                self.event_q.put(("profiles", board.list_profiles()))
                return

            if name == "disconnect":
                if self.board is not None:
                    self.board.close()
                    self.board = None
                self._capturing = False
                self.event_q.put(("disconnected", None))
                return

            if self.board is None:
                self.event_q.put(("error", "not connected"))
                return

            if name == "select_profile":
                self.board.select_profile(args[0])
            elif name == "set_rpm":
                self.board.set_rpm(args[0])
            elif name == "start_gen":
                self.board.start_gen()
            elif name == "stop_gen":
                self.board.stop_gen()
                self._capturing = False
            elif name == "start_capture":
                self.board.start_capture()
                self._capturing = True
            elif name == "stop_capture":
                self.board.stop_capture()
                self._capturing = False
            else:
                self.event_q.put(("error", f"unknown command {name!r}"))
                return

            self.event_q.put(("status", self.board.status()))

        except (ProtocolError, serial.SerialException) as e:
            self.event_q.put(("error", str(e)))


class CrankCamGui:
    def __init__(self, root: tk.Tk):
        self.root = root
        root.title("Crank/Cam Testbench")

        self.cmd_q: "queue.Queue" = queue.Queue()
        self.event_q: "queue.Queue" = queue.Queue()
        self.worker = Worker(self.cmd_q, self.event_q)
        self.worker.start()

        self.profiles: list[Profile] = []
        self.connected = False

        self._build_widgets()
        self.root.after(50, self._poll_events)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # -- widgets --

    def _build_widgets(self):
        conn = ttk.Frame(self.root, padding=6)
        conn.pack(fill="x")
        ttk.Label(conn, text="Port:").pack(side="left")
        self.port_var = tk.StringVar(value="/dev/ttyACM0")
        ttk.Entry(conn, textvariable=self.port_var, width=16).pack(side="left", padx=4)
        self.connect_btn = ttk.Button(conn, text="Connect", command=self._on_connect_click)
        self.connect_btn.pack(side="left", padx=4)
        self.status_var = tk.StringVar(value="not connected")
        ttk.Label(conn, textvariable=self.status_var).pack(side="left", padx=12)

        ctrl = ttk.Frame(self.root, padding=6)
        ctrl.pack(fill="x")

        ttk.Label(ctrl, text="Profile:").grid(row=0, column=0, sticky="w")
        self.profile_var = tk.StringVar()
        self.profile_box = ttk.Combobox(ctrl, textvariable=self.profile_var, state="readonly", width=28)
        self.profile_box.grid(row=0, column=1, padx=4)
        ttk.Button(ctrl, text="Select", command=self._on_select_profile).grid(row=0, column=2)

        ttk.Label(ctrl, text="RPM:").grid(row=0, column=3, padx=(16, 0), sticky="w")
        self.rpm_var = tk.StringVar(value="1000")
        ttk.Entry(ctrl, textvariable=self.rpm_var, width=8).grid(row=0, column=4, padx=4)
        ttk.Button(ctrl, text="Set", command=self._on_set_rpm).grid(row=0, column=5)

        self.gen_btn = ttk.Button(ctrl, text="Start gen", command=self._on_toggle_gen)
        self.gen_btn.grid(row=1, column=0, columnspan=2, pady=6, sticky="w")
        self.capture_btn = ttk.Button(ctrl, text="Start capture", command=self._on_toggle_capture)
        self.capture_btn.grid(row=1, column=2, columnspan=2, pady=6, sticky="w")

        self._set_controls_enabled(False)

        # -- capture visualization: one row per channel, angle 0-720deg --
        self.canvas_w = 760
        self.canvas_h = 24 * CAPTURE_PIN_COUNT + 40
        self.canvas = tk.Canvas(self.root, width=self.canvas_w, height=self.canvas_h,
                                 background="white", highlightthickness=1, highlightbackground="grey")
        self.canvas.pack(padx=6, pady=6)
        self.cycle_var = tk.StringVar(value="no cycles yet")
        ttk.Label(self.root, textvariable=self.cycle_var).pack(anchor="w", padx=8)

        # -- log --
        log_frame = ttk.Frame(self.root, padding=(6, 0, 6, 6))
        log_frame.pack(fill="both", expand=True)
        self.log = tk.Text(log_frame, height=8, state="disabled", wrap="none")
        self.log.pack(fill="both", expand=True, side="left")
        scroll = ttk.Scrollbar(log_frame, command=self.log.yview)
        scroll.pack(fill="y", side="right")
        self.log.configure(yscrollcommand=scroll.set)

        self._draw_axis()

    def _set_controls_enabled(self, enabled: bool):
        state = "normal" if enabled else "disabled"
        self.profile_box.configure(state="readonly" if enabled else "disabled")
        self.gen_btn.configure(state=state)
        self.capture_btn.configure(state=state)

    def _log(self, msg: str):
        self.log.configure(state="normal")
        self.log.insert("end", msg + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    # -- button handlers: just enqueue a command, never touch the board --

    def _on_connect_click(self):
        if self.connected:
            self.cmd_q.put(("disconnect", ()))
            self.connect_btn.configure(text="Connect")
        else:
            self.cmd_q.put(("connect", (self.port_var.get(),)))

    def _on_select_profile(self):
        idx = self.profile_box.current()
        if idx >= 0:
            self.cmd_q.put(("select_profile", (self.profiles[idx].index,)))

    def _on_set_rpm(self):
        try:
            rpm = int(self.rpm_var.get())
        except ValueError:
            self._log("invalid RPM (not a number)")
            return
        self.cmd_q.put(("set_rpm", (rpm,)))

    def _on_toggle_gen(self):
        if self.gen_btn["text"] == "Start gen":
            self.cmd_q.put(("start_gen", ()))
        else:
            self.cmd_q.put(("stop_gen", ()))

    def _on_toggle_capture(self):
        if self.capture_btn["text"] == "Start capture":
            self.cmd_q.put(("start_capture", ()))
        else:
            self.cmd_q.put(("stop_capture", ()))

    def _on_close(self):
        self.worker.stop()
        self.root.after(100, self.root.destroy)

    # -- event pump: only the GUI thread ever touches widgets --

    def _poll_events(self):
        try:
            while True:
                kind, payload = self.event_q.get_nowait()
                self._handle_event(kind, payload)
        except queue.Empty:
            pass
        self.root.after(50, self._poll_events)

    def _handle_event(self, kind: str, payload):
        if kind == "connected":
            self.connected = True
            self.connect_btn.configure(text="Disconnect")
            self._set_controls_enabled(True)
            self._apply_status(payload)
            self._log(f"connected: {payload}")
        elif kind == "disconnected":
            self.connected = False
            self.connect_btn.configure(text="Connect")
            self._set_controls_enabled(False)
            self.status_var.set("not connected")
            self._log("disconnected")
        elif kind == "profiles":
            self.profiles = payload
            self.profile_box["values"] = [f"{p.index}) {p.name}" for p in payload]
        elif kind == "status":
            self._apply_status(payload)
        elif kind == "cycle":
            self._draw_cycle(payload)
        elif kind == "error":
            self._log(f"ERROR: {payload}")

    def _apply_status(self, status: Status):
        self.status_var.set(
            f"profile={status.profile}  rpm={status.rpm}  gen={'on' if status.gen else 'off'}  "
            f"capture={'on' if status.capture else 'off'}"
        )
        self.gen_btn.configure(text="Stop gen" if status.gen else "Start gen")
        self.capture_btn.configure(text="Stop capture" if status.capture else "Start capture")
        if status.rpm:
            self.rpm_var.set(str(status.rpm))
        for i, p in enumerate(self.profiles):
            if p.name == status.profile:
                self.profile_box.current(i)
                break

    # -- visualization --

    def _draw_axis(self):
        self.canvas.delete("axis")
        top = 10
        row_h = 24
        for ch in range(CAPTURE_PIN_COUNT):
            y = top + ch * row_h + row_h // 2
            self.canvas.create_line(60, y, self.canvas_w - 10, y, fill="#ccc", tags="axis")
            self.canvas.create_text(6, y, text=CHANNEL_LABELS[ch], anchor="w", font=("TkDefaultFont", 8),
                                     tags="axis")
        base_y = top + CAPTURE_PIN_COUNT * row_h + 12
        for deg in (0, 180, 360, 540, 720):
            x = 60 + deg / 720 * (self.canvas_w - 70)
            self.canvas.create_line(x, top - 4, x, base_y, fill="#eee", tags="axis")
            self.canvas.create_text(x, base_y, text=f"{deg}°", anchor="n", font=("TkDefaultFont", 8),
                                     tags="axis")

    def _draw_cycle(self, report: CycleReport):
        skipped = f" ({report.skipped} skipped)" if report.skipped else ""
        self.cycle_var.set(f"cycle {report.cycle}{skipped} -- {'ok' if report.have_refs else 'no reference'}")

        self.canvas.delete("data")
        top = 10
        row_h = 24
        x0 = 60
        x1 = self.canvas_w - 10

        def x_for(deg: float) -> float:
            deg = max(0.0, min(720.0, deg))
            return x0 + deg / 720 * (x1 - x0)

        for ch in range(CAPTURE_PIN_COUNT):
            y = top + ch * row_h + row_h // 2
            rise, fall = report.channels.get(ch, (None, None)) if report.have_refs else (None, None)
            if rise is None and fall is None:
                self.canvas.create_text((x0 + x1) / 2, y, text="not detected", fill="#aaa",
                                         font=("TkDefaultFont", 8), tags="data")
                continue
            if rise is not None:
                x = x_for(rise)
                self.canvas.create_line(x, y - 7, x, y + 7, fill="#2a7", width=2, tags="data")
                self.canvas.create_text(x, y - 10, text=f"{rise:.1f}°", fill="#2a7",
                                         font=("TkDefaultFont", 7), anchor="s", tags="data")
            if fall is not None:
                x = x_for(fall)
                self.canvas.create_line(x, y - 7, x, y + 7, fill="#c33", width=2, tags="data")
                self.canvas.create_text(x, y + 10, text=f"{fall:.1f}°", fill="#c33",
                                         font=("TkDefaultFont", 7), anchor="n", tags="data")
            if rise is not None and fall is not None:
                self.canvas.create_line(x_for(rise), y, x_for(fall), y, fill="#2a7", width=3, tags="data")


def main():
    root = tk.Tk()
    CrankCamGui(root)
    root.mainloop()


if __name__ == "__main__":
    main()
