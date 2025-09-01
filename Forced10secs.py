#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Motor Logger GUI (Tkinter + pyX2Cscope) — f fixed to 450
--------------------------------------------------------
Source of truth (FORCED):
- Raw base sample time = 50 µs
- Sample-time factor f = 450 (not user-changeable)
- With f=450 and time=50 µs: total scope time ≈ 11025 ms
- Therefore effective sample period Ts = f * 50 µs = 22.5 ms (≈ 44.444 Hz)

What this app does:
- Connect to target (ELF + COM port list via pyserial)
- Enter speed (RPM) and scale (RPM per count), writes velocityReference once
- START: one-shot RUN, capture for 10 s, one-shot STOP (or early STOP button)
- Scope channels: idqCmd_q, Idq_q, Idq_d, OmegaElectrical, OmegaCmd
- Sampling is FIXED: set_sample_time(450). Shows: “You get one sample every 22.5 ms.”
- “Test Sampling” button calls get_scope_sample_time(50.0) to display real total ms
- Export button: choose where to save CSV (no auto-save)
- Optional matplotlib plots (Currents/Omega) if matplotlib is installed

One-shot semantics respected: RUN/STOP are written exactly once when pressed/needed.
"""

from __future__ import annotations

import csv
import sys
import threading
import time
import traceback
from pathlib import Path
from typing import Dict, List, Optional, Tuple

# Tkinter GUI
import tkinter as tk
from tkinter import filedialog, messagebox

# Optional matplotlib for plots
try:
    import matplotlib.pyplot as plt
    HAS_PLT = True
except Exception:
    plt = None
    HAS_PLT = False

# Serial only for listing ports
import serial.tools.list_ports  # type: ignore

# Primary dependency — import from package root to avoid circular import
try:
    from pyx2cscope import X2CScope  # type: ignore
except Exception:  # last-resort fallback if package doesn’t re-export
    import importlib
    _mod = importlib.import_module("pyx2cscope.x2cscope")
    X2CScope = getattr(_mod, "X2CScope")


# ----------------------------- Forced sampling model -----------------------------

RAW_SAMPLE_TIME_US = 50.0   # fixed base period per raw sample (µs)
FACTOR_F = 450              # fixed decimation factor
TOTAL_MS_AT_ANCHOR = 11025.0  # for f=450 and 50 µs raw time

# Derived effective sampling:
TS_S = (FACTOR_F * RAW_SAMPLE_TIME_US) / 1_000_000.0   # seconds per effective sample
TS_MS = TS_S * 1_000.0                                  # ms per effective sample  ⇒ 22.5 ms
FS_HZ = 1.0 / TS_S if TS_S > 0 else 0.0                 # ≈ 44.444 Hz

# Behavior
RUN_SECONDS = 10.0
BASELINE_SECONDS = 0.5
TAIL_SECONDS = 0.5
SLEEP_POLL = 0.05

# Monitor variables (label, ELF symbol)
MONITOR_VARS: List[Tuple[str, str]] = [
    ("idqCmd_q",        "motor.idqCmd.q"),
    ("Idq_q",           "motor.idq.q"),
    ("Idq_d",           "motor.idq.d"),
    ("OmegaElectrical", "motor.omegaElectrical"),
    ("OmegaCmd",        "motor.omegaCmd"),
]

# Control variables (ELF symbol)
CTRL_HW_UI      = "app.hardwareUiEnabled"
CTRL_VEL_REF    = "motor.apiData.velocityReference"  # counts
CTRL_RUN_REQ    = "motor.apiData.runMotorRequest"
CTRL_STOP_REQ   = "motor.apiData.stopMotorRequest"


# ----------------------------- Utilities -----------------------------

def list_ports() -> List[str]:
    return [p.device for p in serial.tools.list_ports.comports()]

def safe_float(s: str, default: float = 0.0) -> float:
    try:
        return float(s)
    except Exception:
        return default

def try_write(scope: X2CScope, handle, value) -> bool:
    """Write value to a variable handle using whichever API is available."""
    try:
        if hasattr(handle, "set_value"):
            handle.set_value(value)  # type: ignore
            return True
    except Exception:
        pass
    try:
        if hasattr(scope, "write"):
            scope.write(handle, value)  # type: ignore
            return True
    except Exception:
        pass
    return False

def try_clear_channels(scope: X2CScope) -> None:
    for name in ("clear_all_scope_channel", "clear_scope_channels", "clear_scope_channel"):
        if hasattr(scope, name):
            try:
                getattr(scope, name)()  # type: ignore
                return
            except Exception:
                pass

def get_total_ms_via_api(scope: X2CScope, time_us: float) -> float:
    """
    Use the requested API signature:
        get_scope_sample_time(self, time: float) -> float
    Returns total real duration of a dataset in milliseconds for the given raw time.
    """
    if hasattr(scope, "get_scope_sample_time"):
        try:
            return float(scope.get_scope_sample_time(time_us))  # type: ignore
        except TypeError:
            # some builds may have a zero-arg variant — try and trust if it looks like ms
            try:
                ret = scope.get_scope_sample_time()  # type: ignore
                if isinstance(ret, (int, float)) and ret > 1.0:
                    return float(ret)
            except Exception:
                pass
        except Exception:
            pass
    return 0.0


# ----------------------------- Capture Worker -----------------------------

class CaptureWorker(threading.Thread):
    """
    • Configures scope channels, sets sample factor f=450 (fixed)
    • request_scope_data() + polling during RUN window
    • One-shot RUN on start; one-shot STOP on end/early stop
    • Accumulates data; timestamps built from known Ts=22.5 ms (and cross-check via API total ms)
    • No auto-save — GUI Export saves CSV where you choose
    """
    def __init__(
        self,
        app_ref,                     # MotorLoggerApp
        x2c: X2CScope,
        handles: Dict[str, object],  # symbol -> handle
        variables: List[Tuple[str, str]],  # (label, symbol)
        counts: float,
    ):
        super().__init__(daemon=True)
        self.app = app_ref
        self.x2c = x2c
        self.handles = handles
        self.variables = variables
        self.counts = counts

        self._stop_flag = threading.Event()
        self._run_sent = False
        self._stop_sent = False

        # Results (filled on completion)
        self.data: Dict[str, List[float]] = {lbl: [] for (lbl, _sym) in variables}
        self.t_axis: List[float] = []
        self.ts_s: float = TS_S
        self.total_ms_reported: float = 0.0

    def stop_early(self):
        if not self._stop_sent:
            h = self.handles.get(CTRL_STOP_REQ)
            if h is not None:
                try_write(self.x2c, h, 1)
            self._stop_sent = True
        self._stop_flag.set()

    def _ui_status(self, text: str):
        self.app.safe_set_status(text)

    def run(self):
        try:
            # Configure scope channels
            try_clear_channels(self.x2c)
            for (_lbl, sym) in self.variables:
                h = self.handles.get(sym)
                if h is not None:
                    try:
                        self.x2c.add_scope_channel(h)  # type: ignore
                    except Exception:
                        pass

            # Set fixed decimation factor f = 450
            try:
                self.x2c.set_sample_time(FACTOR_F)  # type: ignore
            except Exception:
                pass

            # Program velocityReference once
            vref = self.handles.get(CTRL_VEL_REF)
            if vref is not None:
                try_write(self.x2c, vref, int(self.counts))

            # Arm acquisition
            try:
                self.x2c.request_scope_data()  # type: ignore
            except Exception:
                pass

            # Short baseline
            t0 = time.time()
            while time.time() - t0 < BASELINE_SECONDS and not self._stop_flag.is_set():
                time.sleep(SLEEP_POLL)

            # One-shot RUN
            if not self._stop_flag.is_set() and not self._run_sent:
                run_h = self.handles.get(CTRL_RUN_REQ)
                if run_h is not None:
                    try_write(self.x2c, run_h, 1)
                self._run_sent = True
                self._ui_status(f"RUN sent. Capturing… (f=450 ⇒ Ts=22.5 ms, Fs≈{FS_HZ:.3f} Hz)")

            # Capture during run window
            start = time.time()
            while (time.time() - start) < RUN_SECONDS and not self._stop_flag.is_set():
                if hasattr(self.x2c, "is_scope_data_ready") and self.x2c.is_scope_data_ready():  # type: ignore
                    # Read chunk
                    chunk = {}
                    try:
                        chunk = self.x2c.get_scope_channel_data(valid_data=False)  # type: ignore
                    except Exception:
                        chunk = {}

                    # Align by channel insertion order
                    seq = list(chunk.values()) if isinstance(chunk, dict) else []
                    if seq:
                        m = min((len(v) for v in seq if isinstance(v, (list, tuple))), default=0)
                        if m > 0:
                            for i, (lbl, _sym) in enumerate(self.variables):
                                if i < len(seq):
                                    vals = list(seq[i])[:m]
                                    self.data[lbl].extend(vals)

                    # Re-arm
                    try:
                        self.x2c.request_scope_data()  # type: ignore
                    except Exception:
                        pass

                time.sleep(SLEEP_POLL)

            # One-shot STOP
            if not self._stop_sent:
                stop_h = self.handles.get(CTRL_STOP_REQ)
                if stop_h is not None:
                    try_write(self.x2c, stop_h, 1)
                self._stop_sent = True
                self._ui_status("STOP sent. Finalizing…")

            # Short tail + final read
            t1 = time.time()
            while time.time() - t1 < TAIL_SECONDS and not self._stop_flag.is_set():
                time.sleep(SLEEP_POLL)
            try:
                if hasattr(self.x2c, "is_scope_data_ready") and self.x2c.is_scope_data_ready():  # type: ignore
                    chunk = self.x2c.get_scope_channel_data(valid_data=False)  # type: ignore
                    seq = list(chunk.values()) if isinstance(chunk, dict) else []
                    if seq:
                        m = min((len(v) for v in seq if isinstance(v, (list, tuple))), default=0)
                        if m > 0:
                            for i, (lbl, _sym) in enumerate(self.variables):
                                if i < len(seq):
                                    vals = list(seq[i])[:m]
                                    self.data[lbl].extend(vals)
            except Exception:
                pass

            # Build time axis with FIXED Ts = 22.5 ms (forced model)
            n_min = min((len(v) for v in self.data.values()), default=0)
            for k in list(self.data.keys()):
                self.data[k] = self.data[k][:n_min]
            self.t_axis = [i * TS_S for i in range(n_min)]

            # Cross-check total window from device via API using time=50 µs
            self.total_ms_reported = get_total_ms_via_api(self.x2c, RAW_SAMPLE_TIME_US)
            if self.total_ms_reported <= 0:
                self.total_ms_reported = TOTAL_MS_AT_ANCHOR  # fallback to anchor when API not available

            self._ui_status(
                f"Done. Samples/ch: {n_min}, Ts={TS_MS:.3f} ms (Fs≈{FS_HZ:.3f} Hz). "
                f"Device-reported total≈{self.total_ms_reported:.2f} ms."
            )
            self.app.on_capture_complete(self.data, self.t_axis, TS_S, self.total_ms_reported)
        except Exception as e:
            self._ui_status("Error during capture.")
            self.app.safe_show_error("Capture error", f"{e}\n\n{traceback.format_exc()}")
        finally:
            # Safety: if RUN was sent but STOP wasn't, try one-shot STOP
            if self._run_sent and not self._stop_sent:
                try:
                    h = self.handles.get(CTRL_STOP_REQ)
                    if h is not None:
                        try_write(self.x2c, h, 1)
                except Exception:
                    pass


# ----------------------------- Main App -----------------------------

class MotorLoggerApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Motor Logger (Tk + pyX2Cscope) — f=450 ⇒ Ts=22.5 ms")
        self.geometry("900x580")

        # Session state
        self.x2c: Optional[X2CScope] = None
        self.connected = False
        self.elf_path: Optional[Path] = None
        self.port: Optional[str] = None
        self.baud: int = 115200
        self.handles: Dict[str, object] = {}

        # Last capture
        self.last_data: Optional[Dict[str, List[float]]] = None
        self.last_t: Optional[List[float]] = None
        self.last_total_ms: float = 0.0

        self.worker: Optional[CaptureWorker] = None

        self._build_ui()
        self._refresh_ports()

    # ---------- UI ----------

    def _build_ui(self):
        pad = {"padx": 8, "pady": 6}

        # Row 0: ELF + Port + Baud
        row0 = tk.Frame(self)
        row0.pack(fill="x", **pad)

        tk.Label(row0, text="ELF:").pack(side="left")
        self.lbl_elf = tk.Label(row0, text="(none)", width=44, anchor="w")
        self.lbl_elf.pack(side="left", padx=6)
        tk.Button(row0, text="Browse…", command=self.on_browse_elf).pack(side="left", padx=4)

        tk.Label(row0, text="Port:").pack(side="left", padx=(18, 0))
        self.cbo_port = tk.StringVar(self)
        self.ddl_port = tk.OptionMenu(row0, self.cbo_port, "")
        self.ddl_port.config(width=16)
        self.ddl_port.pack(side="left")
        tk.Button(row0, text="Refresh", command=self._refresh_ports).pack(side="left", padx=4)

        tk.Label(row0, text="Baud:").pack(side="left", padx=(18, 0))
        self.cbo_baud = tk.StringVar(self, "115200")
        self.ddl_baud = tk.OptionMenu(row0, self.cbo_baud, "115200", "230400", "460800", "921600")
        self.ddl_baud.config(width=10)
        self.ddl_baud.pack(side="left")

        # Row 1: Speed/Scale (NO f entry; f is fixed to 450)
        row1 = tk.Frame(self)
        row1.pack(fill="x", **pad)

        tk.Label(row1, text="Speed (RPM):").pack(side="left")
        self.ent_speed = tk.Entry(row1, width=10)
        self.ent_speed.insert(0, "1000")
        self.ent_speed.pack(side="left", padx=6)

        tk.Label(row1, text="Scale (RPM/count):").pack(side="left", padx=(18, 0))
        self.ent_scale = tk.Entry(row1, width=10)
        self.ent_scale.insert(0, "1.0")
        self.ent_scale.pack(side="left", padx=6)

        # Row 2: Fixed sampling info + Test button
        row2 = tk.Frame(self)
        row2.pack(fill="x", **pad)

        fixed_info = (
            f"Sampling FIXED: f=450, base=50 µs ⇒ Ts = {TS_MS:.3f} ms per sample (Fs ≈ {FS_HZ:.3f} Hz).  "
            f"Anchor total≈{TOTAL_MS_AT_ANCHOR:.0f} ms for device dataset."
        )
        self.lbl_fixed = tk.Label(row2, text=fixed_info, anchor="w")
        self.lbl_fixed.pack(side="left", fill="x", expand=True)

        self.btn_test = tk.Button(row2, text="Test Sampling (get_scope_sample_time 50µs)", command=self.on_test_sampling)
        self.btn_test.pack(side="right")

        # Row 3: Buttons
        row3 = tk.Frame(self)
        row3.pack(fill="x", **pad)

        self.btn_connect = tk.Button(row3, text="Connect", width=12, command=self.on_connect_toggle)
        self.btn_connect.pack(side="left")

        self.btn_start = tk.Button(row3, text="START ▶ (10 s)", width=16, command=self.on_start)
        self.btn_start.pack(side="left", padx=6)

        self.btn_stop = tk.Button(row3, text="STOP ■", width=10, command=self.on_stop)
        self.btn_stop.pack(side="left", padx=6)

        self.btn_export = tk.Button(row3, text="Export…", width=12, command=self.on_export)
        self.btn_export.pack(side="left", padx=(18, 6))

        self.btn_plot_curr = tk.Button(row3, text="Plot Currents", width=14, command=lambda: self.show_plot("currents"))
        self.btn_plot_curr.pack(side="left", padx=6)
        self.btn_plot_omega = tk.Button(row3, text="Plot Omega", width=14, command=lambda: self.show_plot("omega"))
        self.btn_plot_omega.pack(side="left", padx=6)

        # Row 4: Status/output
        row4 = tk.Frame(self)
        row4.pack(fill="both", expand=True, **pad)
        self.txt_status = tk.Text(row4, height=14)
        self.txt_status.pack(fill="both", expand=True)
        self.safe_set_status("Ready. Sampling is fixed: one sample every 22.5 ms. Use 'Test Sampling' to query device total window.")

        # Disable plot buttons if matplotlib missing
        if not HAS_PLT:
            self.btn_plot_curr.config(state="disabled")
            self.btn_plot_omega.config(state="disabled")

        # Close protocol
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    # ---------- Event Handlers ----------

    def on_browse_elf(self):
        path = filedialog.askopenfilename(title="Select ELF", filetypes=[("ELF files", "*.elf"), ("All files", "*.*")])
        if not path:
            return
        self.elf_path = Path(path)
        self.lbl_elf.config(text=str(self.elf_path.name))

    def _refresh_ports(self):
        ports = list_ports()
        menu = self.ddl_port["menu"]
        menu.delete(0, "end")
        if not ports:
            ports = [""]
        for p in ports:
            menu.add_command(label=p, command=lambda v=p: self.cbo_port.set(v))
        self.cbo_port.set(ports[0])

    def on_connect_toggle(self):
        if not self.connected:
            self.connect()
        else:
            self.disconnect()

    def connect(self):
        try:
            if not self.elf_path:
                messagebox.showwarning("Missing ELF", "Please select an ELF file first.")
                return
            port = self.cbo_port.get().strip()
            if not port:
                messagebox.showwarning("No Port", "Select a serial port.")
                return
            self.port = port
            self.baud = int(self.cbo_baud.get())

            # Create scope and import variables
            self.x2c = X2CScope(port=self.port, baudrate=self.baud, elf_file=str(self.elf_path))  # many builds accept this
            try:
                self.x2c.import_variables(str(self.elf_path))  # type: ignore (safe if already loaded)
            except Exception:
                pass

            self._resolve_handles()

            # Set app.hardwareUiEnabled = 0 once (best effort)
            h = self.handles.get(CTRL_HW_UI)
            if h is not None:
                try_write(self.x2c, h, 0)

            # Ensure device sampling factor is set to the forced value now
            try:
                self.x2c.set_sample_time(FACTOR_F)  # type: ignore
            except Exception:
                pass

            self.connected = True
            self.btn_connect.config(text="Disconnect")
            self.safe_set_status(f"Connected on {self.port} @ {self.baud} baud. Sampling fixed: Ts={TS_MS:.3f} ms.")
        except Exception as e:
            self.connected = False
            self.x2c = None
            self.safe_show_error("Connection error", f"{e}\n\n{traceback.format_exc()}")

    def disconnect(self):
        # Do NOT write RUN/STOP here (one-shot semantics only on user actions)
        try:
            if self.x2c and hasattr(self.x2c, "close"):
                self.x2c.close()  # type: ignore
        except Exception:
            pass
        self.x2c = None
        self.connected = False
        self.btn_connect.config(text="Connect")
        self.safe_set_status("Disconnected.")

    def _resolve_handles(self):
        assert self.x2c is not None
        self.handles.clear()
        # Monitor
        for _lbl, sym in MONITOR_VARS:
            try:
                self.handles[sym] = self.x2c.get_variable(sym)  # type: ignore
            except Exception:
                self.handles[sym] = None
        # Control
        for sym in (CTRL_HW_UI, CTRL_VEL_REF, CTRL_RUN_REQ, CTRL_STOP_REQ):
            try:
                self.handles[sym] = self.x2c.get_variable(sym)  # type: ignore
            except Exception:
                self.handles[sym] = None

    def on_test_sampling(self):
        """Show device-reported total ms using get_scope_sample_time(50 µs) with f=450 already set."""
        if not self.connected or not self.x2c:
            self.safe_set_status("Not connected. Connect first to test sampling.")
            return

        # Reinforce f=450 just before test
        try:
            self.x2c.set_sample_time(FACTOR_F)  # type: ignore
        except Exception:
            pass

        total_ms = get_total_ms_via_api(self.x2c, RAW_SAMPLE_TIME_US)
        origin = "device API"
        if total_ms <= 0.0:
            total_ms = TOTAL_MS_AT_ANCHOR  # fall back to anchor
            origin = "anchor fallback"

        msg = (
            f"[Sampling Test] f=450, raw=50 µs → device total≈{total_ms:.2f} ms ({origin}). "
            f"Effective Ts is FIXED at {TS_MS:.3f} ms per sample (Fs≈{FS_HZ:.3f} Hz)."
        )
        self.safe_set_status(msg)

    def on_start(self):
        if self.worker and self.worker.is_alive():
            messagebox.showinfo("Busy", "Capture already running.")
            return

        # Connect on demand
        if not self.connected:
            self.connect()
            if not self.connected:
                return

        # Parameters
        speed_rpm = safe_float(self.ent_speed.get(), 0.0)
        scale_rpm_per_count = safe_float(self.ent_scale.get(), 0.0)
        if scale_rpm_per_count == 0.0:
            messagebox.showwarning("Invalid Scale", "Scale (RPM per count) must be non-zero.")
            return
        counts = speed_rpm / scale_rpm_per_count

        # Warn if some variables are missing
        missing = [sym for (_lbl, sym) in MONITOR_VARS if self.handles.get(sym) is None]
        if missing:
            if not messagebox.askyesno(
                "Missing variables",
                "Some scope variables could not be resolved:\n"
                + "\n".join(missing)
                + "\n\nStart capture anyway?"
            ):
                return

        # Start worker (no auto-save)
        self.worker = CaptureWorker(
            app_ref=self,
            x2c=self.x2c,  # type: ignore
            handles=self.handles,
            variables=MONITOR_VARS,
            counts=counts,
        )
        self.worker.start()
        self.safe_set_status("Preparing capture… (RUN will be sent once; sampling fixed: one sample every 22.5 ms)")

    def on_stop(self):
        # Early stop: one-shot STOP + finish loop
        if self.worker and self.worker.is_alive():
            self.worker.stop_early()
            self.safe_set_status("Stop requested (one-shot).")
        else:
            # Best-effort one-shot STOP even if no worker
            if self.connected and self.x2c:
                h = self.handles.get(CTRL_STOP_REQ)
                if h is not None:
                    try_write(self.x2c, h, 1)
                    self.safe_set_status("STOP sent.")

    def on_export(self):
        """Export the most recent capture to a user-selected CSV path."""
        if not self.last_data or not self.last_t:
            messagebox.showinfo("No data", "Run a capture first.")
            return
        path = filedialog.asksaveasfilename(
            title="Save CSV",
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
        )
        if not path:
            return
        try:
            self._save_csv(Path(path), self.last_t, self.last_data)
            self.safe_set_status(f"Exported: {Path(path).resolve()}")
        except Exception as e:
            self.safe_show_error("Export error", f"{e}\n\n{traceback.format_exc()}")

    def _save_csv(self, csv_path: Path, t_axis: List[float], data: Dict[str, List[float]]):
        # Align rows to the shortest column length
        n_min = min([len(t_axis)] + [len(v) for v in data.values()]) if data else 0
        headers = ["t_s"] + [lbl for (lbl, _s) in MONITOR_VARS]
        with csv_path.open("w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(headers)
            for i in range(n_min):
                row = [t_axis[i]] + [data[lbl][i] for (lbl, _s) in MONITOR_VARS]
                w.writerow(row)

    def on_capture_complete(self, data: Dict[str, List[float]], t_axis: List[float], ts_s: float, total_ms: float):
        self.last_data = data
        self.last_t = t_axis
        self.last_total_ms = total_ms
        fs = (1.0 / ts_s) if ts_s > 0 else 0.0
        self.safe_set_status(
            f"Capture complete. Ts={TS_MS:.3f} ms per sample (Fs≈{fs:.3f} Hz). "
            f"Device total≈{total_ms:.2f} ms. Use 'Export…' to save CSV."
        )
        # Enable plot buttons if matplotlib available & data present
        has_any = t_axis and any(len(v) for v in data.values())
        if HAS_PLT and has_any:
            self.btn_plot_curr.config(state="normal")
            self.btn_plot_omega.config(state="normal")

    # ---------- Plotting ----------

    def show_plot(self, group: str):
        if not HAS_PLT:
            self.safe_show_error("Plotting unavailable", "matplotlib is not installed.")
            return
        if not self.last_data or not self.last_t:
            messagebox.showinfo("No data", "Run a capture first.")
            return

        t = self.last_t
        if group == "currents":
            keys = ["idqCmd_q", "Idq_q", "Idq_d"]
            title = "Currents"
        else:
            keys = ["OmegaElectrical", "OmegaCmd"]
            title = "Omega"

        fig, ax = plt.subplots()
        for k in keys:
            if k in self.last_data:
                ax.plot(t, self.last_data[k], label=k)
        ax.set_title(title)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Value (raw)")
        ax.grid(True)
        ax.legend()
        fig.tight_layout()
        plt.show()

    # ---------- Helpers ----------

    def safe_set_status(self, text: str):
        def _do():
            self.txt_status.insert("end", text + "\n")
            self.txt_status.see("end")
        self.after(0, _do)

    def safe_show_error(self, title: str, message: str):
        self.after(0, lambda: messagebox.showerror(title, message))

    def on_close(self):
        try:
            if self.worker and self.worker.is_alive():
                self.worker.stop_early()
                time.sleep(0.2)  # give it a moment to send STOP
        except Exception:
            pass
        try:
            self.disconnect()
        except Exception:
            pass
        self.destroy()


# ----------------------------- Entry Point -----------------------------

def main():
    app = MotorLoggerApp()
    app.mainloop()


if __name__ == "__main__":
    main()
