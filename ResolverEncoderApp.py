#!/usr/bin/env python3
"""Resolver to Encoder Test GUI.

This simple application is derived from the MotorLogger GUI. It connects to a
pyX2Cscope target and logs resolver related variables in order to help with
functionality checks and calibration.
"""

from __future__ import annotations

import pathlib
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from typing import Dict, List

import serial.tools.list_ports

try:
    from pyx2cscope.x2cscope import X2CScope
except ImportError:
    X2CScope = None  # type: ignore

USE_SCOPE = True

# Resolver related variable paths ------------------------------------------------
VAR_PATHS = {
    "rawAngle":       "resolver.rawAngle",       # raw electrical angle
    "convertedAngle": "resolver.convertedAngle", # angle after processing
    "offset":         "resolver.offset",         # calibration offset
    "status":         "resolver.status",         # status bits/flags
}
PATH_TO_KEY = {v: k for k, v in VAR_PATHS.items()}

# Variables that hold calibration results on the MCU.  These are typically
# defined as globals in ``main.c`` and can be read back over X2Cscope.
# Adjust the mapping here if your firmware uses different symbols.
CAL_VARS = {
    "offset": "resolver.offset",
    "status": "resolver.status",
}

# Control paths (example names)
RUN_REQ_VAR = "resolver.run"
CAL_REQ_VAR = "resolver.calibrate"

class _ScopeWrapper:
    def __init__(self):
        self._scope: X2CScope | None = None

    def connect(self, port: str, elf: str):
        if USE_SCOPE and X2CScope:
            self._scope = X2CScope(port=port)
            self._scope.import_variables(elf)

    def get_variable(self, path: str):
        if not USE_SCOPE or not X2CScope:
            return _DummyVar(path)
        if self._scope is None:
            raise RuntimeError("Scope not connected")
        return self._scope.get_variable(path)

    def disconnect(self):
        if USE_SCOPE and self._scope:
            self._scope.disconnect()
            self._scope = None

    def prepare_scope(self, vars: List[object], sample_ms: int):
        if not USE_SCOPE or not self._scope:
            return
        if hasattr(self._scope, "clear_scope_channels"):
            self._scope.clear_scope_channels()
        for var in vars:
            self._scope.add_scope_channel(var)
        # Convert desired interval (ms) to prescaler using 50 µs base period
        base_us = 50.0
        desired_us = max(int(sample_ms), 1) * 1000
        prescaler = max(int(round(desired_us / base_us)) - 1, 0)
        self._scope.set_sample_time(prescaler)
        self._scope.request_scope_data()

    def scope_ready(self) -> bool:
        return bool(USE_SCOPE and self._scope and self._scope.is_scope_data_ready())

    def get_scope_data(self):
        if not USE_SCOPE or not self._scope:
            return {}
        return self._scope.get_scope_channel_data(valid_data=True)

    def request_scope_data(self):
        if USE_SCOPE and self._scope:
            self._scope.request_scope_data()


class _DummyVar:
    def __init__(self, name: str):
        self.name = name
        self._val = 0
    def set_value(self, v):
        self._val = v
    def get_value(self):
        return self._val


class ResolverEncoderGUI:
    GUI_POLL_MS = 500
    DEFAULT_DT = 5

    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Resolver Encoder Test")

        self.scope = _ScopeWrapper()
        self.connected = False
        self._cap_thread: threading.Thread | None = None
        self._stop_flag = threading.Event()
        self.data: Dict[str, List[float]] = {}

        self._build_widgets()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._poll_job: str | None = None
        self._poll_gui()

    # GUI ---------------------------------------------------------------------
    def _build_widgets(self):
        main = ttk.Frame(self.root)
        main.pack(fill="both", expand=True)

        conn = ttk.LabelFrame(main, text="Connection", padding=8)
        conn.pack(fill="x", padx=10, pady=6)

        ttk.Label(conn, text="ELF file:").grid(row=0, column=0, sticky="e")
        self.elf_path = tk.StringVar()
        ttk.Entry(conn, textvariable=self.elf_path, width=42).grid(row=0, column=1, sticky="we", padx=4)
        ttk.Button(conn, text="Browse…", command=self._browse_elf).grid(row=0, column=2, padx=4)

        ttk.Label(conn, text="COM port:").grid(row=1, column=0, sticky="e", pady=4)
        self.port_var = tk.StringVar()
        self.port_menu = ttk.OptionMenu(conn, self.port_var, "-", *self._ports())
        self.port_menu.grid(row=1, column=1, sticky="we", padx=4)
        ttk.Button(conn, text="↻", width=3, command=self._refresh_ports).grid(row=1, column=2, padx=4)

        self.conn_btn = ttk.Button(conn, text="Connect", command=self._toggle_conn)
        self.conn_btn.grid(row=2, column=0, columnspan=3, pady=(6, 2))

        parms = ttk.LabelFrame(main, text="Parameters", padding=8)
        parms.pack(fill="x", padx=10, pady=4)

        ttk.Label(parms, text="Log time (s):").grid(row=0, column=0, sticky="e")
        self.dur_entry = ttk.Entry(parms, width=10)
        self.dur_entry.insert(0, "5")
        self.dur_entry.grid(row=0, column=1, padx=6, pady=2)

        ttk.Label(parms, text="Sample every (ms):").grid(row=1, column=0, sticky="e")
        self.sample_entry = ttk.Entry(parms, width=10)
        self.sample_entry.insert(0, str(self.DEFAULT_DT))
        self.sample_entry.grid(row=1, column=1, padx=6, pady=2)
        ttk.Label(parms, text="≥1 ms total").grid(row=1, column=2, sticky="w")

        btn = ttk.Frame(main)
        btn.pack(pady=(6, 2))
        self.start_btn = ttk.Button(btn, text="START ▶", width=12, command=self._start_capture, state="disabled")
        self.start_btn.pack(side="left", padx=2)
        self.stop_btn = ttk.Button(btn, text="STOP ■", width=12, command=self._stop_capture, state="disabled")
        self.stop_btn.pack(side="left", padx=2)
        self.cal_btn = ttk.Button(btn, text="Calibrate", width=10, command=self._calibrate, state="disabled")
        self.cal_btn.pack(side="left", padx=2)
        self.show_cal_btn = ttk.Button(
            btn,
            text="View Cal Values",
            width=14,
            command=self._show_calibration,
            state="disabled",
        )
        self.show_cal_btn.pack(side="left", padx=2)
        self.save_btn = ttk.Button(btn, text="Save…", width=8, command=self._save, state="disabled")
        self.save_btn.pack(side="left", padx=2)

        read = ttk.LabelFrame(main, text="Live values", padding=6)
        read.pack(fill="x", padx=10, pady=4)
        self.raw_str = tk.StringVar(value="—")
        self.conv_str = tk.StringVar(value="—")
        self.offset_str = tk.StringVar(value="—")
        self.status_str = tk.StringVar(value="—")
        ttk.Label(read, text="Raw angle:").grid(row=0, column=0, sticky="e")
        ttk.Label(read, textvariable=self.raw_str).grid(row=0, column=1, sticky="w")
        ttk.Label(read, text="Converted angle:").grid(row=1, column=0, sticky="e")
        ttk.Label(read, textvariable=self.conv_str).grid(row=1, column=1, sticky="w")
        ttk.Label(read, text="Offset:").grid(row=2, column=0, sticky="e")
        ttk.Label(read, textvariable=self.offset_str).grid(row=2, column=1, sticky="w")
        ttk.Label(read, text="Status:").grid(row=3, column=0, sticky="e")
        ttk.Label(read, textvariable=self.status_str).grid(row=3, column=1, sticky="w")

        self.status = tk.StringVar(value="Idle – not connected")
        ttk.Label(main, textvariable=self.status).pack(pady=(0,4))

    # Helpers -----------------------------------------------------------------
    @staticmethod
    def _ports():
        return [p.device for p in serial.tools.list_ports.comports()] or ["-"]

    def _refresh_ports(self):
        menu = self.port_menu["menu"]; menu.delete(0, "end")
        for p in self._ports():
            menu.add_command(label=p, command=lambda v=p: self.port_var.set(v))
        self.port_var.set("-")

    def _browse_elf(self):
        fn = filedialog.askopenfilename(title="Select ELF", filetypes=[("ELF","*.elf"), ("All","*.*")])
        if fn:
            self.elf_path.set(fn)

    def _toggle_conn(self):
        self._disconnect() if self.connected else self._connect()

    def _connect(self):
        port, elf = self.port_var.get(), self.elf_path.get()
        if port in ("", "-") or not elf:
            messagebox.showwarning("Missing", "Choose COM port and ELF file")
            return
        if not pathlib.Path(elf).is_file():
            messagebox.showerror("File", "ELF not found")
            return
        try:
            self.scope.connect(port, elf)
            self.run_var = self.scope.get_variable(RUN_REQ_VAR)
            self.cal_var = self.scope.get_variable(CAL_REQ_VAR)
            self.mon_vars = {k: self.scope.get_variable(p) for k, p in VAR_PATHS.items()}
        except Exception as e:
            messagebox.showerror("Connect", str(e))
            self.scope.disconnect()
            return
        self.connected = True
        self.conn_btn.config(text="Disconnect")
        self.start_btn.config(state="normal")
        self.cal_btn.config(state="normal")
        self.show_cal_btn.config(state="normal")
        self.status.set(f"Connected ({port})")

    def _disconnect(self):
        self._stop_capture()
        self.scope.disconnect()
        self.connected = False
        self.start_btn.config(state="disabled")
        self.stop_btn.config(state="disabled")
        self.save_btn.config(state="disabled")
        self.cal_btn.config(state="disabled")
        self.show_cal_btn.config(state="disabled")
        self.conn_btn.config(text="Connect")
        self.status.set("Disconnected")

    # Capture -----------------------------------------------------------------
    def _start_capture(self):
        if self._cap_thread and self._cap_thread.is_alive():
            return
        try:
            dur = float(self.dur_entry.get())
            dt_ms = float(self.sample_entry.get())
            if dur <= 0 or dt_ms <= 0:
                raise ValueError
        except ValueError:
            messagebox.showerror("Input", "Enter valid positive numbers")
            return
        if USE_SCOPE and not self.connected:
            messagebox.showwarning("Not connected", "Connect to a target first")
            return
        self.data = {k: [] for k in VAR_PATHS}
        self.data["t"] = []
        self._stop_flag.clear()
        self.ts = dt_ms / 1000.0

        try:
            self.run_var.set_value(1)
        except Exception:
            pass

        self._cap_thread = threading.Thread(target=self._worker, args=(dur,), daemon=True)
        self._cap_thread.start()
        self.start_btn.config(state="disabled")
        self.stop_btn.config(state="normal")
        self.save_btn.config(state="disabled")

    def _stop_capture(self):
        self._stop_flag.set()
        try:
            self.run_var.set_value(0)
        except Exception:
            pass

    def _calibrate(self):
        if not self.connected:
            return
        try:
            self.cal_var.set_value(1)
        except Exception:
            pass

    def _show_calibration(self):
        """Display the current calibration values from the target."""
        if not self.connected:
            messagebox.showinfo("Not connected", "Connect to a target first")
            return
        win = tk.Toplevel(self.root)
        win.title("Calibration values")
        row = 0
        for name, path in CAL_VARS.items():
            ttk.Label(win, text=f"{name}:").grid(row=row, column=0, sticky="e", padx=4, pady=2)
            try:
                var = self.scope.get_variable(path)
                val = var.get_value() if var else "—"
            except Exception:
                val = "—"
            ttk.Label(win, text=str(val)).grid(row=row, column=1, sticky="w", padx=4, pady=2)
            row += 1
        ttk.Button(win, text="Close", command=win.destroy).grid(row=row, column=0, columnspan=2, pady=(6, 2))

    def _worker(self, dur: float):
        try:
            self.status.set("Running + logging…")
            t0 = time.perf_counter()
            end_time = t0 + dur

            vars_to_sample = [self.mon_vars[k] for k in VAR_PATHS]
            self.scope.prepare_scope(vars_to_sample, int(self.ts * 1000))
            sample_idx = 0

            while not self._stop_flag.is_set() and time.perf_counter() < end_time:
                if self.scope.scope_ready():
                    chans = self.scope.get_scope_data()
                    self.scope.request_scope_data()
                    if chans:
                        n = len(next(iter(chans.values())))
                        self.data["t"].extend([(sample_idx + i) * self.ts for i in range(n)])
                        sample_idx += n
                        for ch, vals in chans.items():
                            key = PATH_TO_KEY.get(ch)
                            if key is None:
                                continue
                            self.data[key].extend(vals)
                time.sleep(0.25)
        finally:
            try:
                if self.root.winfo_exists():
                    self.root.after(0, self._worker_done)
            except tk.TclError:
                pass

    def _worker_done(self):
        self.start_btn.config(state="normal")
        self.stop_btn.config(state="disabled")
        if self.data.get("t"):
            self.save_btn.config(state="normal")
            self.status.set("Capture finished")
        else:
            self.status.set("Stopped / no data")

    def _poll_gui(self):
        try:
            if not self.root.winfo_exists():
                return
        except tk.TclError:
            return
        if self.connected:
            try:
                self.raw_str.set(f"{self.mon_vars['rawAngle'].get_value():.1f}")
                self.conv_str.set(f"{self.mon_vars['convertedAngle'].get_value():.1f}")
                self.offset_str.set(f"{self.mon_vars['offset'].get_value():.1f}")
                self.status_str.set(str(self.mon_vars['status'].get_value()))
            except Exception:
                self.raw_str.set("—")
                self.conv_str.set("—")
                self.offset_str.set("—")
                self.status_str.set("—")
        else:
            self.raw_str.set("—")
            self.conv_str.set("—")
            self.offset_str.set("—")
            self.status_str.set("—")
        self._poll_job = self.root.after(self.GUI_POLL_MS, self._poll_gui)

    # Save --------------------------------------------------------------------
    def _save(self):
        if not self.data.get("t"):
            messagebox.showinfo("No data", "Nothing to save")
            return
        fn = filedialog.asksaveasfilename(defaultextension=".csv", filetypes=[("CSV","*.csv"),("All","*.*")])
        if not fn:
            return
        try:
            import pandas as pd
            pd.DataFrame(self.data).to_csv(fn, index=False)
        except Exception as e:
            messagebox.showerror("Save", str(e))
            return
        messagebox.showinfo("Saved", fn)

    # Cleanup -----------------------------------------------------------------
    def _on_close(self):
        self._stop_flag.set()
        if self._poll_job is not None:
            try:
                self.root.after_cancel(self._poll_job)
            except Exception:
                pass
        if self._cap_thread and self._cap_thread.is_alive():
            self._cap_thread.join(timeout=2)
        self.scope.disconnect()
        try:
            if self.root.winfo_exists():
                self.root.destroy()
        except tk.TclError:
            pass


if __name__ == "__main__":
    gui = ResolverEncoderGUI()
    try:
        gui.root.mainloop()
    except KeyboardInterrupt:
        gui._on_close()
