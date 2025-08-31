#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pyX2Cscope motor logger GUI — hardware-only, with robust RUN/STOP and ms+kHz displays
Reverted connection flow to X2CScope(port=...) + import_variables(elf)

• Serial port dropdown + Refresh (pyserial)
• Correct sampling: set_sample_time(f), Fs = 20_000 / f (PWM ISR stays at 20 kHz)
• Linked Desired Hz ↔ f; shows Actual Fs (Hz & kHz) and Ts (ms)
• Live feasibility panel (UART throughput, total size, risk bullets)
• “Ready double-check” after first ready=True before reading
• Read-time sanity check vs UART capacity estimate
• Per-channel scaling and RUN/STOP/velocity preserved
• RUN/STOP sends are hardened: baseline, timed asserts, periodic reassert, clean deassert

Tested with: Python 3.11, pyX2Cscope ≥0.4.x, Windows 10/11
"""

from __future__ import annotations

import pathlib
import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional

import tkinter as tk
from tkinter import filedialog, messagebox, ttk

# ----- Required hardware libs -----
try:
    from pyx2cscope.x2cscope import X2CScope  # type: ignore
except Exception as e:
    raise SystemExit(
        "pyX2Cscope is required. Install with:  pip install pyx2cscope\n"
        f"Import error: {e}"
    )

# Serial port enumeration for dropdown
try:
    import serial.tools.list_ports as list_ports  # type: ignore
    _HAS_PYSERIAL = True
except Exception:
    list_ports = None  # type: ignore
    _HAS_PYSERIAL = False

# ----- Optional runtime deps (plot & save) -----
try:
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg  # type: ignore
except ImportError:  # pragma: no cover
    plt = None  # type: ignore

try:
    import pandas as pd  # type: ignore
except ImportError:  # pragma: no cover
    pd = None  # type: ignore

try:
    import scipy.io as sio  # type: ignore
except ImportError:  # pragma: no cover
    sio = None  # type: ignore

# Optional: load base control-loop Ts from data-model-dump.yaml
try:
    import yaml  # type: ignore
except ImportError:  # pragma: no cover
    yaml = None  # type: ignore


# ====== Fixed device timing ======
PWM_HZ = 20_000.0  # ISR frequency; effective Fs = 20_000 / f

# ====== Variables ======
VAR_PATHS = {
    "idqCmd_q":        "motor.idqCmd.q",
    "Idq_q":           "motor.idq.q",
    "Idq_d":           "motor.idq.d",
    "OmegaElectrical": "motor.omegaElectrical",
    "OmegaCmd":        "motor.omegaCmd",
}
PATH_TO_KEY = {v: k for k, v in VAR_PATHS.items()}

# Control paths (write-only)
HWUI_VAR     = "app.hardwareUiEnabled"
VEL_CMD_VAR  = "motor.apiData.velocityReference"   # counts
RUN_REQ_VAR  = "motor.apiData.runMotorRequest"
STOP_REQ_VAR = "motor.apiData.stopMotorRequest"


# ====== Feasibility math ======
@dataclass
class FeasInputs:
    V: int
    Bv_list: List[int]
    f: int
    baud: int
    duration_s: float
    sda_bytes: Optional[int] = None

@dataclass
class FeasOutputs:
    Fs: float
    bytes_per_sample: int
    payload_Bps: float
    uart_capacity_Bps: float
    uart_ratio: float
    uart_badge: str
    total_bytes: int
    buffer_time_s: Optional[float]
    bullets: List[str]

def _badge_from_ratio(r: float) -> str:
    if r < 0.4: return "GREEN"
    if r < 0.7: return "AMBER"
    return "RED"

def compute_feas(inp: FeasInputs) -> FeasOutputs:
    f = max(1, int(inp.f))
    Fs = PWM_HZ / f
    bytes_per_sample = sum(inp.Bv_list) if inp.Bv_list else 2 * inp.V
    payload_Bps = bytes_per_sample * Fs
    uart_capacity_Bps = inp.baud / 10.0
    ratio = payload_Bps / max(1.0, uart_capacity_Bps)
    badge = _badge_from_ratio(ratio)
    total_bytes = int(round(bytes_per_sample * Fs * max(0.0, inp.duration_s)))
    buffer_time = None
    if inp.sda_bytes and bytes_per_sample > 0:
        buffer_time = inp.sda_bytes * f / (bytes_per_sample * PWM_HZ)

    bullets: List[str] = []
    bullets.append(f"At f = {f}, effective Fs = {Fs:,.0f} Hz (20k/{f}).")
    bullets.append(f"Payload ≈ {payload_Bps:,.0f} B/s vs UART ≈ {uart_capacity_Bps:,.0f} B/s → {badge}.")
    bullets.append(
        "Likely overrun/choppy. Increase f or raise UART baud." if badge=="RED"
        else "Tight headroom; overhead may drop samples." if badge=="AMBER"
        else "Comfortable headroom."
    )
    if buffer_time is None:
        bullets.append("Buffer size unknown. Use 'Probe Buffer' (if supported) to estimate SDA capacity.")
    else:
        if inp.duration_s > buffer_time:
            bullets.append(f"Estimated buffer time ≈ {buffer_time:.3f} s < duration {inp.duration_s:.3f} s → streaming/chunking required.")
        else:
            bullets.append(f"Estimated buffer time ≈ {buffer_time:.3f} s ≥ duration.")
    if total_bytes > 25_000_000:
        bullets.append(f"Total size ≈ {total_bytes/1e6:.1f} MB → CSV may be large/slow.")
    bullets.append("Rule of thumb (V=5, 2 B/var): 115200→f≥18; 230400→f≥9; 921600→f≥3.")

    return FeasOutputs(Fs, bytes_per_sample, payload_Bps, uart_capacity_Bps, ratio, badge, total_bytes, buffer_time, bullets)


# ====== Scope wrapper (hardware only; reverted connect) ======
class ScopeHW:
    def __init__(self):
        self.scope: Optional[X2CScope] = None
        self.port: Optional[str] = None
        self.baud: int = 115200  # just for display

    def connect(self, port: str, elf: str, baud: int):
        """Reverted to previously working style: X2CScope(port=...) + import_variables(elf)."""
        self.port = port
        self.baud = int(baud)
        self.scope = X2CScope(port=port)  # constructor with port only
        self.scope.import_variables(elf)

    def disconnect(self):
        if self.scope:
            try: self.scope.disconnect()
            except Exception: pass
        self.scope = None

    def get_variable(self, path: str):
        if not self.scope: raise RuntimeError("Not connected")
        return self.scope.get_variable(path)

    def configure_channels(self, variables: List[object]):
        if not self.scope: raise RuntimeError("Not connected")
        # Clear channels
        if hasattr(self.scope, "clear_all_scope_channel"):
            self.scope.clear_all_scope_channel()
        else:
            if hasattr(self.scope, "clear_scope_channels"):
                self.scope.clear_scope_channels()
            elif hasattr(self.scope, "clear_all_scope_channels"):
                self.scope.clear_all_scope_channels()
        # Add
        for var in variables:
            self.scope.add_scope_channel(var, trigger=False)

    def set_sample_factor(self, f: int):
        if not self.scope: raise RuntimeError("Not connected")
        self.scope.set_sample_time(int(max(1, f)))

    def request(self):
        if not self.scope: raise RuntimeError("Not connected")
        self.scope.request_scope_data()

    def ready(self) -> bool:
        if not self.scope: return False
        try:
            return bool(self.scope.is_scope_data_ready())
        except Exception:
            return False

    def read(self) -> Dict[str, List[float]]:
        if not self.scope: return {}
        data = self.scope.get_scope_channel_data(valid_data=True) or {}
        return data

    def get_device_info(self) -> Dict:
        if not self.scope: return {}
        try: return dict(self.scope.get_device_info() or {})
        except Exception: return {}


# ====== Main GUI ======
class MotorLoggerGUI:
    PRE_START = 1.0   # s before asserting RUN
    POST_STOP = 1.5   # s after STOP

    def __init__(self):
        self.root = tk.Tk()
        self.root.title("pyX2Cscope – Motor Logger (Hardware)")

        # State
        self.scope = ScopeHW()
        self.connected = False
        self._cap_thread: Optional[threading.Thread] = None
        self._stop_flag = threading.Event()
        self.data: Dict[str, List[float]] = {}
        self.var_enabled: Dict[str, tk.BooleanVar] = {}
        self.scale_vars: Dict[str, tk.StringVar] = {}
        self.scale_factors: Dict[str, float] = {k: 1.0 for k in VAR_PATHS}
        self.selected_vars: List[str] = list(VAR_PATHS)
        self.issue_text: Optional[str] = None
        self.bytes_per_var: Dict[str, int] = {k: 2 for k in VAR_PATHS}  # default 2B each

        # Results meta
        self.f_factor: int = 10
        self.Fs_actual: float = PWM_HZ / self.f_factor
        self.Ts_ms: float = 1000.0 / self.Fs_actual
        self.N_expected: int = 0
        self.N_raw: int = 0
        self.N_after_clip: int = 0
        self.read_time_s: float = 0.0
        self.read_time_est_s: float = 0.0
        self.read_time_note: str = "ok"

        self._build_widgets()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---------- UI ----------
    def _build_widgets(self):
        nb = ttk.Notebook(self.root)
        nb.pack(fill="both", expand=True)

        main = ttk.Frame(nb)
        nb.add(main, text="Logger")

        # Connection
        conn = ttk.LabelFrame(main, text="Connection", padding=10)
        conn.pack(fill="x", padx=10, pady=6)

        ttk.Label(conn, text="ELF file:").grid(row=0, column=0, sticky="e")
        self.elf_path = tk.StringVar()
        ttk.Entry(conn, textvariable=self.elf_path, width=44).grid(row=0, column=1, sticky="we", padx=4)
        ttk.Button(conn, text="Browse…", command=self._browse_elf).grid(row=0, column=2, padx=4)

        ttk.Label(conn, text="COM port:").grid(row=1, column=0, sticky="e", pady=4)
        self.port_var = tk.StringVar(value="(select)")
        self.port_menu = ttk.OptionMenu(conn, self.port_var, "(select)", *self._ports())
        self.port_menu.grid(row=1, column=1, sticky="we", padx=4)
        ttk.Button(conn, text="↻", width=3, command=self._refresh_ports).grid(row=1, column=2, padx=4)

        ttk.Label(conn, text="Baud:").grid(row=2, column=0, sticky="e")
        self.baud_combo = ttk.Combobox(conn, width=12, state="readonly",
                                       values=["115200","230400","460800","921600","Custom…"])
        self.baud_combo.set("115200"); self.baud_combo.grid(row=2, column=1, sticky="w", padx=4)
        self.custom_baud = tk.StringVar()
        self.custom_baud_e = ttk.Entry(conn, textvariable=self.custom_baud, width=12)
        self.custom_baud_e.grid(row=2, column=2, sticky="w"); self.custom_baud_e.grid_remove()
        self.baud_combo.bind("<<ComboboxSelected>>", lambda e: self._toggle_custom_baud())

        self.conn_btn = ttk.Button(conn, text="Connect", command=self._toggle_conn)
        self.conn_btn.grid(row=3, column=0, columnspan=3, pady=(6, 2), sticky="we")

        # Parameters
        parms = ttk.LabelFrame(main, text="Parameters", padding=10)
        parms.pack(fill="x", padx=10, pady=4)

        self._lock_widgets: List[tk.Widget] = []

        # Speed (RPM) and scale
        ttk.Label(parms, text="Speed (RPM):").grid(row=0, column=0, sticky="e")
        self.speed_entry = ttk.Entry(parms, width=12); self.speed_entry.insert(0, "2800")
        self.speed_entry.grid(row=0, column=1, padx=6, pady=2); self._lock_widgets.append(self.speed_entry)
        ttk.Label(parms, text="min 1088 RPM - max 5000 RPM").grid(row=0, column=2, sticky="w")

        ttk.Label(parms, text="Scale (RPM/cnt):").grid(row=1, column=0, sticky="e")
        self.scale_entry = ttk.Entry(parms, width=12); self.scale_entry.insert(0, "0.19913")
        self.scale_entry.grid(row=1, column=1, padx=6, pady=2); self._lock_widgets.append(self.scale_entry)

        ttk.Label(parms, text="Duration (s):").grid(row=2, column=0, sticky="e")
        self.duration_e = ttk.Entry(parms, width=12); self.duration_e.insert(0, "10")
        self.duration_e.grid(row=2, column=1, padx=6, pady=2); self._lock_widgets.append(self.duration_e)

        # Sample rate controls: Desired Hz <-> f, Actual Fs (Hz/kHz) and Ts (ms)
        ttk.Label(parms, text="Desired Hz:").grid(row=3, column=0, sticky="e")
        self.hz_e = ttk.Entry(parms, width=12); self.hz_e.insert(0, "2000")
        self.hz_e.grid(row=3, column=1, padx=6, pady=2); self._lock_widgets.append(self.hz_e)
        self.hz_e.bind("<KeyRelease>", lambda e: self._hz_to_f())

        ttk.Label(parms, text="or f = 20k/Hz:").grid(row=3, column=2, sticky="e")
        self.f_var = tk.IntVar(value=10)
        self.f_spin = ttk.Spinbox(parms, from_=1, to=10000, width=10, textvariable=self.f_var, command=self._f_to_hz)
        self.f_spin.grid(row=3, column=3, padx=6, pady=2); self._lock_widgets.append(self.f_spin)
        self.fs_label = ttk.Label(parms, text="Actual Fs: 2,000 Hz (2.000 kHz), Ts: 0.500 ms")
        self.fs_label.grid(row=3, column=4, sticky="w")

        # Feasibility mini panel
        feas = ttk.LabelFrame(parms, text="Feasibility")
        feas.grid(row=4, column=0, columnspan=5, sticky="we", pady=(8,4))
        feas.grid_columnconfigure(2, weight=1)

        ttk.Label(feas, text="UART:").grid(row=0, column=0, sticky="e")
        self.uart_badge = tk.Label(feas, text="GREY", bg="#6b6b6b", fg="white", width=8)
        self.uart_badge.grid(row=0, column=1, sticky="w", padx=(4,8))
        self.uart_label = ttk.Label(feas, text="UART load: —")
        self.uart_label.grid(row=0, column=2, sticky="w")

        ttk.Label(feas, text="Total:").grid(row=1, column=0, sticky="e")
        self.total_badge = tk.Label(feas, text="GREY", bg="#6b6b6b", fg="white", width=8)
        self.total_badge.grid(row=1, column=1, sticky="w", padx=(4,8))
        self.total_label = ttk.Label(feas, text="Total size: —")
        self.total_label.grid(row=1, column=2, sticky="w")

        self.risks_text = tk.Text(feas, height=5, width=80)
        self.risks_text.grid(row=2, column=0, columnspan=3, sticky="we", pady=(6,2))
        self.risks_text.configure(state="disabled")

        # Per-channel scaling block
        ttk.Label(parms, text="Channel Scaling:").grid(row=5, column=0, columnspan=2, sticky="w", pady=(6,2))
        tbl = ttk.Frame(parms); tbl.grid(row=6, column=0, columnspan=3, sticky="w")
        self.var_enabled.clear(); self.scale_vars.clear()
        for r, (name, _) in enumerate(VAR_PATHS.items()):
            en = tk.BooleanVar(value=True); self.var_enabled[name] = en
            chk = ttk.Checkbutton(tbl, variable=en); chk.grid(row=r, column=0, padx=(0,4))
            self._lock_widgets.append(chk)
            ttk.Label(tbl, text=name, width=16).grid(row=r, column=1, sticky="e", pady=2)
            if name in ("idqCmd_q", "Idq_q", "Idq_d"):
                sv = tk.StringVar(value="0.0003125")
            else:
                sv = tk.StringVar(value="1.0")
            self.scale_vars[name] = sv
            ent = ttk.Entry(tbl, textvariable=sv, width=10)
            ent.grid(row=r, column=2, sticky="w", padx=6)
            self._lock_widgets.append(ent)

        # Buttons
        btn = ttk.Frame(main); btn.pack(pady=(8, 2))
        self.start_btn = ttk.Button(btn, text="START ▶", width=12, command=self._start_capture, state="disabled")
        self.stop_btn  = ttk.Button(btn,  text="STOP ■",  width=12, command=self._stop_capture,  state="disabled")
        self.curr_btn  = ttk.Button(btn,  text="Currents", width=10, command=self._plot_currents, state="disabled")
        self.omega_btn = ttk.Button(btn,  text="Omega",   width=10, command=self._plot_omega,   state="disabled")
        self.save_btn  = ttk.Button(btn,  text="Save…",   width=8,  command=self._save,         state="disabled")
        for b in (self.start_btn, self.stop_btn, self.curr_btn, self.omega_btn, self.save_btn):
            b.pack(side="left", padx=3)

        # Status & validity
        status_frame = ttk.Frame(main); status_frame.pack(fill="x", pady=(0,8))
        self.status = tk.StringVar(value="Idle – not connected")
        ttk.Label(status_frame, textvariable=self.status).pack(side="left")

        self.validity_var = tk.StringVar(value="No data")
        self.validity_label = tk.Label(status_frame, textvariable=self.validity_var,
                                       width=12, bg="gray", fg="white", relief="groove")
        self.validity_label.pack(side="left", padx=6)

        self.issue_var = tk.StringVar(value="")
        ttk.Label(status_frame, textvariable=self.issue_var, wraplength=520, foreground="red").pack(side="left", padx=6)

        # Hook recompute for feasibility when inputs change
        for w in (self.hz_e, self.duration_e):
            w.bind("<KeyRelease>", lambda e: self._update_feasibility())
        self.f_spin.bind("<<Increment>>", lambda e: self._update_feasibility())
        self.f_spin.bind("<<Decrement>>", lambda e: self._update_feasibility())
        self.baud_combo.bind("<<ComboboxSelected>>", lambda e: self._update_feasibility())
        self.custom_baud_e.bind("<KeyRelease>", lambda e: self._update_feasibility())
        for _, var in self.var_enabled.items():
            var.trace_add("write", lambda *_: self._update_feasibility())

        # Initial feasibility
        self._update_fs_label()
        self._update_feasibility()

    # ---------- Helpers ----------
    def _toggle_custom_baud(self):
        if self.baud_combo.get().startswith("Custom"):
            self.custom_baud_e.grid()
        else:
            self.custom_baud_e.grid_remove()
        self._update_feasibility()

    def _baud_value(self) -> int:
        if self.baud_combo.get().startswith("Custom"):
            try: return int(self.custom_baud.get())
            except Exception: return 115200
        try: return int(self.baud_combo.get())
        except Exception: return 115200

    def _ports(self) -> List[str]:
        if _HAS_PYSERIAL and list_ports:
            pts = [p.device for p in list_ports.comports()]
            return pts if pts else ["(no ports)"]
        return ["(pyserial not installed)"]

    def _refresh_ports(self):
        menu = self.port_menu["menu"]; menu.delete(0, "end")
        items = self._ports()
        for p in items:
            menu.add_command(label=p, command=lambda v=p: self.port_var.set(v))
        self.port_var.set(items[0])

    def _browse_elf(self):
        fn = filedialog.askopenfilename(title="Select ELF", filetypes=[("ELF","*.elf"), ("All","*.*")])
        if fn: self.elf_path.set(fn)

    def _hz_to_f(self):
        try:
            hz = int(self.hz_e.get().strip() or "0")
        except Exception:
            return
        hz = max(1, min(20000, hz))
        f = max(1, int(round(PWM_HZ / hz)))
        self.f_var.set(f)
        self._update_fs_label()
        self._update_feasibility()

    def _f_to_hz(self):
        f = max(1, int(self.f_var.get()))
        hz = int(round(PWM_HZ / f))
        self.hz_e.delete(0, tk.END); self.hz_e.insert(0, str(hz))
        self._update_fs_label()
        self._update_feasibility()

    def _update_fs_label(self):
        f = max(1, int(self.f_var.get()))
        Fs = PWM_HZ / f
        Ts_ms = 1000.0 / Fs
        self.f_factor = f
        self.Fs_actual = Fs
        self.Ts_ms = Ts_ms
        self.fs_label.config(text=f"Actual Fs: {Fs:,.0f} Hz ({Fs/1000.0:.3f} kHz), Ts: {Ts_ms:.3f} ms")

    def _update_feasibility(self):
        names = [k for k, v in self.var_enabled.items() if v.get()]
        Bv_list = [self.bytes_per_var.get(k, 2) for k in names]
        f = max(1, int(self.f_var.get()))
        baud = self._baud_value()
        try:
            duration = float(self.duration_e.get() or "0")
        except Exception:
            duration = 0.0

        fo = compute_feas(FeasInputs(V=len(names), Bv_list=Bv_list, f=f, baud=baud, duration_s=duration, sda_bytes=None))
        # Badges + labels
        badge = fo.uart_badge
        color = {"GREEN":"#0b8f2f","AMBER":"#b57f00","RED":"#c62828"}.get(badge, "#6b6b6b")
        self.uart_badge.configure(text=badge, bg=color)
        Ts_ms = 1000.0 / fo.Fs if fo.Fs > 0 else 0.0
        self.uart_label.configure(
            text=f"UART load: {fo.payload_Bps:,.0f} / {fo.uart_capacity_Bps:,.0f} B/s ({100*fo.uart_ratio:.1f}%) | Fs {fo.Fs:,.0f} Hz ({fo.Fs/1000:.3f} kHz) | Ts {Ts_ms:.3f} ms"
        )
        if fo.total_bytes > 25_000_000:
            self.total_badge.configure(text="AMBER", bg="#b57f00")
        else:
            self.total_badge.configure(text="GREEN", bg="#0b8f2f")
        self.total_label.configure(text=f"Total size (est.): {fo.total_bytes/1e6:.2f} MB")
        self.risks_text.configure(state="normal"); self.risks_text.delete("1.0", tk.END)
        self.risks_text.insert(tk.END, "• " + "\n• ".join(fo.bullets))
        self.risks_text.configure(state="disabled")

    # ---------- Robust variable writes ----------
    def _write_var_safe(self, var, value: int, repeats: int = 3, delay_s: float = 0.01):
        """Try a few known setter names and repeat with tiny delays to be robust."""
        setters = ("set_value", "set", "write", "write_value")
        ok = False
        for _ in range(repeats):
            for name in setters:
                try:
                    if hasattr(var, name):
                        getattr(var, name)(value)  # type: ignore
                        ok = True
                except Exception:
                    ok = False
            time.sleep(delay_s)
        return ok

    # ---------- Connection ----------
    def _toggle_conn(self):
        if self.connected:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self.port_var.get().strip()
        if port.startswith("("):
            messagebox.showwarning("Port", "Select a serial port"); return
        elf = self.elf_path.get().strip()
        if not elf or not pathlib.Path(elf).is_file():
            messagebox.showwarning("ELF", "Select a valid ELF file"); return
        baud = self._baud_value()
        try:
            self.scope.connect(port, elf, baud)  # reverted style
            # Get variables
            self.hwui     = self.scope.get_variable(HWUI_VAR)
            self.cmd_var  = self.scope.get_variable(VEL_CMD_VAR)
            self.run_var  = self.scope.get_variable(RUN_REQ_VAR)
            self.stop_var = self.scope.get_variable(STOP_REQ_VAR)
            self.mon_vars = {k: self.scope.get_variable(p) for k, p in VAR_PATHS.items()}

            # Disable HMI (robust)
            self._write_var_safe(self.hwui, 0)

            # ISR info via optional YAML (best-effort)
            status_extra = ""
            try:
                elf_path = pathlib.Path(self.elf_path.get())
                dm_file = next((p / "data-model-dump.yaml" for p in elf_path.parents
                                if (p / "data-model-dump.yaml").is_file()), None)
                if dm_file and yaml is not None:
                    with open(dm_file, "r", encoding="utf-8") as f:
                        _dm = yaml.load(f, Loader=yaml.FullLoader)
                    base_us = float(_dm["drive"]["sampling_time"]["current"]) * 1e6
                    status_extra = f" – ISR: {base_us:.0f} µs"
            except Exception:
                pass

            self.status.set(f"Connected ({port} @ {baud}){status_extra}")
        except Exception as e:
            messagebox.showerror("Connect", str(e))
            try: self.scope.disconnect()
            except Exception: pass
            return

        self.connected = True
        self.conn_btn.config(text="Disconnect")
        self.start_btn.config(state="normal")

    def _disconnect(self):
        self._stop_capture()
        try:
            if hasattr(self, "run_var"):  self._write_var_safe(self.run_var, 0)
            if hasattr(self, "stop_var"): self._write_var_safe(self.stop_var, 0)
        except Exception:
            pass
        self.scope.disconnect()
        self.connected = False
        for b in (self.start_btn, self.stop_btn, self.curr_btn, self.omega_btn, self.save_btn):
            b.config(state="disabled")
        self.conn_btn.config(text="Connect")
        self.status.set("Disconnected")

    # ---------- Capture ----------
    def _start_capture(self):
        if self._cap_thread and self._cap_thread.is_alive():
            return
        # Inputs
        try:
            rpm  = float(self.speed_entry.get());    assert rpm >= 0
            scale= float(self.scale_entry.get());    assert scale > 0
            dur  = float(self.duration_e.get());     assert dur > 0
            f    = int(self.f_var.get());            assert f >= 1
        except Exception:
            messagebox.showerror("Input", "Enter valid positive numbers"); return
        if not self.connected:
            messagebox.showwarning("Not connected", "Connect to a target first"); return

        # Update scaling
        self.selected_vars = [k for k, v in self.var_enabled.items() if v.get()]
        if not self.selected_vars:
            messagebox.showwarning("Variables", "Select at least one variable")
            return
        for k in self.selected_vars:
            try:
                self.scale_factors[k] = float(self.scale_vars[k].get())
            except Exception:
                self.scale_factors[k] = 1.0

        # Program velocity command in counts (RPM / scale), send robustly twice
        try:
            self._write_var_safe(self.cmd_var, int(round(rpm / scale)), repeats=2, delay_s=0.01)
        except Exception:
            pass

        # Prepare data holders
        self.data = {k: [] for k in self.selected_vars}
        self.data["t"] = []
        self.data["MotorRunning"] = []
        self.issue_text = None
        self.read_time_note = "ok"
        self.N_expected = 0; self.N_raw = 0; self.N_after_clip = 0

        # UI lock
        self._stop_flag.clear()
        self.start_btn.config(state="disabled"); self.stop_btn.config(state="normal")
        for b in (self.curr_btn, self.omega_btn, self.save_btn): b.config(state="disabled")
        for w in self._lock_widgets: w.config(state="disabled")
        self.validity_label.config(text="Capturing…", bg="gray"); self.issue_var.set("")
        self.status.set("Running + logging…")

        self._cap_thread = threading.Thread(target=self._capture_worker, args=(dur, f), daemon=True)
        self._cap_thread.start()

    def _stop_capture(self):
        self._stop_flag.set()
        # assert STOP=1, RUN=0 to be safe (robust)
        try:
            self._write_var_safe(self.stop_var, 1, repeats=2)
            self._write_var_safe(self.run_var, 0, repeats=2)
        except Exception:
            pass
        # Wait briefly for worker to exit so a new capture can start
        if self._cap_thread and self._cap_thread.is_alive():
            self._cap_thread.join(timeout=1.0)

    def _capture_worker(self, duration_s: float, f: int):
        """Single-shot capture with robust RUN/STOP, double-check ready, and read-time sanity."""
        try:
            # Configure channels and sample factor
            vars_to_sample = [self.mon_vars[k] for k in self.selected_vars]
            self.scope.configure_channels(vars_to_sample)
            self.scope.set_sample_factor(f)
            Fs = PWM_HZ / max(1, f)
            self.Fs_actual = Fs
            self.Ts_ms = 1000.0 / Fs

            # Clean baseline: RUN=0, STOP=0 (twice with a short pause)
            try:
                self._write_var_safe(self.run_var, 0, repeats=2)
                self._write_var_safe(self.stop_var, 0, repeats=2)
                time.sleep(0.02)
            except Exception:
                pass

            # Timeline
            t0 = time.perf_counter()
            run_time  = t0 + self.PRE_START
            stop_time = run_time + duration_s
            end_time  = stop_time + self.POST_STOP

            # Start MCU capture now
            self.scope.request()

            # Control window
            run_set = False
            stop_set = False
            last_run_refresh = 0.0
            run_refresh_period = 0.2  # re-assert RUN=1 every 200 ms during the window

            # Wait for ready with double-check confirmations
            confirm_needed = 3
            confirm_gap_s = 0.05
            timeout = duration_s + self.PRE_START + self.POST_STOP + 1.0

            while not self._stop_flag.is_set():
                now = time.perf_counter()

                # Assert RUN at run_time, and keep it refreshed
                if not run_set and now >= run_time:
                    try:
                        self._write_var_safe(self.run_var, 1, repeats=2)
                        run_set = True
                        last_run_refresh = now
                    except Exception:
                        pass
                if run_set and not stop_set and (now - last_run_refresh) >= run_refresh_period:
                    # periodic reassert to be extra safe
                    try:
                        self._write_var_safe(self.run_var, 1, repeats=1)
                    except Exception:
                        pass
                    last_run_refresh = now

                # Assert STOP at stop_time, then drop RUN shortly after
                if not stop_set and now >= stop_time:
                    try:
                        self._write_var_safe(self.stop_var, 1, repeats=2)
                        # small gap then drop RUN to 0
                        time.sleep(0.01)
                        self._write_var_safe(self.run_var, 0, repeats=2)
                        stop_set = True
                    except Exception:
                        pass

                # Ready double-check
                if self.scope.ready():
                    ok = True
                    for _ in range(confirm_needed):
                        time.sleep(confirm_gap_s)
                        if not self.scope.ready():
                            ok = False
                            break
                    if ok:
                        break

                if (now - t0) > timeout:
                    break

                time.sleep(0.01)

            # End-of-window cleanup: ensure RUN=0, STOP=1 remains high until finish
            try:
                self._write_var_safe(self.run_var, 0, repeats=2)
                if stop_set:
                    self._write_var_safe(self.stop_var, 1, repeats=1)
            except Exception:
                pass

            # If capture was stopped by user, bail out before reading
            if self._stop_flag.is_set():
                self.issue_text = "Capture stopped by user"
                return

            # Estimate expected bytes/time for sanity
            Bv_list = [self.bytes_per_var.get(k, 2) for k in self.selected_vars]
            bytes_per_sample = sum(Bv_list)
            baud = self._baud_value()
            uart_Bps = max(1.0, baud / 10.0)
            total_window = self.PRE_START + duration_s + self.POST_STOP
            self.N_expected = int(round(total_window * Fs))
            est_bytes = max(0, bytes_per_sample * self.N_expected)
            self.read_time_est_s = est_bytes / uart_Bps if uart_Bps > 0 else 0.0

            # Time the read
            tR0 = time.perf_counter()
            raw = self.scope.read()  # dict: varname -> List[number]
            tR1 = time.perf_counter()
            self.read_time_s = tR1 - tR0

            if not raw:
                self.issue_text = "No data returned from scope."
                self.N_raw = 0; self.N_after_clip = 0
                return

            # Align to shortest channel length
            lens = [len(v) for v in raw.values() if isinstance(v, list)]
            if not lens:
                self.issue_text = "Empty capture."
                return
            Nmin = min(lens); self.N_raw = max(lens)
            self.N_after_clip = Nmin

            # Build time vector from Fs
            tvec = [i / Fs for i in range(Nmin)]
            self.data = {"t": tvec, "MotorRunning": []}
            # MotorRunning flag
            self.data["MotorRunning"] = [1 if (self.PRE_START <= tt < self.PRE_START + duration_s) else 0 for tt in tvec]

            # Apply per-channel scaling
            for chname, vals in raw.items():
                key = PATH_TO_KEY.get(str(chname))
                if key and key in self.selected_vars:
                    scale = self.scale_factors.get(key, 1.0)
                    self.data[key] = [float(v) * scale for v in vals[:Nmin]]

            # Read-time sanity note
            if self.read_time_est_s > 0 and self.read_time_s > 1.3 * self.read_time_est_s:
                self.read_time_note = "read slower than UART estimate — check baud/cable/USB hub/OS load"

        except Exception as e:
            self.issue_text = f"Capture error: {e}"
        finally:
            # Always deassert both at the very end (best-effort)
            try:
                self._write_var_safe(self.run_var, 0, repeats=2)
                self._write_var_safe(self.stop_var, 0, repeats=2)
            except Exception:
                pass
            try:
                if self.root.winfo_exists():
                    self.root.after(0, self._worker_done)
            except tk.TclError:
                pass

    def _worker_done(self):
        # Re-enable UI
        self._cap_thread = None
        self.start_btn.config(state="normal"); self.stop_btn.config(state="disabled")
        for w in self._lock_widgets: w.config(state="normal")

        if "t" in self.data and self.data["t"]:
            # Plot enable
            if any(k in self.data for k in ("idqCmd_q","Idq_q","Idq_d")):
                self.curr_btn.config(state="normal")
            if any(k in self.data for k in ("OmegaElectrical","OmegaCmd")):
                self.omega_btn.config(state="normal")
            self.save_btn.config(state="normal")

            # Validity + messages — include Hz, kHz, and Ts (ms)
            msg_parts = []
            msg_parts.append(f"f={self.f_factor}, Fs={self.Fs_actual:,.0f} Hz ({self.Fs_actual/1000:.3f} kHz), Ts={self.Ts_ms:.3f} ms")
            if self.N_expected:
                msg_parts.append(f"N_expected={self.N_expected}")
            if self.N_raw:
                msg_parts.append(f"N_raw={self.N_raw}")
            if self.N_after_clip:
                msg_parts.append(f"N_after_clip={self.N_after_clip}")
            if self.read_time_est_s > 0:
                msg_parts.append(f"read {self.read_time_s:.3f}s (est {self.read_time_est_s:.3f}s)")
            if self.read_time_note != "ok":
                msg_parts.append(self.read_time_note)

            if self.issue_text:
                self.validity_label.config(text="Invalid Data", bg="red")
                self.issue_var.set(self.issue_text + " | " + " | ".join(msg_parts))
                messagebox.showwarning("Scope", self.issue_text)
            else:
                self.validity_label.config(text="Valid Data", bg="green")
                self.issue_var.set(" | ".join(msg_parts))
            self.status.set("Capture finished")
        else:
            self.validity_label.config(text="Invalid Data", bg="red")
            self.issue_var.set(self.issue_text or "No data captured")
            self.status.set("Stopped / no data")

    # ---------- Plot & save ----------
    def _plot_currents(self):
        if not self.data.get("t"):
            messagebox.showinfo("No data", "Nothing captured yet"); return
        if plt is None:
            messagebox.showerror("Plot", "Install matplotlib"); return
        fig, ax = plt.subplots(figsize=(8,4))
        plotted = False
        for k, lbl in (("idqCmd_q","idqCmd.q [A]"), ("Idq_q","idq.q [A]"), ("Idq_d","idq.d [A]")):
            if k in self.data and self.data[k]:
                ax.plot(self.data["t"], self.data[k], label=lbl, linewidth=0.9)
                plotted = True
        if not plotted:
            messagebox.showinfo("Plot", "No valid current data to plot."); return
        ax.set_xlabel("t [s]"); ax.set_ylabel("Current [scaled]"); ax.grid(True, linestyle=":", linewidth=0.5)
        ax.legend(fontsize="small")
        win = tk.Toplevel(self.root); win.title("Current traces")
        FigureCanvasTkAgg(fig, master=win).get_tk_widget().pack(fill="both", expand=True)
        fig.tight_layout()

    def _plot_omega(self):
        if not self.data.get("t"):
            messagebox.showinfo("No data", "Nothing captured yet"); return
        if plt is None:
            messagebox.showerror("Plot", "Install matplotlib"); return
        fig, ax = plt.subplots(figsize=(8,4))
        plotted = False
        for k, lbl in (("OmegaElectrical","omegaElectrical [scaled]"), ("OmegaCmd","omegaCmd [scaled]")):
            if k in self.data and self.data[k]:
                ax.plot(self.data["t"], self.data[k], label=lbl, linewidth=0.9)
                plotted = True
        if not plotted:
            messagebox.showinfo("Plot", "No valid omega data to plot."); return
        ax.set_xlabel("t [s]"); ax.set_ylabel("Speed [scaled]"); ax.grid(True, linestyle=":", linewidth=0.5)
        ax.legend(fontsize="small")
        win = tk.Toplevel(self.root); win.title("Omega traces")
        FigureCanvasTkAgg(fig, master=win).get_tk_widget().pack(fill="both", expand=True)
        fig.tight_layout()

    def _save(self):
        if not self.data.get("t"):
            messagebox.showinfo("No data", "Nothing to save"); return
        fn = filedialog.asksaveasfilename(defaultextension=".xlsx",
                                          filetypes=[("Excel","*.xlsx"),("MATLAB","*.mat"),("CSV","*.csv"),("All","*.*")])
        if not fn: return
        ext = pathlib.Path(fn).suffix.lower()
        try:
            if ext == ".mat":
                if sio is None: raise RuntimeError("scipy not installed")
                sio.savemat(fn, self.data)
            elif ext == ".csv":
                if pd is None: raise RuntimeError("pandas not installed")
                pd.DataFrame(self.data).to_csv(fn, index=False)
            else:  # Excel
                if pd is None: raise RuntimeError("pandas not installed")
                pd.DataFrame(self.data).to_excel(fn, index=False)
        except Exception as e:
            messagebox.showerror("Save", str(e)); return
        messagebox.showinfo("Saved", fn)

    # ---------- Cleanup ----------
    def _on_close(self):
        try:
            self._stop_flag.set()
            if self._cap_thread and self._cap_thread.is_alive():
                self._cap_thread.join(timeout=2)
            try:
                if hasattr(self, "run_var"):  self._write_var_safe(self.run_var, 0, repeats=2)
                if hasattr(self, "stop_var"): self._write_var_safe(self.stop_var, 0, repeats=2)
            except Exception:
                pass
            self.scope.disconnect()
        finally:
            try:
                if self.root.winfo_exists():
                    self.root.destroy()
            except tk.TclError:
                pass


# ====== Entry point ======
if __name__ == "__main__":
    gui = MotorLoggerGUI()
    gui.root.mainloop()
