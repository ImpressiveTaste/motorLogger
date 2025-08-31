#!/usr/bin/env python3
"""
pyX2Cscope motor logger GUI  –  with per-channel scaling
=======================================================

• Start / stop the motor and set a target speed (RPM)
• Live display of measured and commanded speed
• Logs five MCAF variables for a user-defined duration:
    ─ motor.idqCmd.q        (Q-axis torque command)
    ─ motor.idq.q           (Q-axis measured current)
    ─ motor.idq.d           (D-axis measured current)
    ─ motor.omegaElectrical (actual speed)
    ─ motor.omegaCmd        (target speed)
• New **Scaling** tab lets you type a multiplier for each variable
  (default 1.0).  The capture thread applies it on the fly, so plots
  and saved files show scaled values.

Tested with: pyX2Cscope 0.4.4, Python 3.11, Windows 10.
"""

from __future__ import annotations

import pathlib
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from typing import Dict, List, Union

import serial.tools.list_ports

# ─── Optional runtime deps (plot & save) ──────────────────────────────────────
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

# ─── Switch between real X2CScope and dummy backend ──────────────────────────
USE_SCOPE = True
if USE_SCOPE:
    from pyx2cscope.x2cscope import X2CScope

# ─── Sample interval limitations ----------------------------------------------
# Scope channel logging supports capturing all enabled variables at 1 ms.
# When the sample guard is enabled, the GUI blocks intervals shorter than
# ``MIN_DELAY_MS``.
DEFAULT_ENFORCE_SAMPLE_LIMIT = True
MIN_DELAY_MS = 1  # minimum sample interval in ms

# ─── Variable paths we want to log ───────────────────────────────────────────
VAR_PATHS = {
    "idqCmd_q":        "motor.idqCmd.q",
    "Idq_q":           "motor.idq.q",
    "Idq_d":           "motor.idq.d",
    "OmegaElectrical": "motor.omegaElectrical",
    "OmegaCmd":        "motor.omegaCmd",
}

# Reverse lookup: "motor.idqCmd.q" -> "idqCmd_q"
PATH_TO_KEY = {v: k for k, v in VAR_PATHS.items()}

# Control & read-back paths
HWUI_VAR     = "app.hardwareUiEnabled"
VEL_CMD_VAR  = "motor.apiData.velocityReference"   # counts
VEL_MEAS_VAR = "motor.apiData.velocityMeasured"    # counts
RUN_REQ_VAR  = "motor.apiData.runMotorRequest"
STOP_REQ_VAR = "motor.apiData.stopMotorRequest"

# ─── Dummy replacements (enable by setting USE_SCOPE = False) ────────────────
class _DummyVar:
    def __init__(self, name: str):
        self.name = name
        self._val = 0
    def set_value(self, v): self._val = v
    def get_value(self):    return self._val

class _ScopeWrapper:
    """Hides real / dummy selection."""
    def __init__(self):
        self._scope: Union[X2CScope, None] = None

    def connect(self, port: str, elf: str):
        if USE_SCOPE:
            self._scope = X2CScope(port=port)
            self._scope.import_variables(elf)

    def get_variable(self, path: str):
        if not USE_SCOPE:
            return _DummyVar(path)
        if self._scope is None:
            raise RuntimeError("Scope not connected")
        return self._scope.get_variable(path)

    def disconnect(self):
        if USE_SCOPE and self._scope:
            self._scope.disconnect()
            self._scope = None

    # Scope channel helpers -------------------------------------------------
    def prepare_scope(self, vars: List[object], sample_ms: int):
        """Configure scope channels and sampling interval."""
        if not USE_SCOPE or self._scope is None:
            return
        # pyX2Cscope renamed this helper in newer releases
        if hasattr(self._scope, "clear_scope_channels"):
            self._scope.clear_scope_channels()
        elif hasattr(self._scope, "clear_all_scope_channel"):
            self._scope.clear_all_scope_channel()
        elif hasattr(self._scope, "clear_all_scope_channels"):
            self._scope.clear_all_scope_channels()
        for idx, var in enumerate(vars):
            ch = self._scope.add_scope_channel(var)
        self._scope.set_sample_time(sample_ms)
        self._scope.request_scope_data()

    def scope_ready(self) -> bool:
        if not USE_SCOPE or self._scope is None:
            return False
        return bool(self._scope.is_scope_data_ready())

    def get_scope_data(self):
        if not USE_SCOPE or self._scope is None:
            return {}
        return self._scope.get_scope_channel_data(valid_data=False)

    def request_scope_data(self):
        if USE_SCOPE and self._scope:
            self._scope.request_scope_data()

# ─── Main GUI ────────────────────────────────────────────────────────────────
class MotorLoggerGUI:
    GUI_POLL_MS = 500        # live RPM update
    DEFAULT_DT  = 5          # ms sample interval

    def __init__(self):
        self.root = tk.Tk()
        self.root.title("pyX2Cscope – Motor Logger")

        # State -------------------------------------------------------------
        self.scope = _ScopeWrapper()
        self.connected = False
        self._cap_thread: threading.Thread | None = None
        self._stop_flag = threading.Event()
        self.data: Dict[str, List[float]] = {}
        self.scale_factors = {k: 1.0 for k in VAR_PATHS}  # per-channel scaling
        self.selected_vars = list(VAR_PATHS)
        self.enforce_limit = DEFAULT_ENFORCE_SAMPLE_LIMIT

        self._build_widgets()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._poll_job: str | None = None
        self._poll_gui()

    # ── GUI layout ─────────────────────────────────────────────────────────
    def _build_widgets(self):
        notebook = ttk.Notebook(self.root)
        notebook.pack(fill="both", expand=True)

        # 1) Connection + logger tab ---------------------------------------
        main = ttk.Frame(notebook)
        notebook.add(main, text="Logger")

        # Connection frame
        conn = ttk.LabelFrame(main, text="Connection", padding=10)
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

        # Parameters
        parms = ttk.LabelFrame(main, text="Parameters", padding=10)
        parms.pack(fill="x", padx=10, pady=4)

        self._lock_widgets: List[tk.Widget] = []  # disabled during capture

        def _row(lbl, default, r):
            ttk.Label(parms, text=lbl).grid(row=r, column=0, sticky="e")
            e = ttk.Entry(parms, width=12); e.insert(0, default)
            e.grid(row=r, column=1, padx=6, pady=2)
            self._lock_widgets.append(e)
            return e
        self.speed_entry  = _row("Speed (RPM):",      "1500", 0)
        self.scale_entry  = _row("Scale (RPM/cnt):",  "0.19913", 1)
        self.dur_entry    = _row("Log time (s):",     "5",    2)
        self.sample_entry = _row("Sample every (ms):", str(self.DEFAULT_DT), 3)
        ttk.Label(parms, text="≥1 ms total").grid(row=3, column=2, sticky="w")
        ttk.Button(parms, text="?", width=2, command=self._show_sample_info).grid(row=3, column=3, padx=(2,0))

        # Buttons
        btn = ttk.Frame(main); btn.pack(pady=(6, 2))
        self.start_btn = ttk.Button(btn, text="START ▶", width=12, command=self._start_capture, state="disabled"); self.start_btn.pack(side="left", padx=2)
        self.stop_btn  = ttk.Button(btn, text="STOP ■",  width=12, command=self._stop_capture,  state="disabled"); self.stop_btn.pack(side="left", padx=2)
        self.curr_btn  = ttk.Button(btn, text="Currents", width=10, command=self._plot_currents, state="disabled"); self.curr_btn.pack(side="left", padx=2)
        self.omega_btn = ttk.Button(btn, text="Omega",    width=10, command=self._plot_omega,    state="disabled"); self.omega_btn.pack(side="left", padx=2)
        self.save_btn  = ttk.Button(btn, text="Save…",    width=8,  command=self._save,          state="disabled"); self.save_btn.pack(side="left", padx=2)

        # Status / live RPM
        self.status = tk.StringVar(value="Idle – not connected")
        ttk.Label(main, textvariable=self.status).pack(pady=(0, 4))
        read = ttk.Frame(main); read.pack(pady=(0, 8))
        ttk.Label(read, text="Measured speed:").grid(row=0, column=0, sticky="e")
        ttk.Label(read, text="Command speed:").grid (row=1, column=0, sticky="e")
        self.meas_str = tk.StringVar(value="—"); self.cmd_str = tk.StringVar(value="—")
        ttk.Label(read, textvariable=self.meas_str, width=22, anchor="w").grid(row=0, column=1, padx=6)
        ttk.Label(read, textvariable=self.cmd_str,  width=22, anchor="w").grid(row=1, column=1, padx=6)

        # Per-channel scaling ------------------------------------------------
        ttk.Label(parms, text="Channel Scaling:").grid(row=4, column=0, columnspan=2, sticky="w", pady=(6, 2))
        tbl = ttk.Frame(parms)
        tbl.grid(row=5, column=0, columnspan=2, sticky="w")

        self.scale_vars: Dict[str, tk.StringVar] = {}
        self.var_enabled: Dict[str, tk.BooleanVar] = {}
        for r, (name, _) in enumerate(VAR_PATHS.items()):
            en = tk.BooleanVar(value=True); self.var_enabled[name] = en
            chk = ttk.Checkbutton(tbl, variable=en)
            chk.grid(row=r, column=0, padx=(0,4))
            self._lock_widgets.append(chk)
            ttk.Label(tbl, text=name, width=15).grid(row=r, column=1, sticky="e", pady=2)
            sv = tk.StringVar(value="1.0"); self.scale_vars[name] = sv
            ent = ttk.Entry(tbl, textvariable=sv, width=10)
            ent.grid(row=r, column=2, sticky="w", padx=6)
            self._lock_widgets.append(ent)

        # Experimental: toggle sample guard
        self.guard_btn = ttk.Button(
            parms,
            text="⚠ Experimental: remove mS guard",
            command=self._toggle_guard,
        )
        self.guard_btn.grid(row=6, column=0, columnspan=2, pady=(6, 2))

    # ── Helper utilities ───────────────────────────────────────────────────
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
        if fn: self.elf_path.set(fn)

    def _toggle_guard(self):
        """Toggle enforcement of the sample interval safeguard."""
        self.enforce_limit = not self.enforce_limit
        if self.enforce_limit:
            txt = "⚠ Experimental: remove mS guard"
        else:
            txt = "Re-enable mS guard"
        self.guard_btn.config(text=txt)

    def _show_sample_info(self):
        """Explain sample interval limitations."""
        messagebox.showinfo(
            "Sample rate limits",
            "Scope channels allow logging all selected variables at 1 ms.\n"
            "The GUI enforces a 1 ms minimum interval by default.\n"
            "Use the experimental button if you want to try faster rates.",
        )

    # ── Connection handling ───────────────────────────────────────────────
    def _toggle_conn(self): self._disconnect() if self.connected else self._connect()

    def _connect(self):
        port, elf = self.port_var.get(), self.elf_path.get()
        if port in ("", "-") or not elf:
            messagebox.showwarning("Missing", "Choose COM port and ELF file"); return
        if not pathlib.Path(elf).is_file():
            messagebox.showerror("File", "ELF not found"); return
        try:
            self.scope.connect(port, elf)
            # Grab variables
            self.hwui     = self.scope.get_variable(HWUI_VAR)
            self.cmd_var  = self.scope.get_variable(VEL_CMD_VAR)
            self.meas_var = self.scope.get_variable(VEL_MEAS_VAR)
            self.run_var  = self.scope.get_variable(RUN_REQ_VAR)
            self.stop_var = self.scope.get_variable(STOP_REQ_VAR)
            self.mon_vars = {k: self.scope.get_variable(p) for k, p in VAR_PATHS.items()}
            missing = [k for k, v in self.mon_vars.items() if v is None]
            if missing:
                raise RuntimeError("Symbols not in ELF:\n  • " + "\n  • ".join(missing))
            self.hwui.set_value(0)  # disable on-board HMI
        except Exception as e:
            messagebox.showerror("Connect", str(e)); self.scope.disconnect(); return
        # UI
        self.connected = True
        self.conn_btn.config(text="Disconnect"); self.start_btn.config(state="normal")
        self.status.set(f"Connected ({port})")

    def _disconnect(self):
        self._stop_capture()
        self.scope.disconnect()
        self.connected = False
        for b in (self.start_btn, self.stop_btn, self.curr_btn, self.omega_btn, self.save_btn):
            b.config(state="disabled")
        self.conn_btn.config(text="Connect"); self.status.set("Disconnected")

    # ── Capture control ───────────────────────────────────────────────────
    def _start_capture(self):
        if self._cap_thread and self._cap_thread.is_alive(): return
        try:
            rpm  = float(self.speed_entry.get())
            scale= float(self.scale_entry.get())
            dur  = float(self.dur_entry.get())
            dt_ms= float(self.sample_entry.get())
            if rpm<0 or scale<=0 or dur<=0 or dt_ms<=0: raise ValueError
        except ValueError:
            messagebox.showerror("Input", "Enter valid positive numbers"); return
        if USE_SCOPE and not self.connected:
            messagebox.showwarning("Not connected", "Connect to a target first"); return

        # Update scale factors from Scaling tab
        for k in VAR_PATHS:
            try:
                self.scale_factors[k] = float(self.scale_vars[k].get())
            except ValueError:
                self.scale_factors[k] = 1.0

        # Selected variables and prep
        self.selected_vars = [k for k, v in self.var_enabled.items() if v.get()]
        if not self.selected_vars:
            messagebox.showwarning("Variables", "Select at least one variable")
            return
        if self.enforce_limit:
            if dt_ms < MIN_DELAY_MS:
                messagebox.showwarning(
                    "Sample interval",
                    f"Minimum allowed interval is {MIN_DELAY_MS:.0f} ms",
                )
                return
        self.data = {k: [] for k in self.selected_vars}
        self.data["t"] = []
        self.data["MotorRunning"] = []  # 1 when spinning, 0 after stop command
        self._stop_flag.clear(); self.ts = dt_ms / 1000.0
        self.cmd_var.set_value(int(round(rpm/scale)))

        self._cap_thread = threading.Thread(target=self._worker, args=(dur,), daemon=True)
        self._cap_thread.start()

        self.start_btn.config(state="disabled"); self.stop_btn.config(state="normal")
        for b in (self.curr_btn, self.omega_btn, self.save_btn):
            b.config(state="disabled")
        for w in self._lock_widgets:
            w.config(state="disabled")

    def _stop_capture(self): self._stop_flag.set()

    def _worker(self, dur: float):
        """Background capture."""
        PRE_START = 0.5  # capture before sending run command [s]
        POST_STOP = 1.0  # capture after stop command [s]

        try:
            self.status.set("Running + logging…")
            self.stop_var.set_value(0)

            t0 = time.perf_counter()
            run_cmd_time = t0 + PRE_START
            stop_cmd_time = run_cmd_time + dur
            end_time = stop_cmd_time + POST_STOP

            # Configure scope for fast multi-channel capture
            vars_to_sample = [self.mon_vars[k] for k in self.selected_vars]
            self.scope.prepare_scope(vars_to_sample, int(self.ts * 1000))

            sample_idx = 0
            run_sent = False
            stop_sent = False

            while not self._stop_flag.is_set() and time.perf_counter() < end_time:
                now = time.perf_counter()

                if not run_sent and now >= run_cmd_time:
                    self.run_var.set_value(1)
                    run_sent = True
                if not stop_sent and now >= stop_cmd_time:
                    self.stop_var.set_value(1)
                    stop_sent = True

                running = run_cmd_time <= now < stop_cmd_time

                if self.scope.scope_ready():
                    chans = self.scope.get_scope_data()
                    self.scope.request_scope_data()          # <- leave here

                    if chans:
                        n = len(next(iter(chans.values())))  # all lists are equal

                        # time vector -------------------------------------
                        self.data["t"].extend(
                            [(sample_idx + i) * self.ts for i in range(n)]
                        )
                        self.data["MotorRunning"].extend(
                            [1 if running else 0] * n
                        )
                        sample_idx += n

                        for ch, vals in chans.items():
                            if not vals:
                                continue
                            key = PATH_TO_KEY.get(ch)      # ch is the full path string
                            if key is None:                # a channel we don’t care about
                                continue
                            scale = self.scale_factors[key]
                            self.data[key].extend(v * scale for v in vals)


                time.sleep(self.ts / 10)
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
            t_len = len(self.data["t"])
            for k in list(self.data.keys()):
                if k == "t":
                    continue
                if isinstance(self.data[k], list) and len(self.data[k]) != t_len:
                    self.data[k] = self.data[k][:t_len]

            if any(k in self.data for k in ("idqCmd_q", "Idq_q", "Idq_d")):
                self.curr_btn.config(state="normal")
            else:
                self.curr_btn.config(state="disabled")

            if any(k in self.data for k in ("OmegaElectrical", "OmegaCmd")):
                self.omega_btn.config(state="normal")
            else:
                self.omega_btn.config(state="disabled")

            self.save_btn.config(state="normal")
            self.status.set("Capture finished")
        else:
            self.status.set("Stopped / no data")

        for w in self._lock_widgets:
            w.config(state="normal")

    # ── Live RPM polling ─────────────────────────────────────────────────
    def _poll_gui(self):
        try:
            if not self.root.winfo_exists():
                return
        except tk.TclError:
            return
        if self._cap_thread and self._cap_thread.is_alive():
            self._poll_job = self.root.after(self.GUI_POLL_MS, self._poll_gui); return
        if self.connected:
            try:
                cnt_meas = self.meas_var.get_value(); cnt_cmd = self.cmd_var.get_value()
                scale = float(self.scale_entry.get())
                self.meas_str.set(f"{cnt_meas*scale:+.0f} RPM ({cnt_meas})")
                self.cmd_str .set(f"{cnt_cmd *scale:+.0f} RPM ({cnt_cmd})")
            except Exception:
                self.meas_str.set("—"); self.cmd_str.set("—")
        else:
            self.meas_str.set("—"); self.cmd_str.set("—")
        self._poll_job = self.root.after(self.GUI_POLL_MS, self._poll_gui)

    # ── Plot & save ──────────────────────────────────────────────────────
    def _plot_currents(self):
        if not self.data["t"]:
            messagebox.showinfo("No data", "Nothing captured yet"); return
        if plt is None:
            messagebox.showerror("Plot", "Install matplotlib"); return
        fig, ax = plt.subplots(figsize=(8, 4))
        t = self.data["t"]
        plotted = False
        for k, lbl in (
            ("idqCmd_q", "idqCmd.q [A]"),
            ("Idq_q",    "idq.q [A]"),
            ("Idq_d",    "idq.d [A]"),
        ):
            if (
                k in self.data
                and self.data[k]            # not empty
                and len(self.data[k]) == len(t)
            ):
                ax.plot(t, self.data[k], label=lbl, linewidth=0.9)
                plotted = True

        if not plotted:
            messagebox.showinfo("Plot", "No valid current data to plot."); return
        ax.set_xlabel("Time [s]")
        ax.set_ylabel("Current [scaled]")
        ax.grid(True, linestyle=":", linewidth=0.5)
        ax.legend(fontsize="small")
        win = tk.Toplevel(self.root); win.title("Current traces")
        FigureCanvasTkAgg(fig, master=win).get_tk_widget().pack(fill="both", expand=True)
        fig.tight_layout()

    def _plot_omega(self):
        if not self.data["t"]:
            messagebox.showinfo("No data", "Nothing captured yet"); return
        if plt is None:
            messagebox.showerror("Plot", "Install matplotlib"); return
        fig, ax = plt.subplots(figsize=(8, 4))
        t = self.data["t"]
        plotted = False
        for k, lbl in (
            ("OmegaElectrical", "omegaElectrical [RPM]"),
            ("OmegaCmd",        "omegaCmd [RPM]"),
        ):
            if (
                k in self.data
                and self.data[k]            # not empty
                and len(self.data[k]) == len(t)
            ):
                ax.plot(t, self.data[k], label=lbl, linewidth=0.9)
                plotted = True

        if not plotted:
            messagebox.showinfo("Plot", "No valid omega data to plot."); return
        ax.set_xlabel("Time [s]")
        ax.set_ylabel("Omega [scaled]")
        ax.grid(True, linestyle=":", linewidth=0.5)
        ax.legend(fontsize="small")
        win = tk.Toplevel(self.root); win.title("Omega traces")
        FigureCanvasTkAgg(fig, master=win).get_tk_widget().pack(fill="both", expand=True)
        fig.tight_layout()

    def _save(self):
        if not self.data["t"]:
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

    # ── Cleanup ──────────────────────────────────────────────────────────
    def _on_close(self):
        try:
            self._stop_flag.set()
            if self._poll_job is not None:
                try:
                    self.root.after_cancel(self._poll_job)
                except Exception:
                    pass
            if self._cap_thread and self._cap_thread.is_alive():
                self._cap_thread.join(timeout=2)
            self.scope.disconnect()
        finally:
            try:
                if self.root.winfo_exists():
                    self.root.destroy()
            except tk.TclError:
                pass

# ─── Entry point ─────────────────────────────────────────────────────────────
if __name__ == "__main__":
    gui = MotorLoggerGUI()
    try:
        gui.root.mainloop()
    except KeyboardInterrupt:
        gui._on_close()
