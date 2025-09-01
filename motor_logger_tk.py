#!/usr/bin/env python3
"""Simple Tkinter GUI for motor logging via X2CScope.

This example connects to an X2Cscope enabled target, logs a fixed set of
five variables and displays a snippet of the captured samples.  The scope
sample-time factor ``f`` is configured by the user.  For five variables the
rough capture duration (in milliseconds) follows the linear relation:

``total_ms ≈ 24 + 24.5 × (f - 1)``

where ``f = 1`` → ``24`` ms, ``f = 100`` → ``2450`` ms and ``f = 250`` →
``6125`` ms.

The application falls back to synthetic data when the X2CScope dependencies
are not available.  It is intentionally minimal and meant as a starting
point for further experimentation.
"""

from __future__ import annotations

import random
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from typing import Dict, List

try:  # Optional hardware dependencies
    from X2Cscope import X2CScope
    from mchplnet.interfaces.factory import InterfaceType
except Exception:  # pragma: no cover - allow running without hardware libs
    X2CScope = None
    InterfaceType = None

# Variable paths captured during logging
VAR_PATHS: List[str] = [
    "motor.idqCmd.q",
    "motor.idq.q",
    "motor.idq.d",
    "motor.omegaElectrical",
    "motor.omegaCmd",
]


def estimate_total_time_ms(factor: int, num_vars: int = 5) -> float:
    """Estimate total capture time for the given sample-time factor.

    The numbers are derived from empirical measurements for five variables:
    ``f = 1 → 24 ms``, ``f = 100 → 2450 ms`` and ``f = 250 → 6125 ms``.
    """

    base = 24.0
    step = 24.5
    return base + step * (factor - 1)


class MotorLoggerApp:
    """Tkinter frontend for basic motor logging."""

    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Tk Motor Logger")
        self.scope: X2CScope | None = None
        self._build_gui()

    # ------------------------------------------------------------------ GUI
    def _build_gui(self) -> None:
        frame = ttk.Frame(self.root, padding=10)
        frame.pack(fill="both", expand=True)

        conn = ttk.LabelFrame(frame, text="Connection")
        conn.pack(fill="x", pady=5)
        ttk.Label(conn, text="Port:").grid(row=0, column=0, sticky="w")
        self.port_var = tk.StringVar()
        ttk.Entry(conn, textvariable=self.port_var, width=15).grid(
            row=0, column=1, sticky="w"
        )
        ttk.Label(conn, text="ELF file:").grid(row=1, column=0, sticky="w")
        self.elf_var = tk.StringVar()
        ttk.Entry(conn, textvariable=self.elf_var, width=30).grid(
            row=1, column=1, sticky="we"
        )
        ttk.Button(conn, text="Browse", command=self._browse_elf).grid(
            row=1, column=2, padx=5
        )

        log = ttk.LabelFrame(frame, text="Logging")
        log.pack(fill="x", pady=5)
        ttk.Label(log, text="Sample factor:").grid(row=0, column=0, sticky="w")
        self.factor_var = tk.IntVar(value=1)
        spin = ttk.Spinbox(
            log,
            from_=1,
            to=1000,
            textvariable=self.factor_var,
            width=7,
            command=self._update_time,
        )
        spin.grid(row=0, column=1, sticky="w")
        spin.bind("<KeyRelease>", self._update_time)

        ttk.Label(log, text="Est. total time (ms):").grid(
            row=1, column=0, sticky="w"
        )
        self.time_lbl = ttk.Label(log, text="24")
        self.time_lbl.grid(row=1, column=1, sticky="w")

        self.start_btn = ttk.Button(log, text="Start", command=self.start_logging)
        self.start_btn.grid(row=2, column=0, pady=5)
        self.stop_btn = ttk.Button(
            log, text="Stop", command=self.stop_logging, state=tk.DISABLED
        )
        self.stop_btn.grid(row=2, column=1, pady=5)

        self.output = tk.Text(frame, height=10)
        self.output.pack(fill="both", expand=True, pady=5)

    # ----------------------------------------------------------------- Helpers
    def _browse_elf(self) -> None:
        path = filedialog.askopenfilename(filetypes=[("ELF", "*.elf"), ("All", "*.*")])
        if path:
            self.elf_var.set(path)

    def _update_time(self, event=None) -> None:
        f = max(1, int(self.factor_var.get() or 1))
        total = estimate_total_time_ms(f)
        self.time_lbl.config(text=f"{total:.0f}")

    def _connect(self) -> bool:
        if X2CScope is None:
            messagebox.showerror(
                "Dependency missing", "X2CScope or its dependencies are not installed."
            )
            return False
        if self.scope:
            return True
        port = self.port_var.get() or None
        elf = self.elf_var.get() or None
        try:
            if InterfaceType is not None:
                self.scope = X2CScope(
                    elf_file=elf, interface=InterfaceType.SERIAL, port=port
                )
            else:  # pragma: no cover - should not happen if X2CScope imported
                self.scope = X2CScope(elf_file=elf, port=port)
            self.scope.connect()
            if elf:
                self.scope.import_variables(elf)
            return True
        except Exception as exc:
            messagebox.showerror("Connection error", str(exc))
            self.scope = None
            return False

    # ----------------------------------------------------------------- Logging
    def start_logging(self) -> None:
        if not self._connect():
            return
        self.start_btn.config(state=tk.DISABLED)
        self.stop_btn.config(state=tk.NORMAL)
        thread = threading.Thread(target=self._capture_thread, daemon=True)
        thread.start()

    def stop_logging(self) -> None:
        if self.scope:
            try:
                self.scope.disconnect()
            except Exception:
                pass
            self.scope = None
        self.start_btn.config(state=tk.NORMAL)
        self.stop_btn.config(state=tk.DISABLED)

    def _capture_thread(self) -> None:
        try:
            variables = [self.scope.get_variable(name) for name in VAR_PATHS]
            self.scope.clear_all_scope_channel()
            for var in variables:
                self.scope.add_scope_channel(var)
            factor = self.factor_var.get()
            self.scope.set_sample_time(factor)
            self.scope.request_scope_data()
            while not self.scope.is_scope_data_ready():
                time.sleep(0.01)
            data = self.scope.get_scope_channel_data(valid_data=True)
        except Exception:  # pragma: no cover - demo mode
            data = {name: [random.random() for _ in range(10)] for name in VAR_PATHS}
        self.root.after(0, self._display_data, data)

    def _display_data(self, data: Dict[str, List[float]]) -> None:
        self.output.delete("1.0", tk.END)
        for name, values in data.items():
            snippet = ", ".join(f"{v:.3f}" for v in values[:5])
            self.output.insert(tk.END, f"{name}: {snippet}...\n")
        self.stop_logging()


def main() -> None:
    root = tk.Tk()
    app = MotorLoggerApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
