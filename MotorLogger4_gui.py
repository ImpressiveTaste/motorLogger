#!/usr/bin/env python3
"""Simple Tkinter GUI wrapper for MotorLogger4."""

from __future__ import annotations

import io
import threading
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from contextlib import redirect_stdout, redirect_stderr

from MotorLogger4 import list_ports, run_motor_logger


class MotorLoggerApp(tk.Tk):
    """Tkinter front end for :mod:`MotorLogger4`."""

    def __init__(self) -> None:
        super().__init__()
        self.title("MotorLogger4 GUI")
        self._build_widgets()

    # ------------------------------------------------------------------
    def _build_widgets(self) -> None:
        row = 0
        ttk.Label(self, text="ELF Path:").grid(row=row, column=0, sticky="e", padx=5, pady=2)
        self.elf_var = tk.StringVar()
        ttk.Entry(self, textvariable=self.elf_var, width=40).grid(row=row, column=1, padx=5, pady=2)
        ttk.Button(self, text="Browse", command=self._browse_elf).grid(row=row, column=2, padx=5, pady=2)

        row += 1
        ttk.Label(self, text="Port:").grid(row=row, column=0, sticky="e", padx=5, pady=2)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(self, textvariable=self.port_var, values=list_ports())
        self.port_combo.grid(row=row, column=1, padx=5, pady=2, sticky="we")
        ttk.Button(self, text="Refresh", command=self._refresh_ports).grid(row=row, column=2, padx=5, pady=2)
        self._refresh_ports()

        row += 1
        ttk.Label(self, text="Baud:").grid(row=row, column=0, sticky="e", padx=5, pady=2)
        self.baud_var = tk.IntVar(value=115200)
        ttk.Combobox(
            self,
            textvariable=self.baud_var,
            values=[115200, 230400, 460800, 921600],
            width=10,
        ).grid(row=row, column=1, padx=5, pady=2, sticky="w")

        row += 1
        ttk.Label(self, text="Speed (RPM):").grid(row=row, column=0, sticky="e", padx=5, pady=2)
        self.speed_var = tk.DoubleVar(value=0.0)
        ttk.Entry(self, textvariable=self.speed_var, width=15).grid(row=row, column=1, padx=5, pady=2, sticky="w")

        row += 1
        ttk.Label(self, text="Scale (RPM/count):").grid(row=row, column=0, sticky="e", padx=5, pady=2)
        self.scale_var = tk.DoubleVar(value=1.0)
        ttk.Entry(self, textvariable=self.scale_var, width=15).grid(row=row, column=1, padx=5, pady=2, sticky="w")

        row += 1
        self.run_btn = ttk.Button(self, text="Run", command=self._start_run)
        self.run_btn.grid(row=row, column=0, columnspan=3, pady=5)

        row += 1
        self.output = tk.Text(self, width=60, height=15)
        self.output.grid(row=row, column=0, columnspan=3, padx=5, pady=5)

    # ------------------------------------------------------------------
    def _browse_elf(self) -> None:
        path = filedialog.askopenfilename(filetypes=[("ELF files", "*.elf"), ("All files", "*.*")])
        if path:
            self.elf_var.set(path)

    def _refresh_ports(self) -> None:
        ports = list_ports()
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    # ------------------------------------------------------------------
    def _start_run(self) -> None:
        elf = self.elf_var.get().strip()
        port = self.port_var.get().strip()
        baud = int(self.baud_var.get())
        speed = float(self.speed_var.get())
        scale = float(self.scale_var.get())
        if not elf or not port:
            messagebox.showerror("Error", "ELF path and port are required")
            return
        self.run_btn.config(state=tk.DISABLED)
        self.output.delete("1.0", tk.END)

        def worker() -> None:
            buf = io.StringIO()
            try:
                with redirect_stdout(buf), redirect_stderr(buf):
                    run_motor_logger(elf, port, baud, speed, scale)
            except Exception as exc:  # pragma: no cover - hardware dependent
                buf.write(f"Error: {exc}\n")
            finally:
                text = buf.getvalue()
                self.after(0, self._finish_run, text)

        threading.Thread(target=worker, daemon=True).start()

    def _finish_run(self, text: str) -> None:
        self.output.insert(tk.END, text)
        self.run_btn.config(state=tk.NORMAL)


if __name__ == "__main__":
    app = MotorLoggerApp()
    app.mainloop()
