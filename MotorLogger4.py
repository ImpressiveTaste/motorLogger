#!/usr/bin/env python3
"""Tkinter GUI motor logger using pyX2Cscope.

This module provides a minimal graphical interface for capturing data from
``pyX2Cscope``.  It is based on the command line version of MotorLogger4 but
replaces all prompts with a simple GUI that allows the user to:

* select the serial port and ELF file
* start a capture and see the live plot of all channels
* automatically store the captured samples in ``scope_data.csv``

The example is intentionally compact and only demonstrates the basic technique
for acquiring and plotting data from the scope interface.  It can serve as a
starting point for richer applications.
"""

from __future__ import annotations

import csv
import logging
import threading
import time
from pathlib import Path
from typing import Dict, List

import serial.tools.list_ports
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from pyx2cscope.x2cscope import X2CScope


class MotorLoggerGUI:
    """Simple Tkinter front end for ``pyX2Cscope`` logging."""

    def __init__(self) -> None:
        logging.basicConfig(level=logging.INFO, filename=__file__ + ".log")

        self.root = tk.Tk()
        self.root.title("Motor Logger 4")

        self.port_var = tk.StringVar()
        self.elf_var = tk.StringVar()

        self._build_widgets()

        self.figure = Figure(figsize=(6, 4))
        self.ax = self.figure.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.figure, master=self.root)
        self.canvas.get_tk_widget().pack(fill="both", expand=True, padx=10, pady=10)

        # Storage for latest capture
        self.data_storage: Dict[int, List[float]] = {}

    # ------------------------------------------------------------------ UI ----
    def _build_widgets(self) -> None:
        """Create and lay out the widgets."""

        frame = ttk.Frame(self.root, padding=10)
        frame.pack(fill="x")

        # Port selection
        port_frame = ttk.Frame(frame)
        port_frame.pack(fill="x", pady=2)
        ttk.Label(port_frame, text="COM Port:").pack(side="left")
        self.port_combo = ttk.Combobox(port_frame, textvariable=self.port_var, width=20)
        self._refresh_ports()
        self.port_combo.pack(side="left", padx=5)
        ttk.Button(port_frame, text="Refresh", command=self._refresh_ports).pack(
            side="left"
        )

        # ELF file selector
        elf_frame = ttk.Frame(frame)
        elf_frame.pack(fill="x", pady=2)
        ttk.Label(elf_frame, text="ELF File:").pack(side="left")
        ttk.Entry(elf_frame, textvariable=self.elf_var, width=40).pack(
            side="left", padx=5, fill="x", expand=True
        )
        ttk.Button(elf_frame, text="Browse", command=self._browse_elf).pack(
            side="left"
        )

        # Start button
        self.start_btn = ttk.Button(frame, text="Start Capture", command=self.start)
        self.start_btn.pack(pady=(8, 2))

    def _refresh_ports(self) -> None:
        """Refresh the list of serial ports in the combobox."""

        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if ports:
            self.port_combo.current(0)

    def _browse_elf(self) -> None:
        """Open a file dialog to select the ELF file."""

        path = filedialog.askopenfilename(title="Select ELF file")
        if path:
            self.elf_var.set(path)

    # -------------------------------------------------------------- Capture ----
    def start(self) -> None:
        """Validate inputs and start the capture thread."""

        port = self.port_var.get().strip()
        elf_path = self.elf_var.get().strip()

        if not port:
            messagebox.showerror("Motor Logger", "Please select a serial port")
            return
        if not elf_path:
            messagebox.showerror("Motor Logger", "Please choose an ELF file")
            return
        if not Path(elf_path).exists():
            messagebox.showerror("Motor Logger", f"ELF file not found: {elf_path}")
            return

        self.start_btn.config(state="disabled")
        thread = threading.Thread(target=self._capture_loop, args=(port, elf_path), daemon=True)
        thread.start()

    def _capture_loop(self, port: str, elf_file: str) -> None:
        """Worker thread that performs the data capture."""

        try:
            x2c_scope = X2CScope(port=port, elf_file=elf_file)

            variables: List[str] = [
                "motor.idq.q",
                "motor.vabc.a",
                "motor.vabc.b",
                "motor.vabc.c",
                "motor.apiData.velocityMeasured",
            ]
            for var in variables:
                x2c_scope.add_scope_channel(x2c_scope.get_variable(var))

            x2c_scope.set_sample_time(1)
            x2c_scope.request_scope_data()

            sample_count = 0
            max_sample = 100

            while sample_count < max_sample:
                if x2c_scope.is_scope_data_ready():
                    sample_count += 1
                    logging.info("Scope data is ready.")

                    self.data_storage = {
                        ch: data
                        for ch, data in x2c_scope.get_scope_channel_data(
                            valid_data=False
                        ).items()
                    }

                    self.root.after(0, self._update_plot, self.data_storage)

                    if sample_count >= max_sample:
                        break
                    x2c_scope.request_scope_data()

                time.sleep(0.1)

            if self.data_storage:
                self._save_csv(self.data_storage)
                self.root.after(
                    0,
                    lambda: messagebox.showinfo(
                        "Motor Logger", "Data collection complete. Saved to scope_data.csv"
                    ),
                )
        except Exception as exc:  # pragma: no cover - runtime guard
            logging.error("Error in capture loop: %s", exc)
            self.root.after(0, lambda: messagebox.showerror("Motor Logger", str(exc)))
        finally:
            self.root.after(0, lambda: self.start_btn.config(state="normal"))

    def _update_plot(self, data_storage: Dict[int, List[float]]) -> None:
        """Update the matplotlib plot with new data."""

        self.ax.clear()
        for channel, data in data_storage.items():
            time_values = [i * 0.001 for i in range(len(data))]
            self.ax.plot(time_values, data, label=f"Channel {channel}")

        self.ax.set_xlabel("Time (ms)")
        self.ax.set_ylabel("Value")
        self.ax.set_title("Live Plot of Byte Data")
        self.ax.legend()

        self.canvas.draw()

    def _save_csv(self, data_storage: Dict[int, List[float]]) -> None:
        """Save the captured data to ``scope_data.csv``."""

        csv_file_path = "scope_data.csv"
        max_length = max(len(data) for data in data_storage.values())

        with open(csv_file_path, mode="w", newline="") as file:
            writer = csv.DictWriter(file, fieldnames=data_storage.keys())
            writer.writeheader()
            for i in range(max_length):
                row = {
                    channel: (
                        data_storage[channel][i] if i < len(data_storage[channel]) else None
                    )
                    for channel in data_storage
                }
                writer.writerow(row)

        logging.info("Data saved in %s", csv_file_path)

    # ----------------------------------------------------------------- main ----
    def run(self) -> None:
        """Start the Tkinter event loop."""

        self.root.mainloop()


def main() -> None:
    """Entry point that launches the GUI."""

    gui = MotorLoggerGUI()
    gui.run()


if __name__ == "__main__":
    main()

