#!/usr/bin/env python3
"""Simple motor logger using pyX2Cscope.

This script is derived from ``MotorLogger2.py`` but strips the GUI and
scaling features in favour of a lightweight command line logger.  It
illustrates the data acquisition technique demonstrated in the
pyX2Cscope "Live Scope and saving data to CSV file" example: data from a
set of variables is fetched using the scope interface, plotted live and
finally written to a CSV file.

The user is prompted for the serial port and ELF file path on start-up.
"""

from __future__ import annotations

import csv
import logging
import time
from pathlib import Path
from typing import Dict, List

import serial.tools.list_ports
import matplotlib.pyplot as plt
from pyx2cscope.x2cscope import X2CScope


# ----------------------------------------------------------------------------
# Helper utilities (replacing ``examples.utils``)
# ----------------------------------------------------------------------------

def get_com_port() -> str:
    """Prompt the user to select a serial port."""
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        raise RuntimeError("No serial ports found")
    for idx, p in enumerate(ports):
        print(f"{idx}: {p.device} - {p.description}")
    try:
        selection = int(input("Select COM port index: "))
    except ValueError:
        selection = 0
    selection = max(0, min(selection, len(ports) - 1))
    return ports[selection].device


def get_elf_file_path() -> str:
    """Ask the user for the path to the application ELF file."""
    path = input("Enter path to ELF file: ").strip()
    if not path:
        raise RuntimeError("ELF file path is required")
    elf = Path(path)
    if not elf.exists():
        raise FileNotFoundError(f"ELF file not found: {elf}")
    return str(elf)


# ----------------------------------------------------------------------------
# Main capture routine
# ----------------------------------------------------------------------------

def main() -> None:
    # Set up logging
    logging.basicConfig(level=logging.INFO, filename=__file__ + ".log")

    # X2C Scope Set up
    elf_file = get_elf_file_path()
    x2c_scope = X2CScope(port=get_com_port(), elf_file=elf_file)

    # Define variables (up to 8 may be selected)
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

    # Create the plot
    plt.ion()
    fig, ax = plt.subplots()

    # Main loop
    sample_count = 0
    max_sample = 100
    data_storage: Dict[int, List[float]] = {}

    try:
        while sample_count < max_sample:
            if x2c_scope.is_scope_data_ready():
                sample_count += 1
                logging.info("Scope data is ready.")

                data_storage = {}
                for channel, data in x2c_scope.get_scope_channel_data(
                    valid_data=False
                ).items():
                    data_storage[channel] = data

                ax.clear()
                for channel, data in data_storage.items():
                    time_values = [i * 0.001 for i in range(len(data))]
                    ax.plot(time_values, data, label=f"Channel {channel}")

                ax.set_xlabel("Time (ms)")
                ax.set_ylabel("Value")
                ax.set_title("Live Plot of Byte Data")
                ax.legend()

                plt.pause(0.001)

                if sample_count >= max_sample:
                    break
                x2c_scope.request_scope_data()

            time.sleep(0.1)

    except Exception as exc:  # pragma: no cover - runtime guard
        logging.error("Error in main loop: %s", exc)
    finally:
        plt.ioff()
        plt.show()

    logging.info("Data collection complete.")

    # Data Storage
    if data_storage:
        csv_file_path = "scope_data.csv"
        max_length = max(len(data) for data in data_storage.values())

        with open(csv_file_path, mode="w", newline="") as file:
            writer = csv.DictWriter(file, fieldnames=data_storage.keys())
            writer.writeheader()
            for i in range(max_length):
                row = {
                    channel: (
                        data_storage[channel][i]
                        if i < len(data_storage[channel])
                        else None
                    )
                    for channel in data_storage
                }
                writer.writerow(row)

        logging.info("Data saved in %s", csv_file_path)


if __name__ == "__main__":
    main()
