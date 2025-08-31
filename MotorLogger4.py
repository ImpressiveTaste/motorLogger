#!/usr/bin/env python3
"""Run motor for 10 s and log scope data using pyX2Cscope.

This standalone script connects to a target via ``pyX2Cscope``, starts the
motor for exactly ten seconds while streaming scope channels as fast as
possible, then stops the motor and stores the captured samples in
``scope_data.csv``.

It expects only the standard library plus ``pyserial`` and ``pyx2cscope``
(plus optional ``matplotlib`` for plotting).  No other files are required.
"""
from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional

import serial.tools.list_ports
from pyx2cscope.x2cscope import X2CScope

try:  # optional plotting
    import matplotlib.pyplot as plt
except Exception:  # pragma: no cover - plotting is optional
    plt = None

# ---------------------------------------------------------------------------
# Variable definitions
MONITOR_VARS = [
    ("idqCmd_q", "motor.idqCmd.q"),
    ("Idq_q", "motor.idq.q"),
    ("Idq_d", "motor.idq.d"),
    ("OmegaElectrical", "motor.omegaElectrical"),
    ("OmegaCmd", "motor.omegaCmd"),
]

CONTROL_VARS = {
    "hardware_ui": "app.hardwareUiEnabled",
    "velocity_ref": "motor.apiData.velocityReference",
    "run_req": "motor.apiData.runMotorRequest",
    "stop_req": "motor.apiData.stopMotorRequest",
}

# ---------------------------------------------------------------------------

def list_ports() -> List[str]:
    """Return a list of available serial ports."""
    return [p.device for p in serial.tools.list_ports.comports()]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run motor for 10 s and log data")
    parser.add_argument("--elf", help="Path to ELF file")
    parser.add_argument("--port", help="Serial port (auto if not supplied)")
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        choices=[115200, 230400, 460800, 921600],
        help="Baud rate",
    )
    parser.add_argument("--speed", type=float, help="Speed in RPM")
    parser.add_argument("--scale", type=float, help="RPM per count")
    args = parser.parse_args()

    if not args.port:
        ports = list_ports()
        if ports:
            print("Available COM ports:")
            for p in ports:
                print(f"  {p}")
            args.port = ports[0]
            print(f"Using port: {args.port}")
        else:
            print("No serial ports found.")
            args.port = input("Enter COM port: ").strip()

    if not args.elf:
        args.elf = input("ELF file path: ").strip()
    if not args.speed:
        args.speed = float(input("Speed (RPM): "))
    if not args.scale:
        args.scale = float(input("Scale (RPM per count): "))

    return args


# ---------------------------------------------------------------------------

def run_motor_logger(elf: str, port: str, baud: int, speed: float, scale: float) -> None:
    """Run the logger with provided settings.

    This is the core routine used by both the CLI and the Tkinter GUI.
    """
    counts = speed / scale

    scope: Optional[X2CScope] = None
    run_sent = False
    stop_sent = False
    var_handles: Dict[str, Optional[int]] = {}

    try:
        scope = X2CScope(port=port, baudrate=baud, elf_file=elf)
        try:  # Older versions require explicit import
            scope.import_variables(elf)
        except Exception:
            pass

        # Resolve variable handles ------------------------------------------------
        active_channels: List[tuple[str, int]] = []
        for label, path in MONITOR_VARS:
            try:
                handle = scope.get_variable(path)
                var_handles[label] = handle
                active_channels.append((label, handle))
            except Exception as exc:  # pragma: no cover - hardware dependent
                print(f"Warning: cannot resolve {path}: {exc}")
                var_handles[label] = None

        for key, path in CONTROL_VARS.items():
            try:
                var_handles[key] = scope.get_variable(path)
            except Exception as exc:  # pragma: no cover - hardware dependent
                print(f"Warning: cannot resolve {path}: {exc}")
                var_handles[key] = None

        # Initial writes ---------------------------------------------------------
        hw = var_handles.get("hardware_ui")
        if hw is not None:
            try:
                scope.write(hw, 0)
            except Exception as exc:
                print(f"Warning: failed to write hardwareUiEnabled=0: {exc}")

        vel = var_handles.get("velocity_ref")
        if vel is not None:
            try:
                scope.write(vel, int(counts))
            except Exception as exc:
                print(f"Warning: failed to write velocityReference: {exc}")

        # Scope setup ------------------------------------------------------------
        if hasattr(scope, "clear_scope_channels"):
            scope.clear_scope_channels()
        for label, handle in active_channels:
            scope.add_scope_channel(handle)

        scope.set_sample_time(1)
        scope.request_scope_data()

        time.sleep(0.5)

        # Run motor --------------------------------------------------------------
        run_var = var_handles.get("run_req")
        if run_var is not None:
            scope.write(run_var, 1)
            run_sent = True

        start = time.time()
        data: Dict[str, List[float]] = {lbl: [] for lbl, _ in active_channels}

        while time.time() - start < 10.0:
            if scope.is_scope_data_ready():
                chunk = scope.get_scope_channel_data(valid_data=False)
                for idx, (lbl, _) in enumerate(active_channels):
                    if idx in chunk:
                        data[lbl].extend(chunk[idx])
                scope.request_scope_data()
            time.sleep(0.05)

        # Stop motor -------------------------------------------------------------
        stop_var = var_handles.get("stop_req")
        if stop_var is not None:
            scope.write(stop_var, 1)
            stop_sent = True

        time.sleep(0.5)
        if scope.is_scope_data_ready():
            chunk = scope.get_scope_channel_data(valid_data=False)
            for idx, (lbl, _) in enumerate(active_channels):
                if idx in chunk:
                    data[lbl].extend(chunk[idx])

        # Build time axis --------------------------------------------------------
        n_samples = min(len(v) for v in data.values()) if data else 0
        ts_us = 0
        try:
            ts_us = scope.get_scope_sample_time()
        except Exception:
            pass
        if ts_us and ts_us > 0:
            ts_s = ts_us / 1_000_000.0
        else:
            ts_s = 10.0 / n_samples if n_samples else 0.0
            print("Warning: scope sample time unavailable; estimating from run duration.")
        fs = 1.0 / ts_s if ts_s else 0.0
        t_axis = [i * ts_s for i in range(n_samples)]

        # Save CSV ---------------------------------------------------------------
        csv_path = Path("scope_data.csv")
        with csv_path.open("w", newline="") as f:
            writer = csv.writer(f)
            header = ["t_s"] + [lbl for lbl, _ in active_channels]
            writer.writerow(header)
            for i in range(n_samples):
                row = [t_axis[i]] + [data[lbl][i] for lbl, _ in active_channels]
                writer.writerow(row)

        print(f"Captured {n_samples} samples per channel.")
        print(f"Estimated sample time {ts_s:.6f} s ({fs:.1f} Hz).")
        print(f"CSV saved to {csv_path.resolve()}")

        # Optional plot ---------------------------------------------------------
        if plt and n_samples:
            fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
            if "idqCmd_q" in data:
                ax1.plot(t_axis, data["idqCmd_q"], label="idqCmd_q")
            if "Idq_q" in data:
                ax1.plot(t_axis, data["Idq_q"], label="Idq_q")
            if "Idq_d" in data:
                ax1.plot(t_axis, data["Idq_d"], label="Idq_d")
            ax1.set_ylabel("Currents")
            ax1.legend()

            if "OmegaElectrical" in data:
                ax2.plot(t_axis, data["OmegaElectrical"], label="OmegaElectrical")
            if "OmegaCmd" in data:
                ax2.plot(t_axis, data["OmegaCmd"], label="OmegaCmd")
            ax2.set_ylabel("Omega")
            ax2.set_xlabel("Time [s]")
            ax2.legend()
            plt.show()

    except KeyboardInterrupt:
        print("Interrupted by user.")
    except Exception as exc:
        print(f"Error: {exc}")
    finally:
        if scope is not None:
            if run_sent and not stop_sent:
                stop_var = var_handles.get("stop_req")
                if stop_var is not None:
                    try:
                        scope.write(stop_var, 1)
                        print("Motor stop requested due to early exit.")
                    except Exception:
                        pass
            scope.close()


def main() -> None:
    args = parse_args()
    run_motor_logger(args.elf, args.port, args.baud, args.speed, args.scale)


if __name__ == "__main__":
    main()
