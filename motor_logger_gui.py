# Motor Logger GUI with Sample Rate and Feasibility Checks
# Single-file application as requested.

from __future__ import annotations

import sys
import time
import math
from dataclasses import dataclass
from typing import List, Dict, Optional

import numpy as np

from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import (
    QApplication, QWidget, QTabWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QGroupBox, QLabel, QLineEdit, QPushButton, QComboBox, QSpinBox,
    QDoubleSpinBox, QTextEdit, QTableWidget, QTableWidgetItem,
    QFileDialog, QMessageBox, QHeaderView, QStatusBar
)

from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

try:
    from pyx2cscope import X2CScope, UC_WIDTH_16BIT, UC_WIDTH_32BIT
except Exception:  # pragma: no cover - fallback when library missing
    X2CScope = None
    UC_WIDTH_16BIT = 2
    UC_WIDTH_32BIT = 4

LOOP_HZ = 20_000  # fixed PWM / control loop frequency


# ----------------------------- Backend Interface -----------------------------

class ScopeBackend:
    """Abstract interface for scope backends."""

    def connect(self, port: Optional[str], elf_path: Optional[str]) -> None:
        raise NotImplementedError

    def configure_channels(self, names: List[str], bytes_per_var: List[int]) -> None:
        raise NotImplementedError

    def set_sample_time_factor(self, f: int) -> None:
        raise NotImplementedError

    def write_var(self, name: str, value: float | int) -> None:
        raise NotImplementedError

    def request_scope_data(self, duration_s: float) -> None:
        raise NotImplementedError

    def is_scope_ready(self) -> bool:
        raise NotImplementedError

    def get_data(self) -> Dict[str, np.ndarray]:
        raise NotImplementedError

    def get_device_info(self) -> Dict:
        return {}

    def estimate_buffer(self) -> Optional[int]:
        return None


class DemoBackend(ScopeBackend):
    """Synthetic data generator used when pyX2Cscope is unavailable."""

    def __init__(self) -> None:
        self.names: List[str] = []
        self.bytes_per_var: List[int] = []
        self.f: int = 1
        self.duration_s: float = 0.0
        self.ready: bool = False

    def connect(self, port: Optional[str], elf_path: Optional[str]) -> None:
        self.ready = False

    def configure_channels(self, names: List[str], bytes_per_var: List[int]) -> None:
        self.names = names
        self.bytes_per_var = bytes_per_var

    def set_sample_time_factor(self, f: int) -> None:
        self.f = max(1, int(f))

    def write_var(self, name: str, value: float | int) -> None:
        # Demo backend ignores writes.
        pass

    def request_scope_data(self, duration_s: float) -> None:
        self.duration_s = duration_s
        # In real hardware this would start capture. Demo is immediate.
        self.ready = True

    def is_scope_ready(self) -> bool:
        return self.ready

    def get_data(self) -> Dict[str, np.ndarray]:
        Fs = LOOP_HZ / self.f
        N = int(round(self.duration_s * Fs))
        t = np.arange(N) / Fs
        data: Dict[str, np.ndarray] = {}
        for name in self.names:
            if "omega" in name:
                signal = 10*np.sin(2 * np.pi * 1.0 * t) + 0.5*np.random.randn(N)
            elif "idq" in name:
                signal = 0.5*np.sin(2 * np.pi * 5.0 * t) + 0.1*np.random.randn(N)
            else:
                signal = 0.1*np.random.randn(N)
            data[name] = signal.astype(float)
        return data

    def get_device_info(self) -> Dict:
        return {"mode": "demo"}

    def estimate_buffer(self) -> Optional[int]:
        # Assume 32 kB internal buffer for demo
        return 32 * 1024


class RealScopeBackend(ScopeBackend):
    """Backend using pyX2Cscope. Falls back to demo if library missing."""

    def __init__(self) -> None:
        if X2CScope is None:
            raise RuntimeError("pyX2Cscope not available")
        self.scope = X2CScope()
        self.channel_names: List[str] = []
        self.channel_widths: List[int] = []
        self.f = 1

    def connect(self, port: Optional[str], elf_path: Optional[str]) -> None:
        # Real connection details are highly device specific. This minimal
        # implementation assumes defaults and simply calls connect().
        self.scope.connect()

    def configure_channels(self, names: List[str], bytes_per_var: List[int]) -> None:
        self.scope.clear_all_scope_channel()
        self.channel_names = names
        self.channel_widths = bytes_per_var
        for name in names:
            var = self.scope.get_variable(name)
            self.scope.add_scope_channel(var)

    def set_sample_time_factor(self, f: int) -> None:
        self.f = max(1, int(f))
        self.scope.set_sample_time(self.f)

    def write_var(self, name: str, value: float | int) -> None:
        try:
            var = self.scope.get_variable(name)
            var.value = value
        except Exception:
            pass

    def request_scope_data(self, duration_s: float) -> None:
        # duration is controlled on device by set_sample_time and buffer size.
        self.scope.request_scope_data()

    def is_scope_ready(self) -> bool:
        return self.scope.is_scope_data_ready()

    def get_data(self) -> Dict[str, np.ndarray]:
        raw = self.scope.get_scope_channel_data(valid_data=True)
        return {k: np.asarray(v, dtype=float) for k, v in raw.items()}

    def get_device_info(self) -> Dict:
        try:
            return self.scope.get_device_info() or {}
        except Exception:
            return {}

    def estimate_buffer(self) -> Optional[int]:
        # Try to use internal helper if available; otherwise run a small capture.
        try:
            length = self.scope._calc_sda_used_length()  # type: ignore[attr-defined]
            return int(length)
        except Exception:
            # Fallback: run a short capture with minimal settings
            try:
                self.scope.clear_all_scope_channel()
                if not self.channel_names:
                    return None
                var = self.scope.get_variable(self.channel_names[0])
                self.scope.add_scope_channel(var)
                self.scope.set_sample_time(1)
                self.scope.request_scope_data()
                time.sleep(0.25)
                while not self.scope.is_scope_data_ready():
                    time.sleep(0.25)
                data = self.scope.get_scope_channel_data(valid_data=True)
                samples = len(next(iter(data.values())))
                bytes_per_sample = self.channel_widths[0]
                return samples * bytes_per_sample
            except Exception:
                return None


# --------------------------- Feasibility Functions ---------------------------

@dataclass
class FeasibilityResult:
    uart_bytes_per_sec: float
    uart_capacity: float
    load_ratio: float
    buffer_time: Optional[float]
    total_size: float
    Fs: float
    color: str
    load_text: str


def compute_feasibility(num_vars: int, bytes_per_var: List[int], f: int,
                         baud: int, duration_s: float,
                         sda_bytes: Optional[int]) -> FeasibilityResult:
    bytes_per_sample = sum(bytes_per_var)
    Fs = LOOP_HZ / f
    uart_bytes_per_sec = bytes_per_sample * Fs
    uart_capacity = baud / 10.0
    load_ratio = uart_bytes_per_sec / uart_capacity if uart_capacity > 0 else 0

    if load_ratio > 0.7:
        color = "red"
        load_text = "Likely to overrun / choppy"
    elif load_ratio > 0.4:
        color = "orange"
        load_text = "Tight; may drop if overhead adds up"
    else:
        color = "green"
        load_text = "Comfortable"

    buffer_time = None
    if sda_bytes is not None:
        if bytes_per_sample > 0 and Fs > 0:
            buffer_time = sda_bytes / (bytes_per_sample * Fs)

    total_size = bytes_per_sample * Fs * duration_s

    return FeasibilityResult(
        uart_bytes_per_sec=uart_bytes_per_sec,
        uart_capacity=uart_capacity,
        load_ratio=load_ratio,
        buffer_time=buffer_time,
        total_size=total_size,
        Fs=Fs,
        color=color,
        load_text=load_text,
    )


# ------------------------------- GUI Widgets --------------------------------

class MotorLoggerGUI(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Motor Logger")
        self.backend: ScopeBackend = DemoBackend()
        self.sda_bytes: Optional[int] = None
        self.data: Dict[str, np.ndarray] = {}
        self.time: np.ndarray | None = None
        self.summary_csv_path: Optional[str] = None

        self._build_ui()
        self.update_feasibility()

    # UI Construction -------------------------------------------------
    def _build_ui(self) -> None:
        layout = QVBoxLayout(self)
        self.tabs = QTabWidget()
        layout.addWidget(self.tabs)

        self.capture_tab = QWidget()
        self.results_tab = QWidget()
        self.tabs.addTab(self.capture_tab, "Capture")
        self.tabs.addTab(self.results_tab, "Results")

        self._build_capture_tab()
        self._build_results_tab()
        self.status = QStatusBar()
        layout.addWidget(self.status)

    def _build_capture_tab(self) -> None:
        layout = QVBoxLayout(self.capture_tab)

        # Device group -------------------------------------------------
        dev_group = QGroupBox("Device")
        layout.addWidget(dev_group)
        grid = QGridLayout(dev_group)
        grid.addWidget(QLabel("Port:"), 0, 0)
        self.port_edit = QLineEdit()
        grid.addWidget(self.port_edit, 0, 1)
        grid.addWidget(QLabel("ELF path:"), 1, 0)
        self.elf_edit = QLineEdit()
        grid.addWidget(self.elf_edit, 1, 1)
        self.mode_label = QLabel("Mode: Demo")
        grid.addWidget(self.mode_label, 0, 2)
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.handle_connect)
        grid.addWidget(self.connect_btn, 1, 2)

        # Variables group ---------------------------------------------
        var_group = QGroupBox("Variables")
        layout.addWidget(var_group)
        vbox = QVBoxLayout(var_group)
        self.var_table = QTableWidget(5, 2)
        self.var_table.setHorizontalHeaderLabels(["Variable", "Bytes"])
        self.var_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)
        defaults = [
            "motor.idqCmd.q",
            "motor.idq.q",
            "motor.idq.d",
            "motor.omegaElectrical",
            "motor.omegaCmd",
        ]
        for row, name in enumerate(defaults):
            item = QTableWidgetItem(name)
            self.var_table.setItem(row, 0, item)
            combo = QComboBox()
            combo.addItems(["2", "4"])
            combo.setCurrentText("2")
            combo.currentIndexChanged.connect(self.update_feasibility)
            self.var_table.setCellWidget(row, 1, combo)
        vbox.addWidget(self.var_table)
        self.reset_vars_btn = QPushButton("Reset defaults")
        self.reset_vars_btn.clicked.connect(lambda: self._reset_variables(defaults))
        vbox.addWidget(self.reset_vars_btn)

        # Timing group -------------------------------------------------
        time_group = QGroupBox("Timing")
        layout.addWidget(time_group)
        grid = QGridLayout(time_group)
        grid.addWidget(QLabel("Duration (s):"), 0, 0)
        self.duration_spin = QDoubleSpinBox()
        self.duration_spin.setRange(0.05, 20.0)
        self.duration_spin.setSingleStep(0.05)
        self.duration_spin.setValue(3.0)
        self.duration_spin.valueChanged.connect(self._duration_changed)
        grid.addWidget(self.duration_spin, 0, 1)

        grid.addWidget(QLabel("Motor ON time (s):"), 1, 0)
        self.motor_on_spin = QDoubleSpinBox()
        self.motor_on_spin.setRange(0.0, 20.0)
        self.motor_on_spin.setSingleStep(0.05)
        self.motor_on_spin.setValue(0.0)
        self.motor_on_spin.valueChanged.connect(self.update_feasibility)
        grid.addWidget(self.motor_on_spin, 1, 1)

        grid.addWidget(QLabel("Desired Hz:"), 2, 0)
        self.hz_spin = QDoubleSpinBox()
        self.hz_spin.setRange(1, LOOP_HZ)
        self.hz_spin.setValue(2000.0)
        self.hz_spin.valueChanged.connect(self._hz_changed)
        grid.addWidget(self.hz_spin, 2, 1)

        grid.addWidget(QLabel("Sample factor f:"), 3, 0)
        self.factor_spin = QSpinBox()
        self.factor_spin.setRange(1, 1000)
        self.factor_spin.setValue(10)
        self.factor_spin.valueChanged.connect(self._factor_changed)
        grid.addWidget(self.factor_spin, 3, 1)

        self.actual_fs_label = QLabel("Actual Fs: 2000 Hz")
        grid.addWidget(self.actual_fs_label, 2, 2, 2, 1)

        # Feasibility panel -------------------------------------------
        feas_group = QGroupBox("Feasibility")
        layout.addWidget(feas_group)
        grid = QGridLayout(feas_group)
        grid.addWidget(QLabel("UART baud:"), 0, 0)
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["115200", "230400", "460800", "921600", "Custom"])
        self.baud_combo.currentIndexChanged.connect(self.update_feasibility)
        grid.addWidget(self.baud_combo, 0, 1)
        self.custom_baud = QSpinBox()
        self.custom_baud.setRange(1, 10_000_000)
        self.custom_baud.setValue(115200)
        self.custom_baud.valueChanged.connect(self.update_feasibility)
        grid.addWidget(self.custom_baud, 0, 2)

        self.buffer_label = QLabel("Buffer time: Unknown")
        grid.addWidget(self.buffer_label, 1, 0, 1, 3)
        self.uart_label = QLabel("UART load: -")
        grid.addWidget(self.uart_label, 2, 0, 1, 3)
        self.total_label = QLabel("Total size: -")
        grid.addWidget(self.total_label, 3, 0, 1, 3)

        grid.addWidget(QLabel("Reasons & Risks:"), 4, 0)
        self.reasons_edit = QTextEdit()
        self.reasons_edit.setReadOnly(True)
        grid.addWidget(self.reasons_edit, 5, 0, 1, 3)

        self.probe_btn = QPushButton("Probe Buffer")
        self.probe_btn.clicked.connect(self.handle_probe_buffer)
        grid.addWidget(self.probe_btn, 6, 0)

        # Static ballpark table
        ballpark = QLabel(
            "For V=5, Bv=2:\n"
            "115200 bps → f≥18 (≈1.11 kHz)\n"
            "230400 bps → f≥9 (≈2.22 kHz)\n"
            "921600 bps → f≥3 (≈6.67 kHz)"
        )
        grid.addWidget(ballpark, 6, 1, 1, 2)

        # Actions ------------------------------------------------------
        action_box = QHBoxLayout()
        layout.addLayout(action_box)
        self.start_btn = QPushButton("Start Capture")
        self.start_btn.clicked.connect(self.handle_start)
        action_box.addWidget(self.start_btn)
        self.stop_btn = QPushButton("Stop")
        self.stop_btn.clicked.connect(self.handle_stop)
        action_box.addWidget(self.stop_btn)

    def _build_results_tab(self) -> None:
        layout = QVBoxLayout(self.results_tab)
        self.figure = Figure(figsize=(5, 4))
        self.canvas = FigureCanvas(self.figure)
        layout.addWidget(self.canvas)
        self.summary_edit = QTextEdit()
        self.summary_edit.setReadOnly(True)
        layout.addWidget(self.summary_edit)
        button_layout = QHBoxLayout()
        layout.addLayout(button_layout)
        self.save_btn = QPushButton("Save CSV As...")
        self.save_btn.clicked.connect(self.save_csv)
        button_layout.addWidget(self.save_btn)
        self.copy_btn = QPushButton("Copy summary")
        self.copy_btn.clicked.connect(self.copy_summary)
        button_layout.addWidget(self.copy_btn)

    # Helper methods --------------------------------------------------
    def _reset_variables(self, defaults: List[str]) -> None:
        for row, name in enumerate(defaults):
            self.var_table.item(row, 0).setText(name)
            combo = self.var_table.cellWidget(row, 1)
            combo.setCurrentText("2")
        self.update_feasibility()

    def _duration_changed(self, value: float) -> None:
        self.motor_on_spin.setMaximum(value)
        self.update_feasibility()

    def _hz_changed(self, value: float) -> None:
        f = max(1, int(round(LOOP_HZ / value)))
        self.factor_spin.blockSignals(True)
        self.factor_spin.setValue(f)
        self.factor_spin.blockSignals(False)
        self.actual_fs_label.setText(f"Actual Fs: {LOOP_HZ / f:.2f} Hz")
        self.update_feasibility()

    def _factor_changed(self, value: int) -> None:
        Fs = LOOP_HZ / value
        self.hz_spin.blockSignals(True)
        self.hz_spin.setValue(Fs)
        self.hz_spin.blockSignals(False)
        self.actual_fs_label.setText(f"Actual Fs: {Fs:.2f} Hz")
        self.update_feasibility()

    def _current_vars(self) -> tuple[List[str], List[int]]:
        names: List[str] = []
        bytes_list: List[int] = []
        for row in range(self.var_table.rowCount()):
            item = self.var_table.item(row, 0)
            name = item.text().strip() if item else ""
            if name:
                names.append(name)
                combo: QComboBox = self.var_table.cellWidget(row, 1)  # type: ignore
                bytes_list.append(int(combo.currentText()))
        return names, bytes_list

    def _baud_value(self) -> int:
        if self.baud_combo.currentText() == "Custom":
            return self.custom_baud.value()
        return int(self.baud_combo.currentText())

    def handle_connect(self) -> None:
        port = self.port_edit.text() or None
        elf = self.elf_edit.text() or None
        try:
            if X2CScope is not None:
                self.backend = RealScopeBackend()
                self.backend.connect(port, elf)
                self.mode_label.setText("Mode: Hardware")
            else:
                raise RuntimeError("pyX2Cscope not installed; using demo")
        except Exception as exc:
            QMessageBox.warning(self, "Connection", f"Using Demo mode: {exc}")
            self.backend = DemoBackend()
            self.mode_label.setText("Mode: Demo")
        self.update_feasibility()

    def handle_probe_buffer(self) -> None:
        self.sda_bytes = self.backend.estimate_buffer()
        if self.sda_bytes is None:
            QMessageBox.warning(self, "Probe", "Unable to determine buffer size")
        else:
            QMessageBox.information(self, "Probe", f"Estimated buffer size: {self.sda_bytes} bytes")
        self.update_feasibility()

    def update_feasibility(self) -> None:
        names, bytes_list = self._current_vars()
        if not names:
            return
        duration = self.duration_spin.value()
        f = self.factor_spin.value()
        baud = self._baud_value()
        result = compute_feasibility(len(names), bytes_list, f, baud,
                                     duration, self.sda_bytes)
        self.uart_label.setText(
            f"UART load: {result.uart_bytes_per_sec:.0f} / {result.uart_capacity:.0f} B/s"
        )
        self.uart_label.setStyleSheet(f"color: {result.color}")
        if result.buffer_time is None:
            buf_text = "Buffer time: Unknown"
        else:
            buf_text = f"Buffer time: {result.buffer_time:.2f} s"
        self.buffer_label.setText(buf_text)
        self.total_label.setText(f"Total size: {result.total_size/1024:.1f} kB")

        reasons = [
            f"At your settings, UART load = {result.uart_bytes_per_sec:.0f} B/s vs capacity {result.uart_capacity:.0f} B/s → {result.load_text}.",
            f"At f={f} you are effectively logging Fs={result.Fs:.1f} Hz.",
        ]
        if result.buffer_time is not None:
            if duration > result.buffer_time:
                reasons.append(
                    f"Estimated buffer time {result.buffer_time:.2f}s < duration {duration:.2f}s → streaming required or will stop early."
                )
        if result.total_size > 25 * 1024 * 1024:
            reasons.append("Total capture size is very large (>25 MB); CSV export may be slow.")
        self.reasons_edit.setPlainText("\n".join(reasons))

    def handle_start(self) -> None:
        names, bytes_list = self._current_vars()
        if not names:
            QMessageBox.warning(self, "Start", "No variables configured")
            return
        duration = self.duration_spin.value()
        f = self.factor_spin.value()
        self.backend.configure_channels(names, bytes_list)
        self.backend.set_sample_time_factor(f)
        self.backend.request_scope_data(duration)
        self.backend.write_var("motor.enable", 1)
        motor_on = self.motor_on_spin.value()
        if motor_on > 0:
            QTimer.singleShot(int(motor_on*1000), lambda: self.backend.write_var("motor.enable", 0))
        QTimer.singleShot(0, self.poll_ready)
        self.status.showMessage("Capture in progress...")

    def poll_ready(self) -> None:
        if self.backend.is_scope_ready():
            self.finish_capture()
        else:
            QTimer.singleShot(250, self.poll_ready)

    def finish_capture(self) -> None:
        data = self.backend.get_data()
        names, bytes_list = self._current_vars()
        f = self.factor_spin.value()
        duration = self.duration_spin.value()
        Fs = LOOP_HZ / f
        N_expected = int(round(duration * Fs))
        # Align lengths
        N_raw = min(len(v) for v in data.values()) if data else 0
        N = min(N_expected, N_raw)
        t = np.arange(N) / Fs
        self.time = t
        self.data = {name: data.get(name, np.zeros(N))[:N] for name in names}
        self.status.showMessage("Capture complete", 5000)
        self.update_plots()
        self.update_summary(N_expected, N_raw, N)
        self.tabs.setCurrentWidget(self.results_tab)

    def update_plots(self) -> None:
        self.figure.clear()
        ax1 = self.figure.add_subplot(2, 1, 1)
        ax2 = self.figure.add_subplot(2, 1, 2)
        t = self.time if self.time is not None else np.array([])
        if t.size:
            if "motor.omegaElectrical" in self.data:
                ax1.plot(t, self.data.get("motor.omegaElectrical"), label="omegaElectrical")
            if "motor.omegaCmd" in self.data:
                ax1.plot(t, self.data.get("motor.omegaCmd"), label="omegaCmd")
            ax1.legend(); ax1.set_ylabel("Speed")
            if "motor.idq.q" in self.data:
                ax2.plot(t, self.data.get("motor.idq.q"), label="idq.q")
            if "motor.idq.d" in self.data:
                ax2.plot(t, self.data.get("motor.idq.d"), label="idq.d")
            if "motor.idqCmd.q" in self.data:
                ax2.plot(t, self.data.get("motor.idqCmd.q"), label="idqCmd.q")
            ax2.legend(); ax2.set_ylabel("Currents")
            ax2.set_xlabel("Time (s)")
        self.canvas.draw()

    def update_summary(self, N_expected: int, N_raw: int, N_used: int) -> None:
        f = self.factor_spin.value()
        Fs = LOOP_HZ / f
        names, bytes_list = self._current_vars()
        baud = self._baud_value()
        summary = [
            f"Mode: {'Hardware' if isinstance(self.backend, RealScopeBackend) else 'Demo'}",
            f"Duration: {self.duration_spin.value():.2f} s",
            f"f: {f}",
            f"Effective Fs: {Fs:.1f} Hz",
            f"Variables: {len(names)}",
            f"Per-channel bytes: {bytes_list}",
            f"UART baud: {baud}",
            f"N_expected: {N_expected}",
            f"N_raw: {N_raw}",
            f"N_after_clip: {N_used}",
        ]
        if self.summary_csv_path:
            summary.append(f"CSV path: {self.summary_csv_path}")
        self.summary_edit.setPlainText("\n".join(summary))

    def handle_stop(self) -> None:
        self.backend.write_var("motor.enable", 0)
        self.status.showMessage("Capture stopped", 5000)

    def save_csv(self) -> None:
        if self.time is None or not self.data:
            QMessageBox.warning(self, "Save", "No data to save")
            return
        path, _ = QFileDialog.getSaveFileName(self, "Save CSV", "", "CSV Files (*.csv)")
        if not path:
            return
        with open(path, "w", encoding="utf-8") as f:
            header = ",".join(["t_s"] + list(self.data.keys()))
            f.write(header + "\n")
            for i in range(len(self.time)):
                row = f"{self.time[i]:.9f}" + "," + ",".join(f"{self.data[name][i]:.9f}" for name in self.data)
                f.write(row + "\n")
        self.summary_csv_path = path
        self.status.showMessage(f"Saved CSV to {path}", 5000)
        self.update_summary(len(self.time), len(self.time), len(self.time))

    def copy_summary(self) -> None:
        QApplication.clipboard().setText(self.summary_edit.toPlainText())
        self.status.showMessage("Summary copied", 2000)


# ------------------------------- Entry Point -------------------------------

def main() -> None:
    app = QApplication(sys.argv)
    gui = MotorLoggerGUI()
    gui.resize(900, 700)
    gui.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
