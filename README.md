# Motor Logger GUI (PyQt6)

This project provides a single-file Python application `motor_logger_gui.py` for logging motor control variables via [pyX2Cscope](https://github.com/X2Cscope/pyx2cscope).  It offers a modern PyQt6 interface, a demo mode that generates synthetic data, and live feasibility checks to help you choose safe logging settings.

## Requirements

- Python 3.11+
- [PyQt6](https://pypi.org/project/PyQt6/), [numpy](https://pypi.org/project/numpy/), [matplotlib](https://pypi.org/project/matplotlib/)
- Optional: [pyX2Cscope](https://pypi.org/project/pyx2cscope/) and `mchplnet` for hardware mode

## Running

```bash
python motor_logger_gui.py
```

If `pyX2Cscope` is missing the app starts in **Demo** mode and synthesises data.  When the library is available, click **Connect** and the tool attempts to talk to the hardware.

## Why `set_sample_time(f)` Controls the Rate

X2Cscope is updated inside the fixed 20 kHz PWM loop.  Calling `X2CScope.set_sample_time(f)` tells the MCU to log every `f`‑th PWM sample:

- `f = 1` → 20 kHz effective logging rate
- `f = 2` → 10 kHz
- `f = n` → `20_000 / n` Hz

The firmware side remains untouched—the ISR still runs at 20 kHz—so timing is deterministic while you trade off resolution for duration.

## Feasibility Math

Let

- `V` – number of variables
- `Bv` – bytes per variable (2 or 4)
- `f` – sample‑time factor
- `Fs = 20_000 / f` – effective sample rate
- `baud` – UART baud rate

Per‑second payload (ignoring protocol overhead):

```
Bytes_per_sec = V * Bv * Fs = V * Bv * 20_000 / f
```

UART capacity (8N1 framing ≈ 10 bits/byte):

```
Bytes_per_sec_UART = baud / 10
```

The GUI shows a colour badge:

- **Red** if `Bytes_per_sec` > `0.7 × Bytes_per_sec_UART`
- **Amber** if 0.4–0.7×
- **Green** otherwise

Estimated buffer time (if the device buffer size `SDA_bytes` is known or probed):

```
Bytes_per_sample = V * Bv
Buffer_time ≈ SDA_bytes / (Bytes_per_sample * Fs)
```

Total capture size:

```
Total_bytes ≈ V * Bv * Fs * duration
```

The **Reasons & Risks** panel explains these numbers whenever you change the sample factor, duration or baud rate.

## Ballpark Table (V=5, Bv=2)

| Baud (bps) | f needed for Green | Approx. Fs (Hz) |
|-----------:|-------------------:|---------------:|
| 115 200    | ≥18                | ≈1.11 kHz      |
| 230 400    | ≥9                 | ≈2.22 kHz      |
| 921 600    | ≥3                 | ≈6.67 kHz      |

## Estimating Buffer Size

Some devices report their internal Scope Data Array size.  If not, press **Probe Buffer**:

1. The app arms a minimal channel list and performs a short capture.
2. The number of samples returned infers the buffer size.
3. The estimate (±20 %) is shown in the feasibility panel.

## Known Limits

- UART framing and protocol overhead reduce the effective throughput; staying in the green zone is recommended.
- CSV files larger than ~25 MB may be slow to save or open.
- Demo mode produces synthetic sine‑plus‑noise signals for quick testing.

## Capture Flow

1. Choose variables and byte widths.
2. Enter capture duration and either a desired sample rate or the down‑sample factor `f`.
3. Review the feasibility panel and adjust settings if badges turn red or amber.
4. Press **Start Capture**.  The GUI writes `motor.enable = 1`, captures data and writes `0` after the configured motor‑on time.
5. When finished, plots appear on the **Results** tab where you can export to CSV and copy a summary.

## License

MIT

