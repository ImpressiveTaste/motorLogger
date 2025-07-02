# pyX2Cscope Motor Logger GUI

**Last updated:** July 2, 2025

This project provides a Python GUI built with `tkinter` that interfaces with [pyX2Cscope](https://github.com/X2Cscope/pyx2cscope). It can connect to a Microchip MCAF target, start or stop the motor and log multiple variables with optional per‑channel scaling.

![Updated GUI](https://github.com/ImpressiveTaste/motorLogger/blob/main/UPDATEDGUI.png)

![Old GUI](https://github.com/ImpressiveTaste/motorLogger/blob/main/GUI.png)


## Features

- Connects to MCAF-compatible targets using a selected COM port and ELF file
- Start/stop control with a user defined target speed
- Live display of measured and commanded speed
- Logs the following variables for a specified duration:
  - `motor.idqCmd.q`
  - `motor.idq.q`
  - `motor.idq.d`
  - `motor.omegaElectrical`
  - `motor.omegaCmd`
- Enter a scaling factor for each variable so plots and saved data are scaled on the fly
- Plot currents and speed, or save to Excel, CSV or MATLAB format
- Dummy mode available by setting `USE_SCOPE = False`
- Experimental button to disable the sample interval guard

## Requirements

- Python 3.11+
- [pyX2Cscope](https://pypi.org/project/pyx2cscope/)
- Optional: `matplotlib`, `pandas`, and `scipy` for plotting and saving
- Windows if you want to build a `.exe`

## Setup

1. Create and activate a virtual environment:
   ```bash
   python -m venv venv
   venv\Scripts\activate
   ```
2. Install the required package:
   ```bash
   pip install pyx2cscope
   ```
3. (Optional) Verify the tool:
   ```bash
   pyx2cscope
   ```
4. Run the GUI:
   ```bash
   python MotorLogger.py
   ```

Detailed documentation is available at https://x2cscope.github.io/pyx2cscope/

## Usage

1. Select an ELF file and COM port.
2. Set:
   - Target speed (RPM)
   - Scale (RPM/count)
   - Logging time (seconds)
   - Sample interval (ms)
    - Logging now uses X2Cscope scope channels. All selected variables can be
      sampled at **1 ms** intervals. The GUI enforces a 1 ms minimum by default,
      but you can disable this using the experimental button to try faster
      rates.
3. Click **START** to capture data. Press **STOP** to end the capture early.
4. Use the buttons to plot currents, plot speed, or save the captured data.

## How to Determine RPM/Count Scaling

Refer to the MotorBench report located at:
```
<YourProject>.X/mcc_generated_files/motorBench/aux-files/report.html
```
An example value is `0.19913 RPM/ct`. Copy this into the Scale field.

## Making a Standalone .exe (Optional)

1. Install the tool:
   ```bash
   pip install auto-py-to-exe
   ```
2. Launch it:
   ```bash
   auto-py-to-exe
   ```
3. Select `MotorLogger.py`, one-file mode, and build the project.

## References

- [pyX2Cscope on GitHub](https://github.com/X2Cscope/pyx2cscope)
- [pyX2Cscope on PyPI](https://pypi.org/project/pyx2cscope/)
- [X2Cscope documentation](https://x2cscope.github.io/)
- [auto-py-to-exe on PyPI](https://pypi.org/project/auto-py-to-exe/)

## Notes

The logger uses the following scope variables:

- `motor.apiData.velocityReference`
- `motor.apiData.velocityMeasured`
- `motor.apiData.runMotorRequest`
- `motor.apiData.stopMotorRequest`
- `app.hardwareUiEnabled`

Set `USE_SCOPE = False` to run the GUI without hardware.

### UPDATES

- Fixed multi-channel capture when using newer `pyX2Cscope` releases.
- Fixed irregular time axis in captured data.
- Added experimental button to remove the sample interval safeguard.
- Scope capture now uses X2Cscope channels allowing 1 ms sampling for up to six
  variables.
- Sample guard updated: minimum sample interval is 1 ms for all variables.

