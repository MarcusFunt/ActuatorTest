# Actuator Bench Tool

Reflex/Bokeh web UI for calibrating and characterizing a serial-controlled actuator.

The app includes a reusable Python backend, binary protocol framing, a simulator,
live telemetry plots, ratio calibration, resonance characterization, transmission
deflection calculation, CSV recording, session reports, and firmware hooks for
output-space motion with conservative PID/compensation support.

Scope is intentionally limited to actuator bench control, actuator testing,
calibration analysis, and report generation. ROS/MCAP export and firmware
flashing/updater modules are out of scope.

## Quick start

Requirements:

- Python 3.11 or newer

From the repository root, run:

```powershell
py .\run_app.py
```

That command creates `.venv` if needed, installs this project in editable mode,
and starts the app with the simulator selected. Open:

```text
http://localhost:3000
```

To run against hardware instead of the simulator:

```powershell
py .\run_app.py --port COM3
```

Replace `COM3` with the actuator serial port. Stop the app with `Ctrl+C` in the
terminal.

If `py` is not available, use:

```powershell
python .\run_app.py
```

If your default Python is older than 3.11, use `py -3.11 .\run_app.py`.

## Manual setup

Use this path if you want to manage the environment yourself.

Windows PowerShell:

```powershell
py -3.11 -m venv .venv
.\.venv\Scripts\python -m pip install -e ".[dev]"
.\.venv\Scripts\actuator-gui.exe
```

macOS/Linux:

```bash
python3 -m venv .venv
./.venv/bin/python -m pip install -e ".[dev]"
./.venv/bin/actuator-gui
```

Installed run commands:

```powershell
actuator-gui          # simulator selected by default
actuator-gui --sim    # explicit simulator mode
actuator-gui --port COM3
actuator-gui --prod
python -m actuator_tool --sim
```

The Reflex frontend uses port `3000`, the Reflex backend uses port `8001`, and
the embedded Bokeh telemetry server uses port `5006`.

## Running tests

After using `run_app.py` once, run:

```powershell
.\.venv\Scripts\python -m pytest
```

Or, if you installed manually and activated the environment:

```powershell
python -m pytest
```

## Firmware

The Arduino firmware lives in `firmware/ActuatorFirmware/ActuatorFirmware.ino`.
Advanced characterization currently includes host-driven runners for resonance
chirps, backlash, step response, velocity ramp, and compliance checks. The
firmware exposes `START_CHIRP` and `MOVE_OUTPUT_REL` commands in addition to the
original `MOVE_REL`; output-encoder PID, backlash feed-forward, and resonance
derating are configured through the existing JSON config command path.

## Binary protocol

Frames use a fixed little-endian envelope:

| Field | Size | Notes |
| --- | ---: | --- |
| Magic | 2 bytes | `A5 5A` |
| Version | 1 byte | Current version is `1` |
| Type | 1 byte | Command, response, telemetry, or event |
| Sequence | uint16 | Responses echo the command sequence |
| Payload length | uint16 | Maximum accepted payload is 4096 bytes |
| Payload | variable | Command-specific binary payload |
| CRC | uint16 | CRC-16/CCITT-FALSE over version through payload |

CRC parameters are poly `0x1021`, init `0xFFFF`, no reflection, xorout `0x0000`;
the check value for `123456789` is `0x29B1`. Parsers validate CRC before
accepting version/type/payload data and resynchronize to the next magic marker
after corrupt frames.
