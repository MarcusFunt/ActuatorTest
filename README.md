# Actuator Bench Tool

Reflex/Bokeh web UI for calibrating and characterizing a serial-controlled actuator.

The app includes a reusable Python backend, binary protocol framing, a simulator,
live telemetry plots, ratio calibration, resonance characterization, transmission
deflection calculation, CSV recording, session reports, and firmware hooks for
output-space motion with conservative PID/compensation support.

Scope is intentionally limited to actuator bench control, actuator testing,
calibration analysis, and report generation. ROS/MCAP export and firmware
flashing/updater modules are out of scope.

## Physical plant identification and digital twin

The repository also contains a compact physical plant pipeline for the belted
actuator. It is designed to turn dual-encoder bench measurements into parameters
that can be reused in controller simulations rather than requiring a custom
physics engine.

Implemented pieces include:

- Coulomb + viscous + optional Stribeck friction identification;
- motor/output inertia fitting and relative-mode motor-inertia inference;
- belt damping estimation from the existing resonance/Q and step-response data,
  plus a ring-down-envelope fitter;
- measured NEMA17 torque-speed envelope fitting;
- command latency/jitter and signal-delay characterization;
- encoder resolution, quantization/noise, sample-period, and timing-jitter characterization;
- versioned `ActuatorPlantParameters` JSON schema;
- a two-inertia motor/reduction/belt/output simulator with torque-speed saturation,
  fitted friction, command delay, and encoder observation models;
- measured-vs-simulated trace validation metrics; and
- MuJoCo and Project Chrono adapters.

See [`docs/PLANT_IDENTIFICATION.md`](docs/PLANT_IDENTIFICATION.md) for the bench
sequence, equations, fitting workflow, validation procedure, and engine adapters.

The existing `SimulatedTransport` remains useful for protocol/UI smoke testing.
`TwoInertiaActuatorSimulator` is the physical digital-twin model intended for
controller and drivetrain work.

### Guided Characterize Actuator GUI

The Reflex app now includes a guided workflow at:

```text
http://localhost:3000/characterize
```

Connect to hardware (or the simulator) on the normal Bench page first, then open
**Characterize actuator**. The workflow guides the operator through:

1. ratio calibration;
2. static stiffness with a known applied output torque;
3. bidirectional friction sweep and Coulomb/viscous/Stribeck fit;
4. acceleration-based output-inertia identification;
5. chirp + step-response damping, with optional manual ring-down capture;
6. fitting a measured NEMA17 torque-speed envelope;
7. command-to-motion latency and telemetry timing jitter;
8. motor/output encoder quantization, noise, sampling period and jitter;
9. generation of `plant.json` plus a full characterization summary; and
10. a fresh hold-out mechanical replay validation against the two-inertia digital twin.

The torque-speed step deliberately expects **measured mechanical torque** from a
load cell/dynamometer or another calibrated load method. Electrical input power
is not treated as shaft torque. The final validation is explicitly mechanical
replay using encoder-inferred delivered torque; it does not claim a calibrated
TMC2209-current-to-stepper-torque model.

## Quick start

Requirements:

- Python 3.11 or newer

From the repository root, run:

```powershell
py .\run_app.py
```

That command creates `.venv` if needed, installs the checked-in locked dependency
set and this project in editable mode,
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
.\.venv\Scripts\python -m pip install -r requirements.lock
.\.venv\Scripts\python -m pip install --no-deps -e .
.\.venv\Scripts\actuator-gui.exe
```

macOS/Linux:

```bash
python3 -m venv .venv
./.venv/bin/python -m pip install -r requirements.lock
./.venv/bin/python -m pip install --no-deps -e .
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

## Developer checks

The checked-in `requirements.lock` is the supported Python 3.11 dependency set.
Use the task targets below after activating the virtual environment (or set
`PYTHON=.venv/Scripts/python.exe` on Windows):

```bash
make install
make lint
make test
make build
```

`make firmware` compiles the pinned Arduino source against the supported XIAO
ESP32-C3 RISC-V compatibility target because PlatformIO's XIAO ESP32-C6 board
metadata does not enable Arduino yet. Hardware validation on the actual C6 is
manual: follow
[`docs/HIL_SMOKE_CHECKLIST.md`](docs/HIL_SMOKE_CHECKLIST.md) only with an
interlocked unloaded fixture and a present operator.

## Firmware

The Arduino firmware lives in `firmware/ActuatorFirmware/ActuatorFirmware.ino`.
Advanced characterization currently includes host-driven runners for resonance
chirps, backlash, step response, velocity ramp, and compliance checks. The
firmware exposes `START_CHIRP` and `MOVE_OUTPUT_REL` commands in addition to the
original `MOVE_REL`; output-encoder PID, backlash feed-forward, and resonance
derating are configured through the existing JSON config command path.

Production-oriented actuator controls are now exposed through telemetry schema
version 2 while the host decoder still accepts schema 1 telemetry. The actuator
can run standalone position, velocity, and belt-stretch torque-proxy targets:

- `SET_POSITION_TARGET` accepts absolute or relative output-space targets.
- `SET_VELOCITY_TARGET` streams a continuous output velocity target.
- `SET_TORQUE_PROXY_TARGET` controls deflection in radians, where deflection is
  measured output minus ratio-predicted output from the motor encoder.
- `AUTOTUNE_CONTROL` starts the on-device velocity/position tuning state
  machine and writes successful gains to RAM. Use `SAVE_CONFIG` to persist them.
- `GET_CONTROL_STATUS` returns JSON with active targets, deflection, motor slip,
  commanded current, autotune state, and the last control fault.

The firmware also includes small-slip encoder correction, large-slip faulting,
persistent backlash direction compensation, and TMC2209 RMS-current scheduling
that drops to hold current after motion/load settles.

### Hardware smoke checklist

After flashing production-control firmware, run this sequence before applying
load:

1. Connect with the GUI or Python client, confirm `INFO` reports telemetry schema
   version `2`, and run `SELF_TEST`.
2. Enable telemetry and verify both encoders move in the expected directions.
3. In `POSITION` mode, send a small relative `SET_POSITION_TARGET`; confirm
   output position converges, commanded current rises during motion, then drops
   to hold current.
4. Stall lightly by hand for less than the missed-step fault threshold; confirm
   `motor_slip_rad` appears in `GET_CONTROL_STATUS` and recovers.
5. Exceed the configured missed-step fault threshold at low speed; confirm the
   actuator faults and disables motion.
6. In `TORQUE_PROXY` mode, command a small deflection target with conservative
   velocity/excursion limits; confirm the motor moves in the sign that increases
   the requested belt-stretch proxy and aborts on timeout/excursion.
7. Run `AUTOTUNE_CONTROL` with low amplitude and a generous deflection limit;
   confirm it reports success before saving the resulting gains.

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
