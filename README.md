# Actuator Bench Tool

Dear PyGui desktop calibration and characterization tool for a serial-controlled actuator.

The implementation provides a reusable Python backend, binary protocol framing, a simulator,
live plotting, ratio calibration, resonance characterization, transmission deflection
calculation, raw CSV recording, session reports, and firmware hooks for output-space motion
with conservative PID/compensation support.

Scope is intentionally limited to actuator bench control, actuator testing, calibration analysis,
and report generation. ROS/MCAP export and firmware flashing/updater modules are out of scope.

Advanced characterization currently includes host-driven runners for resonance chirps, backlash,
step response, velocity ramp, and compliance checks. The firmware exposes `START_CHIRP` and
`MOVE_OUTPUT_REL` commands in addition to the original `MOVE_REL`; output-encoder PID,
backlash feed-forward, and resonance derating are configured through the existing JSON config
command path.

Run the GUI against the simulator:

```powershell
actuator-gui --sim
```

Run the GUI against hardware:

```powershell
actuator-gui --port COM3
```

Run tests:

```powershell
python -m pytest
```

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
the check value for `123456789` is `0x29B1`. Parsers validate CRC before accepting
version/type/payload data and resynchronize to the next magic marker after corrupt frames.
