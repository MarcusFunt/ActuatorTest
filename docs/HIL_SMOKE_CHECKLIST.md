# Manual hardware-in-the-loop smoke checklist

This is a manually dispatched, evidence-producing procedure. Run it only on an
unloaded, interlocked fixture with an operator present. It is not CI and must not
be run against a mechanism that can cause injury or damage.

## Before enabling the actuator

1. Record the firmware build revision, board, driver current settings, serial port
   and test operator in the session notes.
2. Confirm the fixture has physical end stops, an accessible power disconnect and
   an independent emergency stop.
3. Start the application with the actuator disabled. Confirm `INFO` reports the
   expected actuator identity and telemetry schema.

## Smoke sequence

1. Start telemetry and confirm both encoder readings are live and stationary.
2. Send `SELF_TEST`, then a small disabled-mode round trip (`PING`, `INFO`).
3. Enter calibration mode and make one low-speed, small relative move; verify
   encoder direction and limit behavior.
4. Send STOP, then ESTOP; confirm motion is disabled and the fault is visible.
5. Clear faults, verify the device remains disabled, then repeat the low-speed move.
6. Exercise the configured missed-step/fault response only at conservative speed.
7. Save the report folder, including `run_manifest.json`, raw telemetry and the
   operator’s pass/fail observations.

## Acceptance

The run passes only when all commands return as expected, ESTOP/fault behavior is
visible and recoverable, limits are respected, and the report artefacts are saved.
Any unexpected movement, missing telemetry, invalid limits, or ambiguous state is
a failed run; disconnect power and record the result.
