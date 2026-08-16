# Guided actuator characterization

Open the normal Bench page first, connect to the actuator, confirm encoder direction and safety limits, then open `/characterize`.

The guided page uses the same `ActuatorClient`, telemetry store, calibration object, report session, and event log as the existing Bench UI. It does not create a second hardware connection.

## Workflow

1. **Ratio** — bidirectional ratio fit. This establishes the physical reduction and seeds `ActuatorPlantParameters`.
2. **Static stiffness** — the operator applies a known output torque while both encoders measure transmission deflection. This determines the effective drivetrain stiffness used by the reduced-order belt model.
3. **Friction** — slow bidirectional velocity segments are used to fit Coulomb, viscous and optional Stribeck friction from encoder-derived belt torque.
4. **Inertia** — alternating velocity commands provide acceleration excitation. The first fit is output-side inertia. Effective motor-side inertia can then be inferred from the measured two-inertia resonance after step 5.
5. **Chirp / ring-down** — the existing resonance chirp and step-response analysis are converted into physical belt damping. A manual ring-down capture is available as an independent check.
6. **Torque-speed** — paste mechanically measured `(motor speed rad/s, available torque N*m)` points. The fitted monotonic envelope is stored with its reference current and bus voltage. Electrical input power is not accepted as a torque measurement.
7. **Latency** — cross-correlation of the fresh step trace estimates command-to-measured-motor lag and telemetry timing jitter.
8. **Encoders** — a static capture estimates quantization, residual angle noise, sample period and timestamp jitter for motor and output encoders.
9. **Generate** — writes `plant.json` and `plant_characterization.json` into the active report folder (or a new characterization folder when no report session exists).
10. **Validation** — runs a fresh hold-out step and compares the measured trace with the two-inertia simulator. The current validation mode is explicitly a **mechanical replay**: delivered motor torque is inferred from the dual-encoder drivetrain state. It validates mechanical dynamics without claiming that TMC2209 current is a calibrated shaft-torque command.

## Safety and measurement quality

The workflow inherits the existing actuator safety limits and fault checks. The stiffness and torque-speed stages still require a physically safe test fixture. Do not hand-load a high-speed torque-speed test.

For static stiffness, use a load cell or a known mass and measured lever arm. Repeat several loads in both directions if you want to check nonlinearity and hysteresis rather than fitting one operating point.

For torque-speed measurements, use a dynamometer/load cell or another calibrated mechanical load. Record the driver current setting, supply voltage, microstep configuration and motor temperature because the available stepper torque envelope depends on them.

## Files produced

- `plant.json` — reusable `ActuatorPlantParameters` for the standalone two-inertia simulator and the MuJoCo/Chrono adapters.
- `plant_characterization.json` — plant plus the individual fit results and validation summary.
- `plant_validation.json` — hold-out validation metrics after step 10.
- Existing session CSV/event/report artifacts continue to be recorded by the normal Bench report infrastructure.
