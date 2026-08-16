# Plant identification and digital twin

`ActuatorTest` now has a reduced-order physical model intended to turn bench measurements into a simulator that can be reused in controller development, MuJoCo, and Project Chrono.

The model is deliberately **not** a tooth-by-tooth belt simulation. It uses two rotating inertias coupled by the measured reduction, belt stiffness, and belt damping:

```text
motor torque -> J_motor -> N:1 reduction -> K_belt + C_belt -> J_output -> external load
```

For a reduction `N = motor speed / output speed`, belt deformation is

```text
delta = theta_motor / N - theta_output
```

and belt torque on the output side is

```text
tau_belt = K_belt * delta + C_belt * delta_dot
```

The motor-side reaction from the belt is `tau_belt / N`.

## Modules

- `plant_schema.py` — `ActuatorPlantParameters`, encoder models, and measured torque-speed points.
- `plant_identification.py` — friction, inertia, damping, torque-speed, latency/jitter, and encoder characterization.
- `plant_simulator.py` — two-inertia physical simulator with command delay/jitter, torque-speed saturation, friction, and encoder observation models.
- `plant_validation.py` — measured-vs-simulated trace metrics.
- `plant_adapters.py` — MuJoCo MJCF fragment/runtime data and Project Chrono shaft-model snippets.

## Recommended bench sequence

### 1. Ratio and encoder sanity

Use the existing ratio calibration first. Do not fit a plant until motor/output encoder signs and the reduction are stable.

### 2. Static compliance / belt stiffness

Use the existing compliance test with a known applied torque. A known mass on a measured lever arm is sufficient for the first pass.

The important measured quantity is the dual-encoder deflection:

```text
delta = theta_motor / N - theta_output
K_belt ~= applied_output_torque / delta
```

Repeat in both directions and at several loads. Nonlinearity or hysteresis is useful evidence rather than something to average away.

### 3. Friction

Collect slow steady-state sweeps in both directions with a calibrated torque estimate. Pass output speed and the *signed resisting torque magnitude* to `fit_friction()`.

The fitted model is:

```text
tau_friction = sign(v) * [Tc + (Ts - Tc) exp(-(abs(v)/Vs)^2)] + Bv
```

where `Tc` is Coulomb friction, `Ts` is the low-speed/static/Stribeck level, `Vs` is the Stribeck velocity scale, and `B` is viscous friction.

If the data does not support a Stribeck fit, call `fit_friction(..., include_stribeck=False)` for a simpler Coulomb + viscous model.

### 4. Inertia

Use an acceleration/deceleration capture with known applied torque. `fit_inertia()` numerically differentiates shaft velocity and fits:

```text
tau_applied - tau_friction = J * alpha + torque_bias
```

Do separate experiments if you want separately identified motor-side and output-side inertias. Otherwise use CAD/scale estimates for one side and identify the other.

### 5. Belt damping / drivetrain resonance

The existing chirp and step-response tests already estimate resonance frequency, Q factor, and a step-response damping ratio. Feed those existing result objects to `fit_belt_damping_from_existing_results()` together with measured belt stiffness.

If you collect a free ring-down response of encoder deflection, `fit_belt_damping_from_ringdown()` fits the exponential envelope directly.

### 6. NEMA17 torque-speed envelope

Measure the maximum sustainable motor torque at multiple motor speeds with the real driver current, bus voltage, and microstepping configuration. Do not infer this from electrical input power alone.

Feed the measured `(motor_speed_rad_s, available_torque_nm)` samples to `fit_torque_speed_map()`. The fit bins noisy measurements and can enforce the physically useful non-increasing torque envelope used by the simulator.

Store the test current and bus voltage in `ActuatorPlantParameters.torque_speed_reference_current_a` and `torque_speed_reference_bus_voltage_v`.

### 7. Command latency and jitter

If you have matched command and first-response timestamps, use `characterize_latency_events()`.

If you only have sampled input/output signals, use `estimate_signal_latency()`; it resamples to a uniform grid and estimates positive command-to-response delay by cross-correlation.

### 8. Encoder resolution/noise/timing

Use `characterize_encoder()` with actuator timestamps, raw counts, and converted radians. Supply the known encoder CPR for wrapped raw counters. It reports quantization, residual noise, mean sample period, and sample jitter.

Encoder latency should be measured separately and copied into the returned `EncoderModelParameters` with `to_parameters(latency_s=...)`.

## Building the parameter file

Start with a real mechanical ratio and sensible CAD estimates:

```python
from actuator_tool.plant_schema import ActuatorPlantParameters

plant = ActuatorPlantParameters(
    actuator_id="x1-left",
    gear_ratio_motor_per_output=3.0,
    motor_inertia_kg_m2=..., 
    output_inertia_kg_m2=...,
    belt_stiffness_nm_per_rad=...,
)
```

Then fit measurements and merge them with `apply_identification_results()`.

```python
from actuator_tool.plant_identification import apply_identification_results

plant = apply_identification_results(
    plant,
    friction=friction_fit,
    motor_inertia=motor_inertia_fit,
    output_inertia=output_inertia_fit,
    belt_damping=damping_fit,
    torque_speed=torque_speed_fit,
    command_latency=latency_fit,
    motor_encoder=motor_encoder_fit,
    output_encoder=output_encoder_fit,
)
plant.to_json("x1_left_plant.json")
```

## Running the physical simulator

```python
import numpy as np
from actuator_tool.plant_simulator import TwoInertiaActuatorSimulator

sim = TwoInertiaActuatorSimulator(plant)
t = np.arange(0.0, 2.0, 0.001)
torque_command = np.zeros_like(t)
torque_command[t >= 0.2] = 0.2
trace = sim.simulate(t, torque_command)
```

The trace includes motor/output angle and velocity, belt deflection, belt torque, applied motor torque after the torque-speed limit, friction torque, and external load torque.

`sim.observe()` returns encoder-like observations with the configured sampling period, quantization, noise, bias, latency, and jitter.

## Validation against real hardware

Do not validate on the same excitation used to fit every parameter. Keep at least one step/ramp/chirp or arbitrary command sequence as a hold-out trace.

```python
from actuator_tool.plant_validation import validate_simulator_against_telemetry

sim_trace, report = validate_simulator_against_telemetry(
    telemetry_samples,
    sim,
    measured_or_calibrated_motor_torque_sequence,
)
print(report.as_dict())
```

The report calculates RMSE, MAE, maximum absolute error, normalized RMSE, and correlation for motor/output position and velocity.

A stepper's driver current is **not** automatically equal to shaft torque. For quantitative validation, derive the input torque from the measured torque-speed map/load-cell data or another calibrated model.

## MuJoCo

`mujoco_xml_fragment()` emits a fixed tendon whose coordinate is `q_motor/N - q_output`, with measured belt stiffness and damping. The generated actuator still needs a small runtime wrapper for the measured torque-speed envelope and command latency/jitter. Stribeck friction also remains a runtime/passive-force correction; MuJoCo's native joint `frictionloss` and `damping` cover the Coulomb and viscous parts.

Be careful not to double-count inertia: use the returned `armature` suggestions only for rotor/output inertia that is not already represented in the rigid-body inertial properties.

## Project Chrono

`chrono_python_snippet()` emits a 1-D `ChShaft` drivetrain with a compliant `ChShaftsGear`. Chrono defines transmission ratio as `w2 / w1`, so the adapter converts the stored motor-per-output ratio `N` to `1/N`.

The snippet applies fitted friction and the torque-speed envelope through per-step shaft loads. Command latency/jitter stays in the controller/cosimulation queue rather than being baked into the mechanical model.

## What this model intentionally does not include yet

- individual belt teeth, tooth skipping, wrap/contact mechanics, or tensioner geometry;
- detailed two-phase winding electrical equations and current-regulator switching;
- temperature-dependent torque/friction;
- nonlinear belt stiffness as a lookup table;
- microstep electrical-angle loss-of-synchronism dynamics.

Those can be added if hold-out validation shows they materially improve prediction. The purpose of this model is to be the smallest plant that reproduces the actuator dynamics relevant to balance/control work.
