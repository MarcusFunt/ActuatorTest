from __future__ import annotations

import math

import numpy as np

from actuator_tool.actuator_data import ActuatorInfo, TelemetrySample
from actuator_tool.characterization_workflow import (
    characterize_encoders_from_samples,
    derive_motor_inertia_from_relative_mode,
    estimate_latency_from_samples,
    fit_torque_speed_text,
    plant_from_calibration,
)
from actuator_tool.config_schema import CalibrationConfig


def _sample(
    t_us: int,
    seq: int,
    *,
    cmd: float,
    motor: float,
    output: float,
    motor_raw: int,
    output_raw: int,
    motor_vel: float = 0.0,
    output_vel: float = 0.0,
) -> TelemetrySample:
    return TelemetrySample(
        t_us=t_us,
        seq=seq,
        cmd_pos=cmd,
        cmd_vel=0.0,
        motor_enc_raw=motor_raw,
        output_enc_raw=output_raw,
        motor_rad=motor,
        output_rad=output,
        motor_vel_rad_s=motor_vel,
        output_vel_rad_s=output_vel,
        driver_current=1.0,
        bus_voltage=24.0,
        temperature=30.0,
        fault_flags=0,
        mode=0,
    )


def test_plant_seed_uses_physical_positive_ratio_and_calibrated_stiffness():
    calibration = CalibrationConfig(
        output_per_motor=-1.0 / 3.0,
        motor_per_output=-3.0,
        compliance_nm_per_rad=37.5,
    )
    plant = plant_from_calibration(
        calibration,
        ActuatorInfo(actuator_id="x1-left"),
        motor_inertia_kg_m2=2.0e-5,
        output_inertia_kg_m2=2.0e-3,
    )
    assert math.isclose(plant.gear_ratio_motor_per_output, 3.0)
    assert math.isclose(plant.belt_stiffness_nm_per_rad, 37.5)
    assert plant.actuator_id == "x1-left"


def test_motor_inertia_can_be_recovered_from_relative_mode_relation():
    ratio = 3.0
    motor_j = 2.0e-5
    output_j = 1.5e-3
    reflected = motor_j * ratio * ratio
    j_eq = reflected * output_j / (reflected + output_j)
    stiffness = 25.0
    frequency = math.sqrt(stiffness / j_eq) / (2.0 * math.pi)
    recovered = derive_motor_inertia_from_relative_mode(
        belt_stiffness_nm_per_rad=stiffness,
        resonance_frequency_hz=frequency,
        output_inertia_kg_m2=output_j,
        gear_ratio_motor_per_output=ratio,
    )
    assert math.isclose(recovered, motor_j, rel_tol=1e-10)


def test_torque_speed_text_builds_monotonic_envelope():
    fit = fit_torque_speed_text(
        "0,0.50\n20,0.49\n40,0.44\n60,0.46\n80,0.31\n100,0.25\n",
        bins=6,
        quantile=0.9,
    )
    torques = [point.torque_nm for point in fit.points]
    speeds = [point.speed_rad_s for point in fit.points]
    assert speeds[0] == 0.0
    assert all(a <= b for a, b in zip(speeds, speeds[1:]))
    assert all(a >= b for a, b in zip(torques, torques[1:]))


def test_encoder_characterization_reports_4096_count_quantization():
    samples = []
    step = 2.0 * math.pi / 4096.0
    for i in range(30):
        angle = 0.2 + (i % 2) * step
        raw = int(round(angle / step))
        samples.append(
            _sample(
                i * 4000,
                i,
                cmd=0.2,
                motor=raw * step,
                output=raw * step,
                motor_raw=raw,
                output_raw=raw,
            )
        )
    motor, output = characterize_encoders_from_samples(samples)
    assert motor.counts_per_rev == 4096
    assert output.counts_per_rev == 4096
    assert math.isclose(motor.quantization_step_rad, step)
    assert math.isclose(motor.sample_period_s, 0.004)


def test_latency_estimator_detects_sampled_command_delay():
    dt = 0.004
    delay_samples = 5
    n = 300
    command = np.sin(np.arange(n) * 0.11) + 0.35 * np.sin(np.arange(n) * 0.031)
    response = np.concatenate([np.zeros(delay_samples), command[:-delay_samples]])
    samples = []
    for i in range(n):
        samples.append(
            _sample(
                int(round(i * dt * 1_000_000)),
                i,
                cmd=float(command[i]),
                motor=float(response[i]),
                output=float(response[i] / 3.0),
                motor_raw=i,
                output_raw=i,
            )
        )
    result = estimate_latency_from_samples(samples, max_lag_s=0.1)
    assert abs(result.latency_s - delay_samples * dt) <= dt
    assert result.correlation > 0.8


def test_characterization_page_builds():
    # Importing this module also proves the extra route can attach to the same
    # Reflex app without a circular import failure.
    from actuator_gui.characterize_page import characterize_page

    component = characterize_page()
    assert component is not None
