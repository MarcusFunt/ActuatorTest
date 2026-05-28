import math

import pytest

from actuator_tool.actuator_analysis import (
    analyze_backlash,
    analyze_compliance,
    analyze_deflection,
    analyze_resonance,
    analyze_step_response,
    analyze_velocity_ramp,
    compute_deflection,
    dominant_frequency,
    fit_ratio,
)
from actuator_tool.actuator_data import TelemetrySample, TelemetryStore


def sample(
    seq: int,
    motor: float,
    output: float,
    t_us: int | None = None,
    motor_vel: float = 1.0,
    output_vel: float = 0.25,
    cmd_pos: float | None = None,
) -> TelemetrySample:
    return TelemetrySample(
        t_us=seq * 10_000 if t_us is None else t_us,
        seq=seq,
        cmd_pos=motor if cmd_pos is None else cmd_pos,
        cmd_vel=0.0,
        motor_enc_raw=int(motor * 1000),
        output_enc_raw=int(output * 1000),
        motor_rad=motor,
        output_rad=output,
        motor_vel_rad_s=motor_vel,
        output_vel_rad_s=output_vel,
        driver_current=0.0,
        bus_voltage=24.0,
        temperature=30.0,
        fault_flags=0,
        mode=1,
    )


def test_telemetry_store_detects_dropped_sequence():
    store = TelemetryStore(max_samples=10)
    store.add(sample(1, 0.0, 0.0))
    store.add(sample(2, 1.0, 0.25))
    store.add(sample(4, 2.0, 0.50))

    assert store.stats.total_samples == 3
    assert store.stats.dropped_samples == 1
    assert store.latest().sample_index == 3


def test_ratio_fit_synthetic_line():
    samples = [
        sample(i, motor=i * 0.1, output=0.25 * i * 0.1 + 0.0132)
        for i in range(100)
    ]

    fit = fit_ratio(samples)

    assert abs(fit.output_per_motor - 0.25) < 1e-9
    assert abs(fit.output_offset_rad - 0.0132) < 1e-9
    assert fit.residual_rms_rad < 1e-10


def test_deflection_zero_for_matching_prediction():
    samples = [
        sample(i, motor=i * 0.1, output=0.25 * i * 0.1 + 0.0132)
        for i in range(10)
    ]

    deflection = compute_deflection(samples, 0.25, 0.0132)

    assert max(abs(value) for value in deflection) < 1e-12


def test_deflection_analysis_reports_rms_peak_and_no_slip():
    samples = [
        sample(i, motor=i * 0.1, output=0.25 * i * 0.1 + 0.0132 + 0.002)
        for i in range(50)
    ]

    result = analyze_deflection(samples, 0.25, 0.0132)

    assert result.sample_count == 50
    assert 0.0019 < result.rms_rad < 0.0021
    assert not result.slip_detected
    assert result.pass_test


def test_backlash_analysis_detects_reversal_deadband():
    samples = []
    seq = 0
    for i in range(30):
        motor = i * 0.02
        samples.append(sample(seq, motor, 0.25 * motor, motor_vel=1.0, output_vel=0.25))
        seq += 1
    start_motor = samples[-1].motor_rad
    start_output = samples[-1].output_rad
    for i in range(1, 8):
        motor = start_motor - i * 0.02
        samples.append(sample(seq, motor, start_output, motor_vel=-1.0, output_vel=0.0))
        seq += 1
    for i in range(8, 25):
        motor = start_motor - i * 0.02
        output = start_output + 0.25 * (motor - (start_motor - 7 * 0.02))
        samples.append(sample(seq, motor, output, motor_vel=-1.0, output_vel=-0.25))
        seq += 1

    result = analyze_backlash(samples, 0.25, output_motion_threshold_rad=0.002)

    assert result.reversal_count >= 1
    assert 0.12 <= result.backlash_motor_rad <= 0.18
    assert result.backlash_output_rad > 0.0


def test_step_response_analysis_basic_metrics():
    samples = []
    for i in range(100):
        t = i * 0.01
        output = 1.0 - pow(2.718281828, -t / 0.12)
        if i > 25:
            output += 0.04 * pow(2.718281828, -(t - 0.25) / 0.2)
        samples.append(
            sample(
                i,
                motor=output * 4.0,
                output=output,
                t_us=int(t * 1_000_000),
                motor_vel=0.0,
                output_vel=0.0,
                cmd_pos=4.0,
            )
        )

    result = analyze_step_response(samples)

    assert result.rise_time_s is not None
    assert result.settling_time_s is not None
    assert result.commanded_step_rad == pytest.approx(result.final_output_rad - result.initial_output_rad)
    assert result.overshoot_percent >= 0.0
    assert result.stable


def test_step_response_uses_known_commanded_step_when_response_unsettled():
    samples = []
    for i in range(80):
        t = i * 0.01
        output = 0.03 * math.sin(2.0 * math.pi * 8.0 * t)
        samples.append(
            sample(
                i,
                motor=0.4,
                output=output,
                t_us=int(t * 1_000_000),
                motor_vel=0.0,
                output_vel=0.0,
                cmd_pos=0.4,
            )
        )

    result = analyze_step_response(samples, output_per_motor=0.25, commanded_step_rad=0.1)

    assert result.commanded_step_rad == pytest.approx(0.1)
    assert result.final_output_rad == pytest.approx(result.initial_output_rad + 0.1)
    assert not result.pass_test
    assert "did not settle" in result.warning


def test_velocity_ramp_analysis_recommends_limits_when_errors_high():
    samples = []
    for i in range(80):
        motor_vel = i / 10.0
        motor = i * 0.05
        expected_output = 0.25 * motor + 0.01
        output = expected_output + (0.1 if motor_vel > 5.0 else 0.002)
        output_vel = motor_vel * 0.25 - (0.8 if motor_vel > 5.0 else 0.0)
        samples.append(
            sample(
                i,
                motor=motor,
                output=output,
                motor_vel=motor_vel,
                output_vel=output_vel,
            )
        )

    result = analyze_velocity_ramp(
        samples,
        0.25,
        0.01,
        max_allowed_tracking_error_rad_s=0.5,
        max_allowed_deflection_rad=0.05,
    )

    assert not result.pass_test
    assert result.recommended_velocity_limit_rad_s < 8.0


def test_compliance_analysis_known_mass_arm_stiffness():
    samples = [
        sample(i, motor=i * 0.01, output=0.25 * i * 0.01 + 0.02)
        for i in range(20)
    ]

    result = analyze_compliance(samples, 0.25, 0.0, mass_kg=1.0, arm_length_m=0.5)

    assert abs(result.torque_nm - 4.905) < 1e-9
    assert result.stiffness_nm_per_rad is not None
    assert result.stiffness_nm_per_rad > 200.0


def test_dominant_frequency_detects_known_signal():
    time_s = [i / 100.0 for i in range(200)]
    values = [__import__("math").sin(2.0 * __import__("math").pi * 5.0 * t) for t in time_s]

    result = dominant_frequency(time_s, values)

    assert result is not None
    assert abs(result - 5.0) < 0.25


def test_resonance_analysis_detects_known_signal():
    samples = []
    sample_hz = 100.0
    for i in range(1000):
        t = i / sample_hz
        motor = 0.08 * math.sin(2.0 * math.pi * 8.0 * t)
        deflection = 0.015 * math.sin(2.0 * math.pi * 8.0 * t)
        samples.append(
            sample(
                i,
                motor=motor,
                output=0.25 * motor + 0.01 + deflection,
                t_us=int(t * 1_000_000),
                motor_vel=0.0,
                output_vel=0.0,
            )
        )

    result = analyze_resonance(samples, 0.25, 0.01, start_frequency_hz=1.0, end_frequency_hz=20.0)

    assert result.peak_frequency_hz is not None
    assert abs(result.peak_frequency_hz - 8.0) < 0.25
    assert result.peak_prominence_db is not None
    assert result.peak_prominence_db > 6.0


def test_resonance_analysis_rejects_too_few_samples():
    samples = [sample(i, motor=0.0, output=0.0) for i in range(20)]

    with pytest.raises(ValueError, match="at least"):
        analyze_resonance(samples, 0.25, 0.0)


def test_resonance_analysis_rejects_low_sample_rate():
    samples = []
    for i in range(100):
        t = i / 20.0
        motor = 0.08 * math.sin(2.0 * math.pi * 4.0 * t)
        samples.append(sample(i, motor=motor, output=0.25 * motor, t_us=int(t * 1_000_000)))

    with pytest.raises(ValueError, match="too low"):
        analyze_resonance(samples, 0.25, 0.0, end_frequency_hz=20.0)


def test_resonance_analysis_rejects_flat_signal():
    samples = [sample(i, motor=0.0, output=0.0) for i in range(100)]

    with pytest.raises(ValueError, match="too small"):
        analyze_resonance(samples, 0.25, 0.0)
