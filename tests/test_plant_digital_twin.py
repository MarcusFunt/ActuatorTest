import math

import numpy as np

from actuator_tool.actuator_analysis import ResonanceResult
from actuator_tool.plant_adapters import (
    chrono_python_snippet,
    clipped_motor_torque_nm,
    mujoco_xml_fragment,
)
from actuator_tool.plant_identification import (
    FrictionFitResult,
    characterize_encoder,
    characterize_latency_events,
    fit_belt_damping_from_existing_results,
    fit_friction,
    fit_inertia,
    fit_torque_speed_map,
)
from actuator_tool.plant_schema import ActuatorPlantParameters, EncoderModelParameters, TorqueSpeedPoint
from actuator_tool.plant_simulator import TwoInertiaActuatorSimulator
from actuator_tool.plant_validation import compare_traces


def _simple_parameters() -> ActuatorPlantParameters:
    return ActuatorPlantParameters(
        gear_ratio_motor_per_output=3.0,
        motor_inertia_kg_m2=2.0e-4,
        output_inertia_kg_m2=2.0e-3,
        belt_stiffness_nm_per_rad=8.0,
        belt_damping_nm_s_per_rad=0.08,
        coulomb_friction_nm=0.0,
        viscous_friction_nm_s_per_rad=0.0,
        static_friction_nm=0.0,
        motor_torque_speed_map=[
            TorqueSpeedPoint(0.0, 1.0),
            TorqueSpeedPoint(1000.0, 1.0),
        ],
        motor_encoder=EncoderModelParameters(sample_period_s=0.001),
        output_encoder=EncoderModelParameters(sample_period_s=0.001),
        integration_step_s=0.0002,
    )


def test_plant_schema_round_trip_and_torque_interpolation():
    params = _simple_parameters()
    encoded = params.as_dict()
    decoded = ActuatorPlantParameters.from_dict(encoded)
    assert decoded.gear_ratio_motor_per_output == 3.0
    assert math.isclose(decoded.output_per_motor, 1.0 / 3.0)
    assert math.isclose(decoded.torque_limit_nm(500.0), 1.0)
    assert decoded.relative_mode_inertia_kg_m2 > 0.0
    assert decoded.predicted_relative_resonance_hz > 0.0


def test_fit_friction_recovers_stribeck_curve():
    velocity = np.linspace(-6.0, 6.0, 801)
    coulomb = 0.075
    static = 0.115
    viscous = 0.012
    v_s = 0.7
    smoothing = 0.05
    torque = np.tanh(velocity / smoothing) * (
        coulomb + (static - coulomb) * np.exp(-np.square(np.abs(velocity) / v_s))
    ) + viscous * velocity

    fit = fit_friction(
        velocity,
        torque,
        include_stribeck=True,
        smoothing_velocity_rad_s=smoothing,
    )
    assert fit.r_squared > 0.995
    assert abs(fit.coulomb_friction_nm - coulomb) < 0.01
    assert abs(fit.static_friction_nm - static) < 0.015
    assert abs(fit.viscous_friction_nm_s_per_rad - viscous) < 0.003
    assert abs(fit.stribeck_velocity_rad_s - v_s) < 0.2


def test_fit_inertia_after_friction_subtraction():
    time_s = np.linspace(0.0, 3.0, 1501)
    velocity = 5.0 * np.sin(2.0 * math.pi * 0.8 * time_s)
    acceleration = np.gradient(velocity, time_s, edge_order=2)
    true_inertia = 0.0032
    friction = FrictionFitResult(
        coulomb_friction_nm=0.05,
        viscous_friction_nm_s_per_rad=0.01,
        static_friction_nm=0.05,
        stribeck_velocity_rad_s=0.5,
        smoothing_velocity_rad_s=0.04,
        rmse_nm=0.0,
        r_squared=1.0,
        sample_count=len(time_s),
        model="coulomb_viscous",
    )
    torque = true_inertia * acceleration + friction.torque_nm(velocity) + 0.007
    fit = fit_inertia(time_s, velocity, torque, friction=friction)
    assert fit.r_squared > 0.999
    assert abs(fit.inertia_kg_m2 - true_inertia) < 1e-4
    assert abs(fit.torque_bias_nm - 0.007) < 1e-3


def test_existing_resonance_result_maps_to_physical_damping():
    stiffness = 4.0
    j_eq = 0.01
    frequency = math.sqrt(stiffness / j_eq) / (2.0 * math.pi)
    resonance = ResonanceResult(
        sample_count=1000,
        sample_rate_hz=500.0,
        start_frequency_hz=1.0,
        end_frequency_hz=20.0,
        peak_frequency_hz=frequency,
        peak_gain=2.0,
        peak_prominence_db=12.0,
        bandwidth_hz=frequency / 10.0,
        q_factor=10.0,
        rms_deflection_rad=0.01,
        peak_deflection_rad=0.03,
        pass_test=True,
        warning="",
    )
    fit = fit_belt_damping_from_existing_results(
        resonance,
        belt_stiffness_nm_per_rad=stiffness,
        equivalent_inertia_kg_m2=j_eq,
    )
    expected = 2.0 * 0.05 * math.sqrt(stiffness * j_eq)
    assert abs(fit.damping_ratio - 0.05) < 1e-12
    assert abs(fit.damping_nm_s_per_rad - expected) < 1e-12


def test_torque_speed_map_is_monotonic_after_fit():
    rng = np.random.default_rng(3)
    speed = np.linspace(0.0, 200.0, 600)
    ideal = 0.9 - 0.003 * speed
    measured = np.maximum(0.1, ideal + rng.normal(0.0, 0.02, len(speed)))
    result = fit_torque_speed_map(speed, measured, bins=10, quantile=0.9)
    torques = [point.torque_nm for point in result.points]
    assert result.points[0].speed_rad_s == 0.0
    assert all(b <= a + 1e-12 for a, b in zip(torques, torques[1:]))


def test_latency_and_encoder_characterization():
    commands = np.arange(20, dtype=float) * 0.1
    latency_samples = np.array([0.012, 0.014, 0.013, 0.015, 0.011] * 4)
    latency = characterize_latency_events(commands, commands + latency_samples)
    assert abs(latency.latency_s - np.mean(latency_samples)) < 1e-12
    assert latency.jitter_std_s > 0.0

    counts_per_rev = 4096
    raw = np.arange(0, 2000, dtype=float)
    time_s = np.arange(len(raw), dtype=float) * 0.004
    angle = raw * (2.0 * math.pi / counts_per_rev)
    characterization = characterize_encoder(time_s, raw, angle)
    assert characterization.counts_per_rev == counts_per_rev
    assert abs(characterization.sample_period_s - 0.004) < 1e-12
    assert characterization.noise_std_rad < 1e-10


def test_two_inertia_simulator_moves_and_limits_motor_torque():
    params = _simple_parameters()
    simulator = TwoInertiaActuatorSimulator(params)
    time_s = np.linspace(0.0, 0.4, 401)
    requested = np.full_like(time_s, 5.0)
    trace = simulator.simulate(time_s, requested)
    assert np.all(np.isfinite(trace.output_angle_rad))
    assert np.max(np.abs(trace.applied_motor_torque_nm)) <= 1.0 + 1e-12
    assert trace.motor_angle_rad[-1] > 0.0
    assert trace.output_angle_rad[-1] > 0.0
    assert np.max(np.abs(trace.relative_deflection_rad)) > 0.0


def test_encoder_observation_quantizes_and_honors_latency_model():
    params = _simple_parameters()
    params.motor_encoder = EncoderModelParameters(
        counts_per_rev=1024,
        sample_period_s=0.002,
        latency_s=0.004,
        noise_std_rad=0.0,
    )
    simulator = TwoInertiaActuatorSimulator(params)
    for _ in range(20):
        simulator.step(0.3, 0.001)
    observation = simulator.observe()
    quantum = 2.0 * math.pi / 1024.0
    assert abs(observation.motor.angle_rad / quantum - round(observation.motor.angle_rad / quantum)) < 1e-10
    assert observation.motor.sample_time_s <= observation.time_s


def test_validation_is_zero_for_identical_trace():
    params = _simple_parameters()
    simulator = TwoInertiaActuatorSimulator(params)
    time_s = np.linspace(0.0, 0.1, 101)
    requested = np.linspace(0.0, 0.3, len(time_s))
    trace = simulator.simulate(time_s, requested)
    report = compare_traces(trace.as_dict(), trace)
    for metric in report.metrics.values():
        assert metric.rmse < 1e-12
        assert metric.mae < 1e-12


def test_mujoco_and_chrono_adapters_keep_compliance_and_ratio():
    params = _simple_parameters()
    xml = mujoco_xml_fragment(params, motor_joint="motor", output_joint="wheel")
    assert '<fixed name="belt_compliance"' in xml
    assert 'joint="motor" coef="0.333333333333"' in xml
    assert f'stiffness="{params.belt_stiffness_nm_per_rad:.12g}"' in xml

    snippet = chrono_python_snippet(params)
    assert "ChShaftsGear" in snippet
    assert "SetCompliant(True" in snippet
    assert "SetTransmissionRatio(0.3333333333333333)" in snippet
    assert clipped_motor_torque_nm(params, 10.0, 0.0) == 1.0
