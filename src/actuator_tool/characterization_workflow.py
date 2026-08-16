"""Guided bench-characterization orchestration for the belted actuator.

This module bridges the existing hardware test runners with the physical plant
identification/digital-twin modules.  It intentionally keeps Reflex/UI concerns
out of the backend so the same workflow can be driven from a CLI or notebook.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import time
from typing import Iterable, Sequence

import numpy as np

from .actuator_analysis import ResonanceResult, StepResponseResult
from .actuator_data import ActuatorInfo, TelemetrySample, TelemetryStore
from .actuator_protocol import ActuatorMode, FaultFlags
from .actuator_serial import ActuatorClient, ActuatorError
from .config_schema import CalibrationConfig, SafetyLimits
from .plant_identification import (
    BeltDampingFitResult,
    EncoderCharacterizationResult,
    FrictionFitResult,
    InertiaFitResult,
    SignalLatencyResult,
    TorqueSpeedMapFitResult,
    apply_identification_results,
    characterize_encoder,
    estimate_signal_latency,
    fit_belt_damping_from_existing_results,
    fit_belt_damping_from_ringdown,
    fit_friction,
    fit_torque_speed_map,
)
from .plant_schema import ActuatorPlantParameters, TorqueSpeedPoint
from .plant_simulator import PlantSimulationTrace, TwoInertiaActuatorSimulator
from .plant_telemetry import derive_telemetry_dynamics, fit_output_inertia_from_telemetry
from .plant_validation import PlantValidationReport, compare_traces, telemetry_to_trace


@dataclass
class GuidedCharacterizationSummary:
    """Serializable snapshot of the fitted plant and workflow results."""

    plant: ActuatorPlantParameters
    stiffness_nm_per_rad: float | None = None
    friction: FrictionFitResult | None = None
    output_inertia: InertiaFitResult | None = None
    motor_inertia_derived_kg_m2: float | None = None
    belt_damping: BeltDampingFitResult | None = None
    torque_speed: TorqueSpeedMapFitResult | None = None
    latency: SignalLatencyResult | None = None
    motor_encoder: EncoderCharacterizationResult | None = None
    output_encoder: EncoderCharacterizationResult | None = None
    validation: PlantValidationReport | None = None

    def as_dict(self) -> dict:
        def convert(value):
            if value is None:
                return None
            if hasattr(value, "as_dict"):
                return value.as_dict()
            if hasattr(value, "__dataclass_fields__"):
                return asdict(value)
            return value

        return {
            "plant": self.plant.as_dict(),
            "stiffness_nm_per_rad": self.stiffness_nm_per_rad,
            "friction": convert(self.friction),
            "output_inertia": convert(self.output_inertia),
            "motor_inertia_derived_kg_m2": self.motor_inertia_derived_kg_m2,
            "belt_damping": convert(self.belt_damping),
            "torque_speed": convert(self.torque_speed),
            "latency": convert(self.latency),
            "motor_encoder": convert(self.motor_encoder),
            "output_encoder": convert(self.output_encoder),
            "validation": convert(self.validation),
        }


def plant_from_calibration(
    calibration: CalibrationConfig,
    info: ActuatorInfo | None = None,
    *,
    motor_inertia_kg_m2: float = 1.0e-5,
    output_inertia_kg_m2: float = 1.0e-3,
    torque_speed_reference_current_a: float | None = None,
    torque_speed_reference_bus_voltage_v: float | None = None,
) -> ActuatorPlantParameters:
    """Create a physical-plant seed from the existing calibration object."""
    if abs(calibration.output_per_motor) < 1e-12:
        raise ValueError("ratio calibration is required before building a plant")
    stiffness = calibration.compliance_nm_per_rad
    plant = ActuatorPlantParameters(
        actuator_id="" if info is None else info.actuator_id,
        source="guided_characterization",
        gear_ratio_motor_per_output=abs(1.0 / calibration.output_per_motor),
        motor_inertia_kg_m2=float(motor_inertia_kg_m2),
        output_inertia_kg_m2=float(output_inertia_kg_m2),
        belt_stiffness_nm_per_rad=float(stiffness) if stiffness and stiffness > 0.0 else 20.0,
        torque_speed_reference_current_a=torque_speed_reference_current_a,
        torque_speed_reference_bus_voltage_v=torque_speed_reference_bus_voltage_v,
    )
    plant.validate()
    return plant


def apply_static_stiffness(
    plant: ActuatorPlantParameters,
    stiffness_nm_per_rad: float,
) -> ActuatorPlantParameters:
    if not math.isfinite(stiffness_nm_per_rad) or stiffness_nm_per_rad <= 0.0:
        raise ValueError("belt stiffness must be finite and positive")
    data = plant.as_dict()
    data["belt_stiffness_nm_per_rad"] = float(stiffness_nm_per_rad)
    return ActuatorPlantParameters.from_dict(data)


def derive_motor_inertia_from_relative_mode(
    *,
    belt_stiffness_nm_per_rad: float,
    resonance_frequency_hz: float,
    output_inertia_kg_m2: float,
    gear_ratio_motor_per_output: float,
) -> float:
    """Infer motor-side inertia from the measured two-inertia relative mode.

    The measured relative-mode inertia is ``K / omega_n^2`` and obeys
    ``1/J_eq = 1/J_motor_reflected + 1/J_output``.  Motor inertia is then
    obtained by undoing the N^2 reflection through the reduction.
    """
    if belt_stiffness_nm_per_rad <= 0.0:
        raise ValueError("belt stiffness must be positive")
    if resonance_frequency_hz <= 0.0:
        raise ValueError("resonance frequency must be positive")
    if output_inertia_kg_m2 <= 0.0:
        raise ValueError("output inertia must be positive")
    if gear_ratio_motor_per_output <= 0.0:
        raise ValueError("gear ratio must be positive")

    omega = 2.0 * math.pi * resonance_frequency_hz
    j_eq = belt_stiffness_nm_per_rad / (omega * omega)
    if j_eq >= output_inertia_kg_m2:
        raise ValueError(
            "relative-mode inertia is not smaller than output inertia; "
            "check stiffness, resonance frequency, and output inertia"
        )
    reflected_motor = 1.0 / (1.0 / j_eq - 1.0 / output_inertia_kg_m2)
    return reflected_motor / (gear_ratio_motor_per_output**2)


def _check_faults(client: ActuatorClient) -> None:
    faults = int(client.faults(timeout=1.0))
    if faults:
        raise ActuatorError(f"actuator fault active: {FaultFlags(faults)}")


def _sleep_capture(store: TelemetryStore, duration_s: float) -> list[TelemetrySample]:
    start = store.total_samples
    time.sleep(max(0.05, float(duration_s)))
    return store.samples_since(start)


def run_friction_velocity_sweep(
    client: ActuatorClient,
    store: TelemetryStore,
    plant: ActuatorPlantParameters,
    safety: SafetyLimits,
    *,
    output_speeds_rad_s: Sequence[float] = (-4.0, -2.0, -1.0, -0.5, 0.5, 1.0, 2.0, 4.0),
    settle_s: float = 0.75,
    capture_s: float = 0.65,
    acceleration_rad_s2: float = 8.0,
    external_load_torque_nm: float = 0.0,
    include_stribeck: bool = True,
) -> tuple[FrictionFitResult, list[TelemetrySample]]:
    """Run slow bidirectional steady-state segments and fit output friction."""
    plant.validate()
    safety.validate()
    client.set_mode(ActuatorMode.VELOCITY)
    client.start_stream()
    _check_faults(client)
    samples: list[TelemetrySample] = []
    accel_limit = min(abs(float(acceleration_rad_s2)), safety.max_accel_rad_s2)
    try:
        for target in output_speeds_rad_s:
            target = max(-safety.max_velocity_rad_s, min(float(target), safety.max_velocity_rad_s))
            if abs(target) < 0.05:
                continue
            client.set_velocity_target(target, accel_limit)
            time.sleep(max(0.05, settle_s))
            samples.extend(_sleep_capture(store, capture_s))
        client.set_velocity_target(0.0, accel_limit)
        time.sleep(0.35)
    finally:
        try:
            client.stop()
        except Exception:
            pass

    if len(samples) < 30:
        raise ActuatorError("not enough steady-state samples for friction fitting")
    dynamics = derive_telemetry_dynamics(samples, plant)
    resisting = (
        dynamics.belt_torque_nm
        - plant.output_inertia_kg_m2 * dynamics.output_acceleration_rad_s2
        - float(external_load_torque_nm)
    )
    accel_abs = np.abs(dynamics.output_acceleration_rad_s2)
    velocity_abs = np.abs(dynamics.output_velocity_rad_s)
    accel_threshold = max(float(np.percentile(accel_abs, 65)), 0.2)
    mask = (accel_abs <= accel_threshold) & (velocity_abs >= 0.05)
    if np.count_nonzero(mask) < 12:
        mask = velocity_abs >= 0.05
    result = fit_friction(
        dynamics.output_velocity_rad_s[mask],
        resisting[mask],
        include_stribeck=include_stribeck,
    )
    return result, samples


def run_output_inertia_excitation(
    client: ActuatorClient,
    store: TelemetryStore,
    plant: ActuatorPlantParameters,
    friction: FrictionFitResult | None,
    safety: SafetyLimits,
    *,
    target_output_speed_rad_s: float = 3.0,
    acceleration_rad_s2: float = 12.0,
    segment_s: float = 1.15,
    external_load_torque_nm: float = 0.0,
) -> tuple[InertiaFitResult, list[TelemetrySample]]:
    """Excite acceleration in both directions and fit output-side inertia."""
    plant.validate()
    safety.validate()
    target = min(abs(float(target_output_speed_rad_s)), safety.max_velocity_rad_s)
    accel = min(abs(float(acceleration_rad_s2)), safety.max_accel_rad_s2)
    client.set_mode(ActuatorMode.VELOCITY)
    client.start_stream()
    _check_faults(client)
    start = store.total_samples
    try:
        for speed in (target, -target, target, 0.0):
            client.set_velocity_target(speed, accel)
            time.sleep(max(0.15, segment_s))
    finally:
        try:
            client.set_velocity_target(0.0, accel)
            time.sleep(0.25)
            client.stop()
        except Exception:
            pass
    samples = store.samples_since(start)
    if len(samples) < 30:
        raise ActuatorError("not enough samples for inertia fitting")
    result = fit_output_inertia_from_telemetry(
        samples,
        plant,
        friction=friction,
        external_load_torque_nm=external_load_torque_nm,
    )
    return result, samples


def fit_damping_and_motor_inertia(
    plant: ActuatorPlantParameters,
    resonance: ResonanceResult,
    *,
    step_response: StepResponseResult | None = None,
) -> tuple[BeltDampingFitResult, float | None]:
    damping = fit_belt_damping_from_existing_results(
        resonance,
        belt_stiffness_nm_per_rad=plant.belt_stiffness_nm_per_rad,
        equivalent_inertia_kg_m2=None,
        step_response=step_response,
    )
    motor_inertia: float | None = None
    if resonance.peak_frequency_hz is not None:
        try:
            motor_inertia = derive_motor_inertia_from_relative_mode(
                belt_stiffness_nm_per_rad=plant.belt_stiffness_nm_per_rad,
                resonance_frequency_hz=float(resonance.peak_frequency_hz),
                output_inertia_kg_m2=plant.output_inertia_kg_m2,
                gear_ratio_motor_per_output=plant.gear_ratio_motor_per_output,
            )
        except ValueError:
            motor_inertia = None
    return damping, motor_inertia


def fit_ringdown_from_samples(
    samples: Iterable[TelemetrySample],
    plant: ActuatorPlantParameters,
    *,
    output_offset_rad: float = 0.0,
) -> BeltDampingFitResult:
    sample_list = list(samples)
    if len(sample_list) < 16:
        raise ValueError("not enough ring-down samples")
    t0 = sample_list[0].t_us
    t = np.array([(sample.t_us - t0) / 1_000_000.0 for sample in sample_list], dtype=float)
    ratio = plant.gear_ratio_motor_per_output
    deflection = np.array(
        [sample.motor_rad / ratio + output_offset_rad - sample.output_rad for sample in sample_list],
        dtype=float,
    )
    return fit_belt_damping_from_ringdown(
        t,
        deflection,
        belt_stiffness_nm_per_rad=plant.belt_stiffness_nm_per_rad,
        equivalent_inertia_kg_m2=plant.relative_mode_inertia_kg_m2,
    )


def parse_torque_speed_points(text: str) -> tuple[np.ndarray, np.ndarray]:
    """Parse newline-separated ``speed,torque`` measurements from the GUI."""
    speeds: list[float] = []
    torques: list[float] = []
    for line_number, raw in enumerate(str(text).splitlines(), start=1):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        parts = [part.strip() for part in line.replace(";", ",").split(",")]
        if len(parts) != 2:
            raise ValueError(f"torque-speed line {line_number} must be speed,torque")
        try:
            speed = float(parts[0])
            torque = float(parts[1])
        except ValueError as exc:
            raise ValueError(f"torque-speed line {line_number} is not numeric") from exc
        if not math.isfinite(speed) or not math.isfinite(torque):
            raise ValueError(f"torque-speed line {line_number} must be finite")
        if torque < 0.0:
            raise ValueError(f"torque-speed line {line_number} has negative torque")
        speeds.append(abs(speed))
        torques.append(torque)
    if len(speeds) < 3:
        raise ValueError("enter at least three measured torque-speed points")
    return np.asarray(speeds, dtype=float), np.asarray(torques, dtype=float)


def fit_torque_speed_text(
    text: str,
    *,
    bins: int = 10,
    quantile: float = 0.9,
) -> TorqueSpeedMapFitResult:
    speed, torque = parse_torque_speed_points(text)
    return fit_torque_speed_map(speed, torque, bins=bins, quantile=quantile, enforce_monotonic=True)


def estimate_latency_from_samples(
    samples: Iterable[TelemetrySample],
    *,
    max_lag_s: float = 0.25,
) -> SignalLatencyResult:
    sample_list = list(samples)
    if len(sample_list) < 16:
        raise ValueError("not enough samples for latency estimation")
    t0 = sample_list[0].t_us
    t = np.array([(sample.t_us - t0) / 1_000_000.0 for sample in sample_list], dtype=float)
    command = np.array([sample.cmd_pos for sample in sample_list], dtype=float)
    response = np.array([sample.motor_rad for sample in sample_list], dtype=float)
    return estimate_signal_latency(t, command, response, max_lag_s=max_lag_s)


def characterize_encoders_from_samples(
    samples: Iterable[TelemetrySample],
    *,
    motor_counts_per_rev: int = 4096,
    output_counts_per_rev: int = 4096,
) -> tuple[EncoderCharacterizationResult, EncoderCharacterizationResult]:
    sample_list = list(samples)
    if len(sample_list) < 8:
        raise ValueError("not enough samples for encoder characterization")
    t0 = sample_list[0].t_us
    t = np.array([(sample.t_us - t0) / 1_000_000.0 for sample in sample_list], dtype=float)
    motor = characterize_encoder(
        t,
        [sample.motor_enc_raw for sample in sample_list],
        [sample.motor_rad for sample in sample_list],
        velocity_rad_s=[sample.motor_vel_rad_s for sample in sample_list],
        counts_per_rev=motor_counts_per_rev,
    )
    output = characterize_encoder(
        t,
        [sample.output_enc_raw for sample in sample_list],
        [sample.output_rad for sample in sample_list],
        velocity_rad_s=[sample.output_vel_rad_s for sample in sample_list],
        counts_per_rev=output_counts_per_rev,
    )
    return motor, output


def capture_static_encoder_samples(
    client: ActuatorClient,
    store: TelemetryStore,
    *,
    duration_s: float = 2.0,
) -> list[TelemetrySample]:
    client.start_stream()
    _check_faults(client)
    return _sleep_capture(store, duration_s)


def update_plant(
    plant: ActuatorPlantParameters,
    *,
    friction: FrictionFitResult | None = None,
    output_inertia: InertiaFitResult | None = None,
    motor_inertia_kg_m2: float | None = None,
    belt_damping: BeltDampingFitResult | None = None,
    torque_speed: TorqueSpeedMapFitResult | None = None,
    latency: SignalLatencyResult | None = None,
    motor_encoder: EncoderCharacterizationResult | None = None,
    output_encoder: EncoderCharacterizationResult | None = None,
) -> ActuatorPlantParameters:
    fitted = apply_identification_results(
        plant,
        friction=friction,
        output_inertia=output_inertia,
        belt_damping=belt_damping,
        torque_speed=torque_speed,
        command_latency=latency,
        motor_encoder=motor_encoder,
        output_encoder=output_encoder,
    )
    if motor_inertia_kg_m2 is not None and motor_inertia_kg_m2 > 0.0:
        data = fitted.as_dict()
        data["motor_inertia_kg_m2"] = float(motor_inertia_kg_m2)
        fitted = ActuatorPlantParameters.from_dict(data)
    return fitted


def mechanical_replay_validation(
    samples: Iterable[TelemetrySample],
    plant: ActuatorPlantParameters,
) -> tuple[PlantSimulationTrace, PlantValidationReport]:
    """Validate mechanics on a hold-out trace using encoder-inferred motor torque.

    This is intentionally called *mechanical replay* rather than full command-to-
    motion validation: the input torque is inferred from the measured dual-
    encoder drivetrain state.  It validates inertia/compliance/friction dynamics
    without pretending phase current is a calibrated stepper torque command.
    """
    sample_list = list(samples)
    if len(sample_list) < 16:
        raise ValueError("not enough hold-out samples for validation")
    measured = telemetry_to_trace(sample_list)
    dynamics = derive_telemetry_dynamics(sample_list, plant)
    torque = dynamics.estimated_motor_torque_to_drivetrain_nm
    simulator = TwoInertiaActuatorSimulator(plant)
    simulator.reset(
        motor_angle_rad=float(measured["motor_angle_rad"][0]),
        motor_velocity_rad_s=float(measured["motor_velocity_rad_s"][0]),
        output_angle_rad=float(measured["output_angle_rad"][0]),
        output_velocity_rad_s=float(measured["output_velocity_rad_s"][0]),
    )
    simulated = simulator.simulate(measured["time_s"], torque, reset=False)
    report = compare_traces(measured, simulated)
    return simulated, report


def save_workflow_artifacts(
    folder: str | Path,
    plant: ActuatorPlantParameters,
    summary: GuidedCharacterizationSummary,
) -> tuple[Path, Path]:
    target = Path(folder)
    target.mkdir(parents=True, exist_ok=True)
    plant_path = target / "plant.json"
    summary_path = target / "plant_characterization.json"
    plant.to_json(plant_path)
    summary_path.write_text(json.dumps(summary.as_dict(), indent=2) + "\n", encoding="utf-8")
    return plant_path, summary_path
