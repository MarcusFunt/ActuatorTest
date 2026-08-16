"""Derive physical drivetrain quantities directly from dual-encoder telemetry."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Sequence

import numpy as np

from .actuator_data import TelemetrySample
from .plant_identification import FrictionFitResult, InertiaFitResult, fit_friction, fit_inertia
from .plant_schema import ActuatorPlantParameters


@dataclass
class TelemetryDerivedDynamics:
    time_s: np.ndarray
    motor_angle_rad: np.ndarray
    output_angle_rad: np.ndarray
    motor_velocity_rad_s: np.ndarray
    output_velocity_rad_s: np.ndarray
    motor_acceleration_rad_s2: np.ndarray
    output_acceleration_rad_s2: np.ndarray
    belt_deflection_rad: np.ndarray
    belt_deflection_rate_rad_s: np.ndarray
    belt_torque_nm: np.ndarray
    estimated_motor_torque_to_drivetrain_nm: np.ndarray


def derive_telemetry_dynamics(
    samples: Iterable[TelemetrySample],
    parameters: ActuatorPlantParameters,
    *,
    output_offset_rad: float = 0.0,
) -> TelemetryDerivedDynamics:
    """Convert synchronized motor/output encoders into drivetrain state estimates.

    The repository's older ``compute_deflection`` helper uses ``output -
    predicted_output``.  The physical plant uses the opposite sign because that
    makes positive motor lead produce positive output-driving belt torque:

    ``delta = motor/N + offset - output``.
    """
    sample_list = list(samples)
    if len(sample_list) < 3:
        raise ValueError("at least three telemetry samples are required")
    parameters.validate()

    base_t_us = sample_list[0].t_us
    time_s = np.array([(sample.t_us - base_t_us) / 1_000_000.0 for sample in sample_list], dtype=float)
    if np.any(np.diff(time_s) <= 0.0):
        raise ValueError("telemetry timestamps must be strictly increasing")

    motor_angle = np.array([sample.motor_rad for sample in sample_list], dtype=float)
    output_angle = np.array([sample.output_rad for sample in sample_list], dtype=float)
    motor_velocity = np.array([sample.motor_vel_rad_s for sample in sample_list], dtype=float)
    output_velocity = np.array([sample.output_vel_rad_s for sample in sample_list], dtype=float)
    ratio = parameters.gear_ratio_motor_per_output

    belt_deflection = motor_angle / ratio + float(output_offset_rad) - output_angle
    belt_deflection_rate = motor_velocity / ratio - output_velocity
    belt_torque = (
        parameters.belt_stiffness_nm_per_rad * belt_deflection
        + parameters.belt_damping_nm_s_per_rad * belt_deflection_rate
    )

    motor_acceleration = np.gradient(motor_velocity, time_s, edge_order=2)
    output_acceleration = np.gradient(output_velocity, time_s, edge_order=2)

    # Torque delivered by the motor to the driveline, before any unmodeled motor
    # internal friction.  This is useful for torque-speed-envelope experiments.
    estimated_motor_torque = (
        belt_torque / ratio + parameters.motor_inertia_kg_m2 * motor_acceleration
    )

    return TelemetryDerivedDynamics(
        time_s=time_s,
        motor_angle_rad=motor_angle,
        output_angle_rad=output_angle,
        motor_velocity_rad_s=motor_velocity,
        output_velocity_rad_s=output_velocity,
        motor_acceleration_rad_s2=motor_acceleration,
        output_acceleration_rad_s2=output_acceleration,
        belt_deflection_rad=belt_deflection,
        belt_deflection_rate_rad_s=belt_deflection_rate,
        belt_torque_nm=belt_torque,
        estimated_motor_torque_to_drivetrain_nm=estimated_motor_torque,
    )


def fit_output_friction_from_telemetry(
    samples: Iterable[TelemetrySample],
    parameters: ActuatorPlantParameters,
    *,
    output_offset_rad: float = 0.0,
    external_load_torque_nm: Sequence[float] | np.ndarray | float = 0.0,
    output_inertia_kg_m2: float | None = None,
    include_stribeck: bool = True,
) -> FrictionFitResult:
    """Fit output-side friction using belt torque inferred from dual encoders.

    This is most useful after belt stiffness/damping are known.  The resisting
    torque estimate is

    ``tau_f = tau_belt - J_output*alpha_output - tau_external``.
    """
    dynamics = derive_telemetry_dynamics(
        samples,
        parameters,
        output_offset_rad=output_offset_rad,
    )
    if np.isscalar(external_load_torque_nm):
        external = np.full(len(dynamics.time_s), float(external_load_torque_nm), dtype=float)
    else:
        external = np.asarray(external_load_torque_nm, dtype=float).reshape(-1)
        if len(external) != len(dynamics.time_s):
            raise ValueError("external_load_torque_nm must be scalar or match telemetry length")
    inertia = parameters.output_inertia_kg_m2 if output_inertia_kg_m2 is None else float(output_inertia_kg_m2)
    if inertia < 0.0:
        raise ValueError("output inertia must be non-negative")
    resisting_torque = (
        dynamics.belt_torque_nm
        - inertia * dynamics.output_acceleration_rad_s2
        - external
    )
    return fit_friction(
        dynamics.output_velocity_rad_s,
        resisting_torque,
        include_stribeck=include_stribeck,
    )


def fit_output_inertia_from_telemetry(
    samples: Iterable[TelemetrySample],
    parameters: ActuatorPlantParameters,
    *,
    friction: FrictionFitResult | None = None,
    output_offset_rad: float = 0.0,
    external_load_torque_nm: Sequence[float] | np.ndarray | float = 0.0,
) -> InertiaFitResult:
    """Fit output inertia using encoder-derived belt torque as the applied torque."""
    dynamics = derive_telemetry_dynamics(
        samples,
        parameters,
        output_offset_rad=output_offset_rad,
    )
    if np.isscalar(external_load_torque_nm):
        external = np.full(len(dynamics.time_s), float(external_load_torque_nm), dtype=float)
    else:
        external = np.asarray(external_load_torque_nm, dtype=float).reshape(-1)
        if len(external) != len(dynamics.time_s):
            raise ValueError("external_load_torque_nm must be scalar or match telemetry length")
    applied = dynamics.belt_torque_nm - external
    return fit_inertia(
        dynamics.time_s,
        dynamics.output_velocity_rad_s,
        applied,
        friction=friction,
    )
