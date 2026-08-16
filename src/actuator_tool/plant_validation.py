"""Validation helpers for comparing measured actuator traces to the digital twin."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Iterable, Mapping, Sequence

import numpy as np

from .actuator_data import TelemetrySample
from .plant_simulator import PlantSimulationTrace, TwoInertiaActuatorSimulator


@dataclass
class ChannelValidationMetric:
    rmse: float
    mae: float
    max_abs_error: float
    normalized_rmse: float | None
    correlation: float | None
    sample_count: int

    def as_dict(self) -> dict[str, float | int | None]:
        return asdict(self)


@dataclass
class PlantValidationReport:
    metrics: dict[str, ChannelValidationMetric]
    measured_duration_s: float
    simulated_duration_s: float
    sample_count: int

    def as_dict(self) -> dict:
        return {
            "metrics": {name: metric.as_dict() for name, metric in self.metrics.items()},
            "measured_duration_s": self.measured_duration_s,
            "simulated_duration_s": self.simulated_duration_s,
            "sample_count": self.sample_count,
        }

    @property
    def mean_normalized_rmse(self) -> float | None:
        values = [
            metric.normalized_rmse
            for metric in self.metrics.values()
            if metric.normalized_rmse is not None
        ]
        return None if not values else float(np.mean(values))


def telemetry_to_trace(samples: Iterable[TelemetrySample]) -> dict[str, np.ndarray]:
    sample_list = list(samples)
    if len(sample_list) < 2:
        raise ValueError("at least two telemetry samples are required")
    base_t_us = sample_list[0].t_us
    time_s = np.array([(sample.t_us - base_t_us) / 1_000_000.0 for sample in sample_list], dtype=float)
    if np.any(np.diff(time_s) <= 0.0):
        raise ValueError("telemetry timestamps must be strictly increasing")
    return {
        "time_s": time_s,
        "motor_angle_rad": np.array([sample.motor_rad for sample in sample_list], dtype=float),
        "output_angle_rad": np.array([sample.output_rad for sample in sample_list], dtype=float),
        "motor_velocity_rad_s": np.array([sample.motor_vel_rad_s for sample in sample_list], dtype=float),
        "output_velocity_rad_s": np.array([sample.output_vel_rad_s for sample in sample_list], dtype=float),
        "commanded_current_a": np.array([sample.commanded_current for sample in sample_list], dtype=float),
        "driver_current_a": np.array([sample.driver_current for sample in sample_list], dtype=float),
        "command_position_rad": np.array([sample.cmd_pos for sample in sample_list], dtype=float),
    }


def _metric(measured: np.ndarray, simulated: np.ndarray) -> ChannelValidationMetric:
    error = simulated - measured
    rmse = float(math.sqrt(np.mean(np.square(error))))
    mae = float(np.mean(np.abs(error)))
    max_abs = float(np.max(np.abs(error)))
    span = float(np.ptp(measured))
    normalized = None if span <= 1e-12 else rmse / span
    measured_std = float(np.std(measured))
    simulated_std = float(np.std(simulated))
    correlation = None
    if measured_std > 1e-12 and simulated_std > 1e-12:
        correlation = float(np.corrcoef(measured, simulated)[0, 1])
    return ChannelValidationMetric(
        rmse=rmse,
        mae=mae,
        max_abs_error=max_abs,
        normalized_rmse=normalized,
        correlation=correlation,
        sample_count=len(measured),
    )


def compare_traces(
    measured: Mapping[str, Sequence[float] | np.ndarray],
    simulated: Mapping[str, Sequence[float] | np.ndarray] | PlantSimulationTrace,
    *,
    channels: Sequence[str] = (
        "motor_angle_rad",
        "output_angle_rad",
        "motor_velocity_rad_s",
        "output_velocity_rad_s",
    ),
) -> PlantValidationReport:
    """Interpolate simulated channels onto measured timestamps and score them."""
    measured_time = np.asarray(measured["time_s"], dtype=float).reshape(-1)
    if len(measured_time) < 2 or np.any(np.diff(measured_time) <= 0.0):
        raise ValueError("measured time_s must be strictly increasing and contain at least two samples")

    if isinstance(simulated, PlantSimulationTrace):
        simulated_map = simulated.as_dict()
    else:
        simulated_map = simulated
    simulated_time = np.asarray(simulated_map["time_s"], dtype=float).reshape(-1)
    if len(simulated_time) < 2 or np.any(np.diff(simulated_time) <= 0.0):
        raise ValueError("simulated time_s must be strictly increasing and contain at least two samples")

    overlap_start = max(float(measured_time[0]), float(simulated_time[0]))
    overlap_end = min(float(measured_time[-1]), float(simulated_time[-1]))
    mask = (measured_time >= overlap_start) & (measured_time <= overlap_end)
    if np.count_nonzero(mask) < 2:
        raise ValueError("measured and simulated traces do not overlap")
    aligned_time = measured_time[mask]

    metrics: dict[str, ChannelValidationMetric] = {}
    for channel in channels:
        if channel not in measured or channel not in simulated_map:
            raise KeyError(f"channel {channel!r} missing from measured or simulated trace")
        measured_values = np.asarray(measured[channel], dtype=float).reshape(-1)
        simulated_values = np.asarray(simulated_map[channel], dtype=float).reshape(-1)
        if len(measured_values) != len(measured_time):
            raise ValueError(f"measured channel {channel!r} length does not match time_s")
        if len(simulated_values) != len(simulated_time):
            raise ValueError(f"simulated channel {channel!r} length does not match time_s")
        aligned_measured = measured_values[mask]
        aligned_simulated = np.interp(aligned_time, simulated_time, simulated_values)
        metrics[channel] = _metric(aligned_measured, aligned_simulated)

    return PlantValidationReport(
        metrics=metrics,
        measured_duration_s=float(aligned_time[-1] - aligned_time[0]),
        simulated_duration_s=float(simulated_time[-1] - simulated_time[0]),
        sample_count=len(aligned_time),
    )


def validate_simulator_against_telemetry(
    samples: Iterable[TelemetrySample],
    simulator: TwoInertiaActuatorSimulator,
    requested_motor_torque_nm: Sequence[float] | np.ndarray,
    *,
    load_torque_nm: Sequence[float] | np.ndarray | float = 0.0,
    channels: Sequence[str] = (
        "motor_angle_rad",
        "output_angle_rad",
        "motor_velocity_rad_s",
        "output_velocity_rad_s",
    ),
) -> tuple[PlantSimulationTrace, PlantValidationReport]:
    """Run a measured input sequence through the plant and compare state traces.

    The caller supplies the motor-side torque estimate because current alone is
    not a trustworthy torque measurement for a stepper.  Use a measured torque
    map, load cell, or other calibrated conversion before treating this as a
    quantitative validation.
    """
    measured = telemetry_to_trace(samples)
    torque = np.asarray(requested_motor_torque_nm, dtype=float).reshape(-1)
    if len(torque) != len(measured["time_s"]):
        raise ValueError("requested_motor_torque_nm must match telemetry sample count")
    simulated = simulator.simulate(
        measured["time_s"],
        torque,
        load_torque_nm=load_torque_nm,
        reset=True,
    )
    report = compare_traces(measured, simulated, channels=channels)
    return simulated, report
