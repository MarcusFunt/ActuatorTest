"""System-identification helpers for the belted actuator digital twin.

The functions in this module operate on ordinary NumPy arrays so they can be
used from the existing bench tests, notebooks, CSV replays, or future UI flows.
They intentionally estimate small, interpretable models rather than hiding the
plant behind a black-box neural network.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
from typing import Iterable, Sequence

import numpy as np
from scipy import optimize, signal

from .actuator_analysis import ResonanceResult, StepResponseResult
from .plant_schema import (
    ActuatorPlantParameters,
    EncoderModelParameters,
    TorqueSpeedPoint,
)


@dataclass
class FrictionFitResult:
    coulomb_friction_nm: float
    viscous_friction_nm_s_per_rad: float
    static_friction_nm: float
    stribeck_velocity_rad_s: float
    smoothing_velocity_rad_s: float
    rmse_nm: float
    r_squared: float
    sample_count: int
    model: str = "stribeck"

    def torque_nm(self, velocity_rad_s: np.ndarray | float) -> np.ndarray:
        velocity = np.asarray(velocity_rad_s, dtype=float)
        smooth_sign = np.tanh(velocity / self.smoothing_velocity_rad_s)
        stribeck = self.coulomb_friction_nm + (
            self.static_friction_nm - self.coulomb_friction_nm
        ) * np.exp(-np.square(np.abs(velocity) / self.stribeck_velocity_rad_s))
        return smooth_sign * stribeck + self.viscous_friction_nm_s_per_rad * velocity


@dataclass
class InertiaFitResult:
    inertia_kg_m2: float
    torque_bias_nm: float
    rmse_nm: float
    r_squared: float
    acceleration_rms_rad_s2: float
    sample_count: int


@dataclass
class BeltDampingFitResult:
    damping_nm_s_per_rad: float
    damping_ratio: float
    equivalent_inertia_kg_m2: float
    natural_frequency_hz: float
    source: str


@dataclass
class TorqueSpeedMapFitResult:
    points: list[TorqueSpeedPoint]
    sample_count: int
    quantile: float
    bins: int
    monotonic_enforced: bool


@dataclass
class LatencyJitterResult:
    latency_s: float
    jitter_std_s: float
    p05_s: float
    p95_s: float
    minimum_s: float
    maximum_s: float
    sample_count: int


@dataclass
class SignalLatencyResult:
    latency_s: float
    correlation: float
    sample_period_s: float
    sample_jitter_std_s: float
    sample_count: int


@dataclass
class EncoderCharacterizationResult:
    counts_per_rev: int
    quantization_step_rad: float
    noise_std_rad: float
    bias_rad: float
    sample_period_s: float
    sample_jitter_std_s: float
    sample_count: int

    def to_parameters(self, *, latency_s: float = 0.0, jitter_std_s: float | None = None) -> EncoderModelParameters:
        return EncoderModelParameters(
            counts_per_rev=self.counts_per_rev,
            noise_std_rad=self.noise_std_rad,
            bias_rad=self.bias_rad,
            sample_period_s=self.sample_period_s,
            latency_s=float(latency_s),
            jitter_std_s=self.sample_jitter_std_s if jitter_std_s is None else float(jitter_std_s),
        )


def _as_matching_arrays(*arrays: Sequence[float] | np.ndarray) -> list[np.ndarray]:
    converted = [np.asarray(array, dtype=float).reshape(-1) for array in arrays]
    if not converted or len(converted[0]) == 0:
        raise ValueError("at least one sample is required")
    length = len(converted[0])
    if any(len(array) != length for array in converted):
        raise ValueError("input arrays must have matching lengths")
    mask = np.ones(length, dtype=bool)
    for array in converted:
        mask &= np.isfinite(array)
    filtered = [array[mask] for array in converted]
    if len(filtered[0]) < 3:
        raise ValueError("at least three finite samples are required")
    return filtered


def _r_squared(measured: np.ndarray, predicted: np.ndarray) -> float:
    residual = measured - predicted
    ss_res = float(np.sum(np.square(residual)))
    centered = measured - float(np.mean(measured))
    ss_tot = float(np.sum(np.square(centered)))
    if ss_tot <= 1e-18:
        return 1.0 if ss_res <= 1e-18 else 0.0
    return 1.0 - ss_res / ss_tot


def fit_friction(
    velocity_rad_s: Sequence[float] | np.ndarray,
    resisting_torque_nm: Sequence[float] | np.ndarray,
    *,
    include_stribeck: bool = True,
    smoothing_velocity_rad_s: float | None = None,
) -> FrictionFitResult:
    """Fit Coulomb + viscous friction, optionally with a Stribeck term.

    ``resisting_torque_nm`` should use the same sign as shaft velocity: positive
    speed should have a positive *magnitude of torque required to overcome
    friction*.  In a free-decay experiment, negate the measured friction torque
    before passing it here if necessary.
    """
    velocity, torque = _as_matching_arrays(velocity_rad_s, resisting_torque_nm)
    if float(np.ptp(velocity)) < 1e-6:
        raise ValueError("velocity range is too small for friction identification")

    speed_scale = max(float(np.percentile(np.abs(velocity), 20)), 1e-3)
    smoothing = float(smoothing_velocity_rad_s or max(speed_scale * 0.1, 1e-4))
    sign_basis = np.tanh(velocity / smoothing)

    nonzero = np.abs(velocity) > max(smoothing, 1e-4)
    if np.count_nonzero(nonzero) < 3:
        raise ValueError("not enough non-zero-velocity samples for friction fitting")

    high_speed = np.abs(velocity) >= float(np.percentile(np.abs(velocity[nonzero]), 70))
    initial_coulomb = max(0.0, float(np.median(np.abs(torque[nonzero]))))
    initial_viscous = 0.0
    if np.count_nonzero(high_speed) >= 3:
        design = np.column_stack([sign_basis[high_speed], velocity[high_speed]])
        coefficients, *_ = np.linalg.lstsq(design, torque[high_speed], rcond=None)
        initial_coulomb = max(0.0, float(coefficients[0]))
        initial_viscous = max(0.0, float(coefficients[1]))

    if include_stribeck:
        initial_static_extra = max(initial_coulomb * 0.15, 1e-5)
        initial_v_s = max(float(np.percentile(np.abs(velocity[nonzero]), 25)), 1e-3)

        def residual(x: np.ndarray) -> np.ndarray:
            coulomb, viscous, static_extra, v_s = x
            static = coulomb + static_extra
            curve = coulomb + static_extra * np.exp(-np.square(np.abs(velocity) / v_s))
            predicted = sign_basis * curve + viscous * velocity
            return predicted - torque

        result = optimize.least_squares(
            residual,
            x0=np.array([initial_coulomb, initial_viscous, initial_static_extra, initial_v_s]),
            bounds=(
                np.array([0.0, 0.0, 0.0, 1e-5]),
                np.array([np.inf, np.inf, np.inf, max(float(np.max(np.abs(velocity))) * 2.0, 1e-3)]),
            ),
            loss="soft_l1",
        )
        coulomb, viscous, static_extra, v_s = [float(value) for value in result.x]
        static = coulomb + static_extra
        model_name = "stribeck"
    else:
        design = np.column_stack([sign_basis, velocity])
        coefficients, *_ = np.linalg.lstsq(design, torque, rcond=None)
        coulomb = max(0.0, float(coefficients[0]))
        viscous = max(0.0, float(coefficients[1]))
        static = coulomb
        v_s = max(float(np.percentile(np.abs(velocity[nonzero]), 25)), smoothing)
        model_name = "coulomb_viscous"

    curve = coulomb + (static - coulomb) * np.exp(-np.square(np.abs(velocity) / v_s))
    predicted = sign_basis * curve + viscous * velocity
    rmse = float(math.sqrt(np.mean(np.square(predicted - torque))))
    return FrictionFitResult(
        coulomb_friction_nm=coulomb,
        viscous_friction_nm_s_per_rad=viscous,
        static_friction_nm=static,
        stribeck_velocity_rad_s=v_s,
        smoothing_velocity_rad_s=smoothing,
        rmse_nm=rmse,
        r_squared=float(_r_squared(torque, predicted)),
        sample_count=len(velocity),
        model=model_name,
    )


def fit_inertia(
    time_s: Sequence[float] | np.ndarray,
    velocity_rad_s: Sequence[float] | np.ndarray,
    applied_torque_nm: Sequence[float] | np.ndarray,
    *,
    friction: FrictionFitResult | None = None,
    minimum_acceleration_rad_s2: float | None = None,
) -> InertiaFitResult:
    """Fit ``tau = J * alpha + bias`` after subtracting fitted friction."""
    time, velocity, torque = _as_matching_arrays(time_s, velocity_rad_s, applied_torque_nm)
    order = np.argsort(time)
    time, velocity, torque = time[order], velocity[order], torque[order]
    if np.any(np.diff(time) <= 0.0):
        raise ValueError("time_s must be strictly increasing")

    acceleration = np.gradient(velocity, time, edge_order=2 if len(time) >= 3 else 1)
    corrected = torque.copy()
    if friction is not None:
        corrected = corrected - friction.torque_nm(velocity)

    threshold = minimum_acceleration_rad_s2
    if threshold is None:
        threshold = max(float(np.percentile(np.abs(acceleration), 20)), 1e-5)
    mask = np.abs(acceleration) >= float(threshold)
    if np.count_nonzero(mask) < 3:
        mask = np.ones_like(acceleration, dtype=bool)

    design = np.column_stack([acceleration[mask], np.ones(np.count_nonzero(mask))])
    coefficients, *_ = np.linalg.lstsq(design, corrected[mask], rcond=None)
    inertia = max(0.0, float(coefficients[0]))
    bias = float(coefficients[1])
    predicted = inertia * acceleration[mask] + bias
    measured = corrected[mask]
    rmse = float(math.sqrt(np.mean(np.square(predicted - measured))))
    return InertiaFitResult(
        inertia_kg_m2=inertia,
        torque_bias_nm=bias,
        rmse_nm=rmse,
        r_squared=float(_r_squared(measured, predicted)),
        acceleration_rms_rad_s2=float(math.sqrt(np.mean(np.square(acceleration[mask])))),
        sample_count=int(np.count_nonzero(mask)),
    )


def fit_belt_damping_from_ringdown(
    time_s: Sequence[float] | np.ndarray,
    relative_deflection_rad: Sequence[float] | np.ndarray,
    *,
    belt_stiffness_nm_per_rad: float,
    equivalent_inertia_kg_m2: float,
) -> BeltDampingFitResult:
    """Estimate damping from the exponential envelope of a free/ring-down mode."""
    time, deflection = _as_matching_arrays(time_s, relative_deflection_rad)
    order = np.argsort(time)
    time, deflection = time[order], deflection[order]
    if np.any(np.diff(time) <= 0.0):
        raise ValueError("time_s must be strictly increasing")
    if belt_stiffness_nm_per_rad <= 0.0 or equivalent_inertia_kg_m2 <= 0.0:
        raise ValueError("stiffness and equivalent inertia must be positive")

    detrended = signal.detrend(deflection)
    envelope = np.abs(signal.hilbert(detrended))
    floor = max(float(np.max(envelope)) * 0.05, 1e-12)
    mask = envelope > floor
    if np.count_nonzero(mask) < 8:
        raise ValueError("ring-down envelope is too small for a damping fit")
    slope, _intercept = np.polyfit(time[mask], np.log(envelope[mask]), 1)
    decay_rate = max(0.0, -float(slope))
    omega_n = math.sqrt(belt_stiffness_nm_per_rad / equivalent_inertia_kg_m2)
    damping_ratio = min(0.999, decay_rate / omega_n)
    damping = 2.0 * damping_ratio * math.sqrt(
        belt_stiffness_nm_per_rad * equivalent_inertia_kg_m2
    )
    return BeltDampingFitResult(
        damping_nm_s_per_rad=float(damping),
        damping_ratio=float(damping_ratio),
        equivalent_inertia_kg_m2=float(equivalent_inertia_kg_m2),
        natural_frequency_hz=float(omega_n / (2.0 * math.pi)),
        source="ringdown_envelope",
    )


def fit_belt_damping_from_existing_results(
    resonance: ResonanceResult,
    *,
    belt_stiffness_nm_per_rad: float,
    equivalent_inertia_kg_m2: float | None = None,
    step_response: StepResponseResult | None = None,
) -> BeltDampingFitResult:
    """Turn the existing resonance/step analysis into a physical belt damping value.

    The step-response damping-ratio estimate is preferred when available.  If it
    is absent, the resonance Q factor is converted with ``zeta ~= 1/(2Q)``.
    When equivalent inertia is not known, it is inferred from measured natural
    frequency and the fitted belt stiffness.
    """
    if resonance.peak_frequency_hz is None or resonance.peak_frequency_hz <= 0.0:
        raise ValueError("resonance peak frequency is required")
    if belt_stiffness_nm_per_rad <= 0.0:
        raise ValueError("belt stiffness must be positive")

    frequency = float(resonance.peak_frequency_hz)
    omega_n = 2.0 * math.pi * frequency
    j_eq = (
        float(equivalent_inertia_kg_m2)
        if equivalent_inertia_kg_m2 is not None
        else belt_stiffness_nm_per_rad / (omega_n * omega_n)
    )
    if j_eq <= 0.0:
        raise ValueError("equivalent inertia must be positive")

    damping_ratio: float | None = None
    source = "resonance_q"
    if step_response is not None and step_response.damping_estimate is not None:
        damping_ratio = float(step_response.damping_estimate)
        source = "step_response_damping_ratio"
    elif resonance.q_factor is not None and resonance.q_factor > 0.0:
        damping_ratio = 1.0 / (2.0 * float(resonance.q_factor))
    if damping_ratio is None:
        raise ValueError("step-response damping estimate or resonance Q factor is required")

    damping_ratio = max(0.0, min(damping_ratio, 0.999))
    damping = 2.0 * damping_ratio * math.sqrt(belt_stiffness_nm_per_rad * j_eq)
    return BeltDampingFitResult(
        damping_nm_s_per_rad=float(damping),
        damping_ratio=float(damping_ratio),
        equivalent_inertia_kg_m2=float(j_eq),
        natural_frequency_hz=frequency,
        source=source,
    )


def fit_torque_speed_map(
    motor_speed_rad_s: Sequence[float] | np.ndarray,
    available_torque_nm: Sequence[float] | np.ndarray,
    *,
    bins: int = 12,
    quantile: float = 0.9,
    enforce_monotonic: bool = True,
) -> TorqueSpeedMapFitResult:
    """Compress measured stall/limit samples into a usable torque-speed envelope."""
    speed, torque = _as_matching_arrays(motor_speed_rad_s, available_torque_nm)
    speed = np.abs(speed)
    torque = np.abs(torque)
    if not 0.5 <= quantile <= 1.0:
        raise ValueError("quantile must be between 0.5 and 1.0")
    bins = max(2, min(int(bins), len(speed)))
    order = np.argsort(speed)
    speed, torque = speed[order], torque[order]
    groups = [group for group in np.array_split(np.arange(len(speed)), bins) if len(group)]

    point_speeds: list[float] = []
    point_torques: list[float] = []
    for group in groups:
        point_speeds.append(float(np.median(speed[group])))
        point_torques.append(float(np.quantile(torque[group], quantile)))

    # Merge bins whose median speed is effectively identical.
    merged_speeds: list[float] = []
    merged_torques: list[float] = []
    for s, t in zip(point_speeds, point_torques):
        if merged_speeds and abs(s - merged_speeds[-1]) < 1e-9:
            merged_torques[-1] = max(merged_torques[-1], t)
        else:
            merged_speeds.append(s)
            merged_torques.append(t)

    if merged_speeds[0] > 1e-9:
        merged_speeds.insert(0, 0.0)
        merged_torques.insert(0, merged_torques[0])
    else:
        merged_speeds[0] = 0.0

    if enforce_monotonic:
        running = float("inf")
        for index, torque_value in enumerate(merged_torques):
            running = min(running, torque_value)
            merged_torques[index] = running

    points = [
        TorqueSpeedPoint(speed_rad_s=float(s), torque_nm=max(0.0, float(t)))
        for s, t in zip(merged_speeds, merged_torques)
    ]
    return TorqueSpeedMapFitResult(
        points=points,
        sample_count=len(speed),
        quantile=float(quantile),
        bins=len(points),
        monotonic_enforced=bool(enforce_monotonic),
    )


def characterize_latency_events(
    command_time_s: Sequence[float] | np.ndarray,
    response_time_s: Sequence[float] | np.ndarray,
) -> LatencyJitterResult:
    commands, responses = _as_matching_arrays(command_time_s, response_time_s)
    latency = responses - commands
    if np.any(latency < 0.0):
        raise ValueError("response timestamps must not precede command timestamps")
    return LatencyJitterResult(
        latency_s=float(np.mean(latency)),
        jitter_std_s=float(np.std(latency, ddof=1)) if len(latency) > 1 else 0.0,
        p05_s=float(np.percentile(latency, 5)),
        p95_s=float(np.percentile(latency, 95)),
        minimum_s=float(np.min(latency)),
        maximum_s=float(np.max(latency)),
        sample_count=len(latency),
    )


def estimate_signal_latency(
    time_s: Sequence[float] | np.ndarray,
    command_signal: Sequence[float] | np.ndarray,
    response_signal: Sequence[float] | np.ndarray,
    *,
    max_lag_s: float = 0.25,
) -> SignalLatencyResult:
    """Estimate command->response delay with normalized cross-correlation."""
    time, command, response = _as_matching_arrays(time_s, command_signal, response_signal)
    order = np.argsort(time)
    time, command, response = time[order], command[order], response[order]
    dt_values = np.diff(time)
    if np.any(dt_values <= 0.0):
        raise ValueError("time_s must be strictly increasing")
    dt = float(np.median(dt_values))
    jitter = float(np.std(dt_values - dt, ddof=1)) if len(dt_values) > 1 else 0.0

    # Resample to a uniform grid so the lag index has a clear physical meaning.
    uniform_time = np.arange(time[0], time[-1] + 0.5 * dt, dt)
    x = np.interp(uniform_time, time, command)
    y = np.interp(uniform_time, time, response)
    x = signal.detrend(x)
    y = signal.detrend(y)
    x_std = float(np.std(x))
    y_std = float(np.std(y))
    if x_std <= 1e-12 or y_std <= 1e-12:
        raise ValueError("command and response signals must vary")
    x /= x_std
    y /= y_std

    correlation = signal.correlate(y, x, mode="full", method="auto") / len(x)
    lags = signal.correlation_lags(len(y), len(x), mode="full")
    max_lag_samples = max(1, int(abs(max_lag_s) / dt))
    mask = (lags >= 0) & (lags <= max_lag_samples)
    if not np.any(mask):
        raise ValueError("max_lag_s is too small for the sample period")
    masked_indices = np.where(mask)[0]
    index = int(masked_indices[np.argmax(correlation[mask])])
    lag_samples = int(lags[index])
    return SignalLatencyResult(
        latency_s=float(lag_samples * dt),
        correlation=float(correlation[index]),
        sample_period_s=dt,
        sample_jitter_std_s=jitter,
        sample_count=len(time),
    )


def characterize_encoder(
    time_s: Sequence[float] | np.ndarray,
    raw_counts: Sequence[float] | np.ndarray,
    angle_rad: Sequence[float] | np.ndarray,
    *,
    velocity_rad_s: Sequence[float] | np.ndarray | None = None,
    counts_per_rev: int | None = None,
    reference_angle_rad: Sequence[float] | np.ndarray | None = None,
    static_velocity_threshold_rad_s: float = 0.02,
) -> EncoderCharacterizationResult:
    """Estimate encoder resolution, sampling jitter, residual noise, and bias.

    For wrapped raw counters, pass ``counts_per_rev`` explicitly.  When omitted,
    resolution is estimated from a linear raw-count-to-angle fit and is therefore
    intended for unwrapped captures or captures that do not cross the wrap.
    """
    time, raw, angle = _as_matching_arrays(time_s, raw_counts, angle_rad)
    order = np.argsort(time)
    time, raw, angle = time[order], raw[order], angle[order]
    dt_values = np.diff(time)
    if np.any(dt_values <= 0.0):
        raise ValueError("time_s must be strictly increasing")
    sample_period = float(np.median(dt_values))
    sample_jitter = float(np.std(dt_values - sample_period, ddof=1)) if len(dt_values) > 1 else 0.0

    if counts_per_rev is None:
        if float(np.ptp(raw)) < 2.0 or float(np.ptp(angle)) < 1e-4:
            raise ValueError("encoder moved too little to infer counts_per_rev")
        design = np.column_stack([raw, np.ones_like(raw)])
        slope, _offset = np.linalg.lstsq(design, angle, rcond=None)[0]
        if abs(float(slope)) < 1e-12:
            raise ValueError("could not infer encoder resolution")
        inferred = int(round(abs(2.0 * math.pi / float(slope))))
        counts_per_rev = max(2, inferred)
    else:
        counts_per_rev = int(counts_per_rev)
        if counts_per_rev < 2:
            raise ValueError("counts_per_rev must be at least 2")

    step = 2.0 * math.pi / counts_per_rev
    # Fit one constant phase/zero offset between raw counts and reported angle.
    predicted = raw * step
    offset = float(np.median(angle - predicted))
    residual = angle - (predicted + offset)

    if velocity_rad_s is not None:
        velocity = np.asarray(velocity_rad_s, dtype=float).reshape(-1)
        if len(velocity) != len(order):
            raise ValueError("velocity_rad_s must match the original sample count")
        velocity = velocity[order]
        static_mask = np.abs(velocity) <= static_velocity_threshold_rad_s
        noise_data = residual[static_mask] if np.count_nonzero(static_mask) >= 3 else residual
    else:
        noise_data = residual
    noise_std = float(np.std(noise_data, ddof=1)) if len(noise_data) > 1 else 0.0

    bias = 0.0
    if reference_angle_rad is not None:
        reference = np.asarray(reference_angle_rad, dtype=float).reshape(-1)
        if len(reference) != len(order):
            raise ValueError("reference_angle_rad must match the original sample count")
        reference = reference[order]
        bias = float(np.mean(angle - reference))

    return EncoderCharacterizationResult(
        counts_per_rev=counts_per_rev,
        quantization_step_rad=step,
        noise_std_rad=noise_std,
        bias_rad=bias,
        sample_period_s=sample_period,
        sample_jitter_std_s=sample_jitter,
        sample_count=len(time),
    )


def apply_identification_results(
    base: ActuatorPlantParameters,
    *,
    friction: FrictionFitResult | None = None,
    motor_inertia: InertiaFitResult | None = None,
    output_inertia: InertiaFitResult | None = None,
    belt_damping: BeltDampingFitResult | None = None,
    torque_speed: TorqueSpeedMapFitResult | None = None,
    command_latency: LatencyJitterResult | SignalLatencyResult | None = None,
    motor_encoder: EncoderCharacterizationResult | None = None,
    output_encoder: EncoderCharacterizationResult | None = None,
) -> ActuatorPlantParameters:
    """Return a validated plant parameter object updated with fitted results."""
    updates: dict[str, object] = {}
    if friction is not None:
        updates.update(
            coulomb_friction_nm=friction.coulomb_friction_nm,
            viscous_friction_nm_s_per_rad=friction.viscous_friction_nm_s_per_rad,
            static_friction_nm=friction.static_friction_nm,
            stribeck_velocity_rad_s=friction.stribeck_velocity_rad_s,
            friction_smoothing_velocity_rad_s=friction.smoothing_velocity_rad_s,
        )
    if motor_inertia is not None:
        updates["motor_inertia_kg_m2"] = motor_inertia.inertia_kg_m2
    if output_inertia is not None:
        updates["output_inertia_kg_m2"] = output_inertia.inertia_kg_m2
    if belt_damping is not None:
        updates["belt_damping_nm_s_per_rad"] = belt_damping.damping_nm_s_per_rad
    if torque_speed is not None:
        updates["motor_torque_speed_map"] = list(torque_speed.points)
    if command_latency is not None:
        updates["command_latency_s"] = command_latency.latency_s
        jitter = getattr(command_latency, "jitter_std_s", None)
        if jitter is not None:
            updates["command_jitter_std_s"] = float(jitter)
    if motor_encoder is not None:
        updates["motor_encoder"] = motor_encoder.to_parameters()
    if output_encoder is not None:
        updates["output_encoder"] = output_encoder.to_parameters()

    fitted = replace(base, **updates)
    fitted.validate()
    return fitted
