"""Numerical analysis for actuator calibration and actuator test diagnostics."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Iterable, Sequence

import numpy as np
from scipy import signal

from .actuator_data import TelemetrySample


@dataclass
class CalibrationParameters:
    calibration_version: int = 1
    actuator_id: str = ""
    hardware_revision: str = ""
    firmware_version: str = ""
    motor_encoder_sign: int = 1
    output_encoder_sign: int = 1
    motor_encoder_zero: float = 0.0
    output_encoder_zero: float = 0.0
    output_per_motor: float = 1.0
    motor_per_output: float = 1.0
    output_offset_rad: float = 0.0
    backlash_motor_rad: float = 0.0
    backlash_output_rad: float = 0.0
    hysteresis_rad: float = 0.0
    compliance_nm_per_rad: float | None = None
    max_safe_velocity_rad_s: float = 8.0
    max_safe_accel_rad_s2: float = 60.0
    resonance_frequency_hz: float | None = None
    settling_time_s: float | None = None
    overshoot_percent: float | None = None
    pid_enabled: bool = False
    pid_kp: float = 0.0
    pid_ki: float = 0.0
    pid_kd: float = 0.0
    pid_i_limit_motor_rad: float = 0.05
    pid_output_limit_motor_rad: float = 0.25
    backlash_comp_enabled: bool = False
    resonance_derating_enabled: bool = False
    calibrated_at: str = ""

    def as_dict(self) -> dict[str, float | int | str | None]:
        return asdict(self)


@dataclass
class RatioFitResult:
    output_per_motor: float
    motor_per_output: float
    output_offset_rad: float
    residual_rms_rad: float
    residual_max_abs_rad: float
    sample_count: int
    hysteresis_rad: float = 0.0
    forward_ratio: float | None = None
    reverse_ratio: float | None = None
    forward_offset_rad: float | None = None
    reverse_offset_rad: float | None = None
    pass_fit: bool = True
    warning: str = ""

    def as_dict(self) -> dict[str, float | int | bool | str | None]:
        return asdict(self)


@dataclass
class EncoderSanityResult:
    motor_encoder_sign: int
    output_encoder_sign: int
    motor_delta_rad: float
    output_delta_rad: float
    motor_noise_rad: float
    output_noise_rad: float
    motor_discontinuity: bool
    output_discontinuity: bool
    frozen_motor: bool
    frozen_output: bool
    sample_count: int
    pass_test: bool
    warning: str = ""

    def as_dict(self) -> dict[str, float | int | bool | str]:
        return asdict(self)


@dataclass
class DeflectionAnalysisResult:
    sample_count: int
    peak_abs_rad: float
    rms_rad: float
    mean_rad: float
    positive_direction_mean_rad: float | None
    negative_direction_mean_rad: float | None
    velocity_correlation: float | None
    acceleration_correlation: float | None
    dominant_frequency_hz: float | None
    slip_detected: bool
    pass_test: bool
    warning: str = ""

    def as_dict(self) -> dict[str, float | int | bool | str | None]:
        return asdict(self)


@dataclass
class BacklashResult:
    backlash_motor_rad: float
    backlash_output_rad: float
    direction_asymmetry_rad: float
    repeatability_rad: float
    reversal_count: int
    sample_count: int
    pass_test: bool
    warning: str = ""

    def as_dict(self) -> dict[str, float | int | bool | str]:
        return asdict(self)


@dataclass
class StepResponseResult:
    sample_count: int
    initial_output_rad: float
    final_output_rad: float
    commanded_step_rad: float
    rise_time_s: float | None
    settling_time_s: float | None
    overshoot_percent: float
    undershoot_percent: float
    output_lag_s: float | None
    oscillation_frequency_hz: float | None
    damping_estimate: float | None
    stable: bool
    pass_test: bool
    warning: str = ""

    def as_dict(self) -> dict[str, float | int | bool | str | None]:
        return asdict(self)


@dataclass
class VelocityRampResult:
    sample_count: int
    max_abs_velocity_rad_s: float
    rms_velocity_tracking_error_rad_s: float
    peak_velocity_tracking_error_rad_s: float
    rms_deflection_rad: float
    peak_deflection_rad: float
    resonance_frequency_hz: float | None
    resonance_detected: bool
    recommended_velocity_limit_rad_s: float
    recommended_accel_limit_rad_s2: float
    pass_test: bool
    warning: str = ""

    def as_dict(self) -> dict[str, float | int | bool | str | None]:
        return asdict(self)


@dataclass
class ComplianceResult:
    sample_count: int
    torque_nm: float | None
    mean_deflection_rad: float
    stiffness_nm_per_rad: float | None
    relative_compliance_rad: float
    hysteresis_rad: float
    creep_rad: float
    pass_test: bool
    warning: str = ""

    def as_dict(self) -> dict[str, float | int | bool | str | None]:
        return asdict(self)


@dataclass
class ResonanceResult:
    sample_count: int
    sample_rate_hz: float
    start_frequency_hz: float
    end_frequency_hz: float
    peak_frequency_hz: float | None
    peak_gain: float | None
    peak_prominence_db: float | None
    bandwidth_hz: float | None
    q_factor: float | None
    rms_deflection_rad: float
    peak_deflection_rad: float
    pass_test: bool
    warning: str = ""

    def as_dict(self) -> dict[str, float | int | bool | str | None]:
        return asdict(self)


def _sample_list(samples: Iterable[TelemetrySample]) -> list[TelemetrySample]:
    return list(samples)


def _as_arrays(samples: Iterable[TelemetrySample]) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    sample_list = list(samples)
    if not sample_list:
        return np.array([]), np.array([]), np.array([])
    motor = np.array([sample.motor_rad for sample in sample_list], dtype=float)
    output = np.array([sample.output_rad for sample in sample_list], dtype=float)
    motor_vel = np.array([sample.motor_vel_rad_s for sample in sample_list], dtype=float)
    return motor, output, motor_vel


def _time_array(samples: Sequence[TelemetrySample]) -> np.ndarray:
    if not samples:
        return np.array([], dtype=float)
    base = samples[0].t_us
    return np.array([(sample.t_us - base) / 1_000_000.0 for sample in samples], dtype=float)


def _series_arrays(
    samples: Iterable[TelemetrySample],
) -> tuple[list[TelemetrySample], np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    sample_list = _sample_list(samples)
    time_s = _time_array(sample_list)
    motor = np.array([sample.motor_rad for sample in sample_list], dtype=float)
    output = np.array([sample.output_rad for sample in sample_list], dtype=float)
    motor_vel = np.array([sample.motor_vel_rad_s for sample in sample_list], dtype=float)
    output_vel = np.array([sample.output_vel_rad_s for sample in sample_list], dtype=float)
    cmd_pos = np.array([sample.cmd_pos for sample in sample_list], dtype=float)
    return sample_list, time_s, motor, output, motor_vel, output_vel, cmd_pos


def _fit_line(x: np.ndarray, y: np.ndarray) -> tuple[float, float, np.ndarray]:
    if len(x) < 2:
        raise ValueError("at least two samples are required for line fit")
    design = np.column_stack([x, np.ones_like(x)])
    slope, offset = np.linalg.lstsq(design, y, rcond=None)[0]
    residuals = y - (slope * x + offset)
    return float(slope), float(offset), residuals


def compute_deflection(
    samples: Iterable[TelemetrySample],
    output_per_motor: float,
    output_offset_rad: float,
) -> np.ndarray:
    motor, output, _ = _as_arrays(samples)
    if len(motor) == 0:
        return np.array([])
    predicted = motor * float(output_per_motor) + float(output_offset_rad)
    return output - predicted


def predicted_output(
    samples: Iterable[TelemetrySample],
    output_per_motor: float,
    output_offset_rad: float,
) -> np.ndarray:
    motor, _, _ = _as_arrays(samples)
    if len(motor) == 0:
        return np.array([])
    return motor * float(output_per_motor) + float(output_offset_rad)


def analyze_deflection(
    samples: Iterable[TelemetrySample],
    output_per_motor: float,
    output_offset_rad: float,
    *,
    max_allowed_rms_rad: float = 0.02,
    max_allowed_peak_rad: float = 0.08,
    slip_threshold_rad: float = 0.12,
) -> DeflectionAnalysisResult:
    sample_list, time_s, _motor, _output, motor_vel, _output_vel, _cmd_pos = _series_arrays(samples)
    if len(sample_list) < 3:
        raise ValueError("at least three samples are required for deflection analysis")

    deflection = compute_deflection(sample_list, output_per_motor, output_offset_rad)
    rms = float(math.sqrt(np.mean(np.square(deflection))))
    peak_abs = float(np.max(np.abs(deflection)))
    mean = float(np.mean(deflection))
    positive = deflection[motor_vel > 1e-4]
    negative = deflection[motor_vel < -1e-4]
    positive_mean = float(np.mean(positive)) if len(positive) else None
    negative_mean = float(np.mean(negative)) if len(negative) else None

    accel = _gradient(motor_vel, time_s)
    velocity_correlation = _safe_corrcoef(deflection, motor_vel)
    acceleration_correlation = _safe_corrcoef(deflection, accel)
    dominant = dominant_frequency(time_s, deflection - mean)
    slip_detected = peak_abs >= slip_threshold_rad or bool(np.any(np.abs(np.diff(deflection)) >= slip_threshold_rad))

    warnings: list[str] = []
    if rms > max_allowed_rms_rad:
        warnings.append(f"deflection RMS {rms:.5f} rad exceeds {max_allowed_rms_rad:.5f} rad")
    if peak_abs > max_allowed_peak_rad:
        warnings.append(f"peak deflection {peak_abs:.5f} rad exceeds {max_allowed_peak_rad:.5f} rad")
    if slip_detected:
        warnings.append("possible slip or discontinuous lost motion detected")

    return DeflectionAnalysisResult(
        sample_count=len(sample_list),
        peak_abs_rad=peak_abs,
        rms_rad=rms,
        mean_rad=mean,
        positive_direction_mean_rad=positive_mean,
        negative_direction_mean_rad=negative_mean,
        velocity_correlation=velocity_correlation,
        acceleration_correlation=acceleration_correlation,
        dominant_frequency_hz=dominant,
        slip_detected=slip_detected,
        pass_test=not warnings,
        warning="; ".join(warnings),
    )


def fit_ratio(
    samples: Iterable[TelemetrySample],
    residual_threshold_rad: float = 0.02,
) -> RatioFitResult:
    motor, output, motor_vel = _as_arrays(samples)
    if len(motor) < 10:
        raise ValueError("at least 10 telemetry samples are required for ratio fitting")
    if float(np.ptp(motor)) < 1e-6:
        raise ValueError("motor angle did not move enough for ratio fitting")

    slope, offset, residuals = _fit_line(motor, output)
    rms = float(math.sqrt(np.mean(np.square(residuals))))
    max_abs = float(np.max(np.abs(residuals)))

    forward_ratio: float | None = None
    reverse_ratio: float | None = None
    forward_offset: float | None = None
    reverse_offset: float | None = None
    hysteresis = 0.0

    forward_mask = motor_vel > 1e-4
    reverse_mask = motor_vel < -1e-4
    if np.count_nonzero(forward_mask) >= 10:
        forward_ratio, forward_offset, _ = _fit_line(motor[forward_mask], output[forward_mask])
    if np.count_nonzero(reverse_mask) >= 10:
        reverse_ratio, reverse_offset, _ = _fit_line(motor[reverse_mask], output[reverse_mask])
    if forward_offset is not None and reverse_offset is not None:
        hysteresis = abs(float(forward_offset) - float(reverse_offset))

    warning = ""
    pass_fit = rms <= residual_threshold_rad
    if not pass_fit:
        warning = f"ratio fit residual RMS {rms:.5f} rad exceeds {residual_threshold_rad:.5f} rad"

    return RatioFitResult(
        output_per_motor=float(slope),
        motor_per_output=float(1.0 / slope),
        output_offset_rad=float(offset),
        residual_rms_rad=rms,
        residual_max_abs_rad=max_abs,
        sample_count=len(motor),
        hysteresis_rad=float(hysteresis),
        forward_ratio=None if forward_ratio is None else float(forward_ratio),
        reverse_ratio=None if reverse_ratio is None else float(reverse_ratio),
        forward_offset_rad=None if forward_offset is None else float(forward_offset),
        reverse_offset_rad=None if reverse_offset is None else float(reverse_offset),
        pass_fit=pass_fit,
        warning=warning,
    )


def analyze_backlash(
    samples: Iterable[TelemetrySample],
    output_per_motor: float,
    *,
    output_motion_threshold_rad: float = 0.001,
    max_allowed_backlash_motor_rad: float = 0.1,
) -> BacklashResult:
    sample_list, _time_s, motor, output, motor_vel, _output_vel, _cmd_pos = _series_arrays(samples)
    if len(sample_list) < 10:
        raise ValueError("at least 10 samples are required for backlash analysis")

    signs = np.sign(motor_vel)
    signs[np.abs(motor_vel) < 1e-4] = 0
    reversals: list[float] = []

    for idx in range(1, len(signs)):
        prev_sign = _last_nonzero(signs[:idx])
        new_sign = signs[idx]
        if prev_sign == 0 or new_sign == 0 or prev_sign == new_sign:
            continue
        start_motor = motor[idx]
        start_output = output[idx]
        for follow_idx in range(idx + 1, len(sample_list)):
            if signs[follow_idx] != new_sign:
                break
            if abs(output[follow_idx] - start_output) >= output_motion_threshold_rad:
                reversals.append(abs(float(motor[follow_idx] - start_motor)))
                break

    if not reversals:
        raise ValueError("no usable direction reversals found for backlash analysis")

    reversal_values = np.array(reversals, dtype=float)
    backlash_motor = float(np.median(reversal_values))
    backlash_output = float(abs(backlash_motor * output_per_motor))
    direction_asymmetry = float(np.max(reversal_values) - np.min(reversal_values))
    repeatability = float(np.std(reversal_values))

    warnings: list[str] = []
    if backlash_motor > max_allowed_backlash_motor_rad:
        warnings.append(
            f"backlash {backlash_motor:.5f} motor rad exceeds {max_allowed_backlash_motor_rad:.5f}"
        )
    if repeatability > max(backlash_motor * 0.5, 0.01):
        warnings.append("backlash repeatability is poor")

    return BacklashResult(
        backlash_motor_rad=backlash_motor,
        backlash_output_rad=backlash_output,
        direction_asymmetry_rad=direction_asymmetry,
        repeatability_rad=repeatability,
        reversal_count=len(reversals),
        sample_count=len(sample_list),
        pass_test=not warnings,
        warning="; ".join(warnings),
    )


def analyze_step_response(
    samples: Iterable[TelemetrySample],
    *,
    output_per_motor: float | None = None,
    output_offset_rad: float = 0.0,
    commanded_step_rad: float | None = None,
    settling_band_fraction: float = 0.02,
    max_allowed_overshoot_percent: float = 20.0,
    max_allowed_settling_time_s: float | None = None,
) -> StepResponseResult:
    sample_list, time_s, _motor, output, _motor_vel, _output_vel, cmd_pos = _series_arrays(samples)
    if len(sample_list) < 10:
        raise ValueError("at least 10 samples are required for step response analysis")

    initial = float(np.median(output[: max(2, len(output) // 10)]))
    measured_final = float(np.median(output[-max(2, len(output) // 10) :]))
    command = cmd_pos
    if output_per_motor is not None:
        command = cmd_pos * float(output_per_motor) + float(output_offset_rad)
    if commanded_step_rad is None:
        command_step = float(command[-1] - command[0])
        if abs(command_step) < 1e-9:
            command_step = float(np.max(command) - np.min(command))
    else:
        command_step = float(commanded_step_rad)
    final = initial + command_step if abs(command_step) >= 1e-9 else measured_final
    step = final - initial
    amplitude = abs(step)
    if amplitude < 1e-9:
        raise ValueError("output did not change enough for step response analysis")
    if abs(command_step) < 1e-9:
        command_step = measured_final - initial

    direction = 1.0 if step >= 0 else -1.0
    normalized = direction * (output - initial) / amplitude
    rise_start = _threshold_crossing_time(time_s, normalized, 0.1)
    rise_end = _threshold_crossing_time(time_s, normalized, 0.9)
    rise_time = rise_end - rise_start
    if math.isnan(rise_time) or rise_time < 0:
        rise_time_value: float | None = None
    else:
        rise_time_value = float(rise_time)

    band = amplitude * settling_band_fraction
    settling_time = _settling_time(time_s, output, final, band)
    peak = float(np.max(direction * (output - final)))
    trough = float(np.max(-direction * (output - initial)))
    overshoot = max(0.0, peak / amplitude * 100.0)
    undershoot = max(0.0, trough / amplitude * 100.0)
    output_lag = _output_lag(time_s, command, output)
    residual = output - final
    osc_freq = dominant_frequency(time_s, residual)
    damping = estimate_damping_ratio(time_s, residual)

    warnings: list[str] = []
    if overshoot > max_allowed_overshoot_percent:
        warnings.append(f"overshoot {overshoot:.2f}% exceeds {max_allowed_overshoot_percent:.2f}%")
    if settling_time is None:
        warnings.append("response did not settle within capture window")
    if max_allowed_settling_time_s is not None and settling_time is not None:
        if settling_time > max_allowed_settling_time_s:
            warnings.append(
                f"settling time {settling_time:.4f}s exceeds {max_allowed_settling_time_s:.4f}s"
            )
    stable = settling_time is not None and (osc_freq is None or overshoot <= max_allowed_overshoot_percent * 2.0)

    return StepResponseResult(
        sample_count=len(sample_list),
        initial_output_rad=initial,
        final_output_rad=final,
        commanded_step_rad=command_step,
        rise_time_s=rise_time_value,
        settling_time_s=settling_time,
        overshoot_percent=float(overshoot),
        undershoot_percent=float(undershoot),
        output_lag_s=output_lag,
        oscillation_frequency_hz=osc_freq,
        damping_estimate=damping,
        stable=stable,
        pass_test=not warnings and stable,
        warning="; ".join(warnings),
    )


def analyze_velocity_ramp(
    samples: Iterable[TelemetrySample],
    output_per_motor: float,
    output_offset_rad: float,
    *,
    max_allowed_tracking_error_rad_s: float = 0.5,
    max_allowed_deflection_rad: float = 0.08,
    default_velocity_limit_rad_s: float = 8.0,
    default_accel_limit_rad_s2: float = 60.0,
) -> VelocityRampResult:
    sample_list, time_s, _motor, _output, motor_vel, output_vel, _cmd_pos = _series_arrays(samples)
    if len(sample_list) < 10:
        raise ValueError("at least 10 samples are required for velocity ramp analysis")

    predicted_output_vel = motor_vel * output_per_motor
    tracking_error = output_vel - predicted_output_vel
    deflection = compute_deflection(sample_list, output_per_motor, output_offset_rad)
    accel = _gradient(motor_vel, time_s)
    max_abs_vel = float(np.max(np.abs(motor_vel)))
    rms_error = float(math.sqrt(np.mean(np.square(tracking_error))))
    peak_error = float(np.max(np.abs(tracking_error)))
    rms_deflection = float(math.sqrt(np.mean(np.square(deflection))))
    peak_deflection = float(np.max(np.abs(deflection)))
    resonance_freq = dominant_frequency(time_s, tracking_error)
    resonance_detected = bool(resonance_freq is not None and peak_deflection > max_allowed_deflection_rad * 0.75)

    recommended_velocity = default_velocity_limit_rad_s
    if peak_error > max_allowed_tracking_error_rad_s or peak_deflection > max_allowed_deflection_rad:
        high_error = np.abs(tracking_error) > max_allowed_tracking_error_rad_s
        high_deflection = np.abs(deflection) > max_allowed_deflection_rad
        bad_velocity = np.abs(motor_vel[high_error | high_deflection])
        if len(bad_velocity):
            recommended_velocity = max(0.1, float(np.min(bad_velocity) * 0.8))

    recommended_accel = min(default_accel_limit_rad_s2, max(0.1, float(np.percentile(np.abs(accel), 90))))

    warnings: list[str] = []
    if peak_error > max_allowed_tracking_error_rad_s:
        warnings.append(
            f"velocity tracking error {peak_error:.5f} rad/s exceeds {max_allowed_tracking_error_rad_s:.5f}"
        )
    if peak_deflection > max_allowed_deflection_rad:
        warnings.append(f"deflection {peak_deflection:.5f} rad exceeds {max_allowed_deflection_rad:.5f}")
    if resonance_detected:
        warnings.append("possible resonance region detected")

    return VelocityRampResult(
        sample_count=len(sample_list),
        max_abs_velocity_rad_s=max_abs_vel,
        rms_velocity_tracking_error_rad_s=rms_error,
        peak_velocity_tracking_error_rad_s=peak_error,
        rms_deflection_rad=rms_deflection,
        peak_deflection_rad=peak_deflection,
        resonance_frequency_hz=resonance_freq,
        resonance_detected=resonance_detected,
        recommended_velocity_limit_rad_s=recommended_velocity,
        recommended_accel_limit_rad_s2=recommended_accel,
        pass_test=not warnings,
        warning="; ".join(warnings),
    )


def analyze_compliance(
    samples: Iterable[TelemetrySample],
    output_per_motor: float,
    output_offset_rad: float,
    *,
    mass_kg: float | None = None,
    arm_length_m: float | None = None,
    known_torque_nm: float | None = None,
    max_creep_rad: float = 0.01,
) -> ComplianceResult:
    sample_list = _sample_list(samples)
    if len(sample_list) < 3:
        raise ValueError("at least three samples are required for compliance analysis")

    deflection = compute_deflection(sample_list, output_per_motor, output_offset_rad)
    mean_deflection = float(np.mean(deflection))
    relative_compliance = float(np.ptp(deflection))
    hysteresis = _directional_hysteresis(sample_list, deflection)
    creep = float(deflection[-1] - deflection[0])

    torque = known_torque_nm
    if torque is None and mass_kg is not None and arm_length_m is not None:
        torque = float(mass_kg) * 9.81 * float(arm_length_m)

    stiffness: float | None = None
    if torque is not None and abs(mean_deflection) > 1e-9:
        stiffness = abs(float(torque) / mean_deflection)

    warnings: list[str] = []
    if torque is None:
        warnings.append("known torque not provided; reporting relative compliance only")
    if abs(creep) > max_creep_rad:
        warnings.append(f"creep {creep:.5f} rad exceeds {max_creep_rad:.5f} rad")
    if stiffness is None and torque is not None:
        warnings.append("deflection too small to compute stiffness")

    return ComplianceResult(
        sample_count=len(sample_list),
        torque_nm=None if torque is None else float(torque),
        mean_deflection_rad=mean_deflection,
        stiffness_nm_per_rad=stiffness,
        relative_compliance_rad=relative_compliance,
        hysteresis_rad=hysteresis,
        creep_rad=creep,
        pass_test=not [warning for warning in warnings if not warning.startswith("known torque")],
        warning="; ".join(warnings),
    )


def analyze_resonance(
    samples: Iterable[TelemetrySample],
    output_per_motor: float,
    output_offset_rad: float,
    *,
    start_frequency_hz: float = 0.5,
    end_frequency_hz: float = 20.0,
    min_sample_count: int = 64,
    min_peak_deflection_rad: float = 0.001,
    min_prominence_db: float = 6.0,
    max_allowed_peak_deflection_rad: float = 0.12,
) -> ResonanceResult:
    sample_list, time_s, motor, _output, _motor_vel, _output_vel, _cmd_pos = _series_arrays(samples)
    if len(sample_list) < min_sample_count:
        raise ValueError(f"at least {min_sample_count} samples are required for resonance analysis")
    if float(time_s[-1] - time_s[0]) <= 0:
        raise ValueError("telemetry timestamps did not advance")

    dt = float(np.median(np.diff(time_s)))
    if dt <= 0:
        raise ValueError("telemetry sample interval must be positive")
    sample_rate = 1.0 / dt
    nyquist = sample_rate / 2.0
    if sample_rate < end_frequency_hz * 2.5:
        raise ValueError(
            f"sample rate {sample_rate:.2f} Hz is too low for {end_frequency_hz:.2f} Hz resonance analysis"
        )

    deflection = compute_deflection(sample_list, output_per_motor, output_offset_rad)
    detrended_deflection = signal.detrend(deflection)
    detrended_motor = signal.detrend(motor)
    rms_deflection = float(math.sqrt(np.mean(np.square(deflection))))
    peak_deflection = float(np.max(np.abs(deflection)))
    if peak_deflection < min_peak_deflection_rad:
        raise ValueError("resonance signal is too small to analyze")

    nperseg = min(len(detrended_deflection), 512)
    if nperseg < 32:
        nperseg = len(detrended_deflection)
    frequencies, deflection_power = signal.welch(
        detrended_deflection,
        fs=sample_rate,
        nperseg=nperseg,
        scaling="density",
    )
    motor_frequencies, motor_power = signal.welch(
        detrended_motor,
        fs=sample_rate,
        nperseg=nperseg,
        scaling="density",
    )
    if not np.allclose(frequencies, motor_frequencies):
        motor_power = np.interp(frequencies, motor_frequencies, motor_power)

    band_mask = (
        (frequencies >= max(0.0, float(start_frequency_hz)))
        & (frequencies <= min(float(end_frequency_hz), nyquist * 0.9))
    )
    if np.count_nonzero(band_mask) < 3:
        raise ValueError("resonance frequency band has too few spectral bins")

    band_freq = frequencies[band_mask]
    band_power = deflection_power[band_mask]
    if float(np.max(band_power)) <= 1e-18:
        raise ValueError("resonance spectrum is flat")

    peak_index = int(np.argmax(band_power))
    peak_frequency = float(band_freq[peak_index])
    peak_power = float(band_power[peak_index])
    floor_power = float(np.median(band_power))
    if floor_power <= 0.0:
        floor_power = float(np.mean(band_power))
    prominence_db = 10.0 * math.log10(peak_power / max(floor_power, 1e-18))
    if prominence_db < min_prominence_db:
        raise ValueError("resonance peak is not prominent enough")

    absolute_peak_index = int(np.where(band_mask)[0][peak_index])
    input_power = float(motor_power[absolute_peak_index])
    peak_gain = math.sqrt(peak_power / input_power) if input_power > 1e-18 else None

    half_power = peak_power * 0.5
    left = peak_index
    right = peak_index
    while left > 0 and band_power[left] >= half_power:
        left -= 1
    while right < len(band_power) - 1 and band_power[right] >= half_power:
        right += 1
    bandwidth = None
    q_factor = None
    if right > left:
        bandwidth = float(band_freq[right] - band_freq[left])
        if bandwidth > 0:
            q_factor = float(peak_frequency / bandwidth)

    warnings: list[str] = []
    if peak_deflection > max_allowed_peak_deflection_rad:
        warnings.append(
            f"peak deflection {peak_deflection:.5f} rad exceeds {max_allowed_peak_deflection_rad:.5f} rad"
        )
    if peak_frequency >= end_frequency_hz * 0.95:
        warnings.append("resonance peak is near the top of the sweep range")
    if peak_frequency <= start_frequency_hz * 1.05:
        warnings.append("resonance peak is near the bottom of the sweep range")

    return ResonanceResult(
        sample_count=len(sample_list),
        sample_rate_hz=float(sample_rate),
        start_frequency_hz=float(start_frequency_hz),
        end_frequency_hz=float(end_frequency_hz),
        peak_frequency_hz=peak_frequency,
        peak_gain=None if peak_gain is None else float(peak_gain),
        peak_prominence_db=float(prominence_db),
        bandwidth_hz=bandwidth,
        q_factor=q_factor,
        rms_deflection_rad=rms_deflection,
        peak_deflection_rad=peak_deflection,
        pass_test=not warnings,
        warning="; ".join(warnings),
    )


def estimate_encoder_signs(
    samples: Iterable[TelemetrySample],
    frozen_threshold_rad: float = 1e-4,
    discontinuity_threshold_rad: float = 1.0,
) -> EncoderSanityResult:
    sample_list = list(samples)
    if len(sample_list) < 5:
        raise ValueError("at least five samples are required for encoder sanity analysis")

    motor = np.array([sample.motor_rad for sample in sample_list], dtype=float)
    output = np.array([sample.output_rad for sample in sample_list], dtype=float)
    motor_delta = float(motor[-1] - motor[0])
    output_delta = float(output[-1] - output[0])
    motor_step = np.diff(motor)
    output_step = np.diff(output)

    motor_noise = float(np.std(motor_step)) if len(motor_step) else 0.0
    output_noise = float(np.std(output_step)) if len(output_step) else 0.0
    motor_discontinuity = bool(np.max(np.abs(motor_step)) > discontinuity_threshold_rad)
    output_discontinuity = bool(np.max(np.abs(output_step)) > discontinuity_threshold_rad)
    frozen_motor = abs(motor_delta) < frozen_threshold_rad
    frozen_output = abs(output_delta) < frozen_threshold_rad

    motor_sign = 1 if motor_delta >= 0 else -1
    output_sign = 1 if output_delta >= 0 else -1
    pass_test = not (motor_discontinuity or output_discontinuity or frozen_motor or frozen_output)

    warnings: list[str] = []
    if frozen_motor:
        warnings.append("motor encoder appears frozen")
    if frozen_output:
        warnings.append("output encoder appears frozen")
    if motor_discontinuity:
        warnings.append("motor encoder has a large unexplained jump")
    if output_discontinuity:
        warnings.append("output encoder has a large unexplained jump")

    return EncoderSanityResult(
        motor_encoder_sign=motor_sign,
        output_encoder_sign=output_sign,
        motor_delta_rad=motor_delta,
        output_delta_rad=output_delta,
        motor_noise_rad=motor_noise,
        output_noise_rad=output_noise,
        motor_discontinuity=motor_discontinuity,
        output_discontinuity=output_discontinuity,
        frozen_motor=frozen_motor,
        frozen_output=frozen_output,
        sample_count=len(sample_list),
        pass_test=pass_test,
        warning="; ".join(warnings),
    )


def dominant_frequency(time_s: Sequence[float] | np.ndarray, values: Sequence[float] | np.ndarray) -> float | None:
    time = np.asarray(time_s, dtype=float)
    signal = np.asarray(values, dtype=float)
    if len(time) < 4 or len(signal) != len(time):
        return None
    duration = float(time[-1] - time[0])
    if duration <= 0:
        return None
    dt = float(np.median(np.diff(time)))
    if dt <= 0:
        return None
    centered = signal - float(np.mean(signal))
    if float(np.max(np.abs(centered))) < 1e-12:
        return None
    spectrum = np.fft.rfft(centered)
    frequencies = np.fft.rfftfreq(len(centered), d=dt)
    if len(frequencies) <= 1:
        return None
    magnitudes = np.abs(spectrum)
    magnitudes[0] = 0.0
    idx = int(np.argmax(magnitudes))
    if magnitudes[idx] <= 1e-12:
        return None
    return float(frequencies[idx])


def estimate_damping_ratio(
    time_s: Sequence[float] | np.ndarray,
    residual: Sequence[float] | np.ndarray,
) -> float | None:
    _time = np.asarray(time_s, dtype=float)
    values = np.abs(np.asarray(residual, dtype=float))
    if len(values) < 5:
        return None
    peak_indices = [
        idx
        for idx in range(1, len(values) - 1)
        if values[idx] > values[idx - 1] and values[idx] >= values[idx + 1] and values[idx] > 1e-9
    ]
    if len(peak_indices) < 2:
        return None
    first = float(values[peak_indices[0]])
    second = float(values[peak_indices[1]])
    if first <= 0 or second <= 0 or first <= second:
        return None
    log_dec = math.log(first / second)
    return float(log_dec / math.sqrt((2.0 * math.pi) ** 2 + log_dec**2))


def _gradient(values: np.ndarray, time_s: np.ndarray) -> np.ndarray:
    if len(values) < 2 or len(time_s) != len(values):
        return np.zeros_like(values)
    if float(time_s[-1] - time_s[0]) <= 0:
        return np.zeros_like(values)
    return np.gradient(values, time_s)


def _safe_corrcoef(a: np.ndarray, b: np.ndarray) -> float | None:
    if len(a) < 2 or len(b) != len(a):
        return None
    if float(np.std(a)) <= 1e-12 or float(np.std(b)) <= 1e-12:
        return None
    return float(np.corrcoef(a, b)[0, 1])


def _last_nonzero(values: np.ndarray) -> float:
    nonzero = values[np.nonzero(values)]
    if len(nonzero) == 0:
        return 0.0
    return float(nonzero[-1])


def _threshold_crossing_time(time_s: np.ndarray, normalized: np.ndarray, threshold: float) -> float:
    hits = np.where(normalized >= threshold)[0]
    if len(hits) == 0:
        return math.nan
    idx = int(hits[0])
    if idx == 0:
        return float(time_s[0])
    prev_idx = idx - 1
    y0 = normalized[prev_idx]
    y1 = normalized[idx]
    if abs(y1 - y0) < 1e-12:
        return float(time_s[idx])
    ratio = (threshold - y0) / (y1 - y0)
    return float(time_s[prev_idx] + ratio * (time_s[idx] - time_s[prev_idx]))


def _settling_time(time_s: np.ndarray, values: np.ndarray, final_value: float, band: float) -> float | None:
    error = np.abs(values - final_value)
    for idx in range(len(values)):
        if np.all(error[idx:] <= band):
            return float(time_s[idx])
    return None


def _output_lag(time_s: np.ndarray, command: np.ndarray, output: np.ndarray) -> float | None:
    if len(time_s) < 4:
        return None
    command_step = command[-1] - command[0]
    output_step = output[-1] - output[0]
    if abs(command_step) < 1e-9 or abs(output_step) < 1e-9:
        return None
    command_norm = np.sign(command_step) * (command - command[0]) / abs(command_step)
    output_norm = np.sign(output_step) * (output - output[0]) / abs(output_step)
    command_time = _threshold_crossing_time(time_s, command_norm, 0.5)
    output_time = _threshold_crossing_time(time_s, output_norm, 0.5)
    if math.isnan(command_time) or math.isnan(output_time):
        return None
    return float(output_time - command_time)


def _directional_hysteresis(samples: Sequence[TelemetrySample], deflection: np.ndarray) -> float:
    motor_vel = np.array([sample.motor_vel_rad_s for sample in samples], dtype=float)
    positive = deflection[motor_vel > 1e-4]
    negative = deflection[motor_vel < -1e-4]
    if len(positive) == 0 or len(negative) == 0:
        return 0.0
    return float(abs(np.mean(positive) - np.mean(negative)))
