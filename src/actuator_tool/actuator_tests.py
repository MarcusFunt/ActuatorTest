"""Automated test and calibration routines.

These routines are deliberately UI-free. They accept an actuator client, a telemetry store, and
configuration objects, then return structured results that any frontend can display or save.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import time
from typing import Callable

from .actuator_analysis import (
    BacklashResult,
    ComplianceResult,
    EncoderSanityResult,
    RatioFitResult,
    ResonanceResult,
    StepResponseResult,
    VelocityRampResult,
    analyze_backlash,
    analyze_compliance,
    analyze_resonance,
    analyze_step_response,
    analyze_velocity_ramp,
    estimate_encoder_signs,
    fit_ratio,
)
from .actuator_data import ActuatorInfo, TelemetrySample, TelemetryStore
from .actuator_protocol import ActuatorMode, FaultFlags
from .actuator_serial import ActuatorClient, ActuatorError
from .config_schema import CalibrationConfig, SafetyLimits


ProgressCallback = Callable[[str], None]


@dataclass
class DetectionResult:
    passed: bool
    info: ActuatorInfo | None
    message: str

    def as_dict(self):
        return asdict(self)


@dataclass
class RatioCalibrationResult:
    passed: bool
    fit: RatioFitResult
    calibration: CalibrationConfig
    sample_count: int
    message: str

    def as_dict(self):
        return asdict(self)


@dataclass
class ResonanceTestResult:
    passed: bool
    analysis: ResonanceResult
    calibration: CalibrationConfig
    sample_count: int
    message: str

    def as_dict(self):
        return asdict(self)


def _progress(callback: ProgressCallback | None, message: str) -> None:
    if callback is not None:
        callback(message)


def _abort_on_faults(client: ActuatorClient) -> None:
    faults = client.faults(timeout=0.5)
    if faults:
        raise ActuatorError(f"actuator fault active: {FaultFlags(faults)}")


def _wait_for_samples(
    store: TelemetryStore,
    start_index: int,
    min_samples: int,
    timeout_s: float,
) -> list[TelemetrySample]:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        samples = store.samples_since(start_index)
        if len(samples) >= min_samples:
            return samples
        time.sleep(0.02)
    return store.samples_since(start_index)


def _wait_after_move(
    client: ActuatorClient,
    store: TelemetryStore,
    start_index: int,
    duration_s: float,
) -> list[TelemetrySample]:
    deadline = time.monotonic() + duration_s
    last_fault_check = 0.0
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now - last_fault_check > 0.25:
            _abort_on_faults(client)
            last_fault_check = now
        time.sleep(0.02)
    return store.samples_since(start_index)


def run_detection(client: ActuatorClient, progress: ProgressCallback | None = None) -> DetectionResult:
    try:
        _progress(progress, "Sending PING")
        if not client.ping(timeout=1.0):
            return DetectionResult(False, None, "PING reply was not PONG")
        _progress(progress, "Reading INFO")
        info = client.info(timeout=1.0)
        if info.telemetry_schema_version != 1:
            return DetectionResult(False, info, "unsupported telemetry schema version")
        return DetectionResult(True, info, "actuator detected")
    except Exception as exc:
        return DetectionResult(False, None, str(exc))


def run_encoder_sanity_test(
    client: ActuatorClient,
    store: TelemetryStore,
    safety: SafetyLimits | None = None,
    progress: ProgressCallback | None = None,
) -> EncoderSanityResult:
    limits = safety or SafetyLimits()
    limits.validate()
    _progress(progress, "Entering calibration mode")
    client.set_mode(ActuatorMode.CALIBRATION)
    client.start_stream()
    _abort_on_faults(client)

    positive_start = store.total_samples
    velocity = limits.clamp_velocity(limits.calibration_velocity_rad_s)
    accel = limits.clamp_accel(limits.calibration_accel_rad_s2)
    delta = limits.clamp_delta(0.75)

    _progress(progress, "Moving positive for encoder sanity")
    client.move_rel(delta, velocity, accel)
    positive_samples = _wait_after_move(client, store, positive_start, abs(delta) / velocity + 0.4)
    if len(positive_samples) < 5:
        raise ActuatorError("not enough telemetry during positive encoder sanity move")

    _progress(progress, "Moving negative for encoder sanity")
    negative_start = store.total_samples
    client.move_rel(-delta, velocity, accel)
    _wait_after_move(client, store, negative_start, abs(delta) / velocity + 0.4)
    client.stop()

    result = estimate_encoder_signs(positive_samples)
    if result.pass_test:
        _progress(progress, "Encoder sanity test passed")
    else:
        _progress(progress, f"Encoder sanity warning: {result.warning}")
    return result


def run_ratio_calibration(
    client: ActuatorClient,
    store: TelemetryStore,
    actuator_info: ActuatorInfo | None = None,
    safety: SafetyLimits | None = None,
    progress: ProgressCallback | None = None,
    motor_sweep_rad: float = 8.0,
    residual_threshold_rad: float = 0.03,
) -> RatioCalibrationResult:
    limits = safety or SafetyLimits()
    limits.validate()
    velocity = limits.clamp_velocity(max(limits.calibration_velocity_rad_s, 2.0))
    accel = limits.clamp_accel(limits.calibration_accel_rad_s2)
    sweep = abs(limits.clamp_delta(motor_sweep_rad))
    if sweep < 0.5:
        raise ValueError("motor_sweep_rad is too small for ratio calibration")

    _progress(progress, "Entering calibration mode")
    client.set_mode(ActuatorMode.CALIBRATION)
    client.start_stream()
    _abort_on_faults(client)
    start_index = store.total_samples

    _progress(progress, "Sweeping forward for ratio fit")
    client.move_rel(sweep, velocity, accel)
    _wait_after_move(client, store, start_index, sweep / velocity + 0.5)

    _progress(progress, "Sweeping reverse for ratio fit")
    client.move_rel(-sweep, velocity, accel)
    samples = _wait_after_move(client, store, start_index, sweep / velocity + 0.7)
    client.stop()

    if len(samples) < 20:
        raise ActuatorError("not enough telemetry for ratio calibration")

    _progress(progress, "Fitting ratio")
    fit = fit_ratio(samples, residual_threshold_rad=residual_threshold_rad)
    signs = estimate_encoder_signs(samples[: max(5, len(samples) // 3)])
    info = actuator_info or ActuatorInfo()
    calibration = CalibrationConfig.from_fit(
        actuator_id=info.actuator_id,
        hardware_revision=info.hardware_revision,
        firmware_version=info.firmware_version,
        motor_encoder_sign=signs.motor_encoder_sign,
        output_encoder_sign=signs.output_encoder_sign,
        output_per_motor=fit.output_per_motor,
        output_offset_rad=fit.output_offset_rad,
        hysteresis_rad=fit.hysteresis_rad,
        max_safe_velocity_rad_s=limits.max_velocity_rad_s,
        max_safe_accel_rad_s2=limits.max_accel_rad_s2,
    )
    passed = fit.pass_fit and signs.pass_test
    message = "ratio calibration passed" if passed else "; ".join([fit.warning, signs.warning]).strip("; ")
    _progress(progress, message)
    return RatioCalibrationResult(
        passed=passed,
        fit=fit,
        calibration=calibration,
        sample_count=len(samples),
        message=message,
    )


def run_resonance_test(
    client: ActuatorClient,
    store: TelemetryStore,
    calibration: CalibrationConfig,
    safety: SafetyLimits | None = None,
    progress: ProgressCallback | None = None,
    *,
    amplitude_rad: float = 0.08,
    start_frequency_hz: float = 0.5,
    end_frequency_hz: float = 20.0,
    duration_s: float = 20.0,
    max_deflection_rad: float = 0.12,
) -> ResonanceTestResult:
    limits = safety or SafetyLimits()
    limits.validate()
    if abs(calibration.output_per_motor) < 1e-9:
        raise ValueError("ratio calibration is required before resonance testing")

    _progress(progress, "Entering calibration mode")
    client.set_mode(ActuatorMode.CALIBRATION)
    client.start_stream()
    _abort_on_faults(client)
    start_index = store.total_samples

    _progress(progress, "Starting low-amplitude chirp")
    client.start_chirp(
        amplitude_rad=abs(limits.clamp_delta(amplitude_rad)),
        start_frequency_hz=start_frequency_hz,
        end_frequency_hz=end_frequency_hz,
        duration_s=duration_s,
        max_deflection_rad=max_deflection_rad,
    )
    samples = _wait_after_move(client, store, start_index, duration_s + 0.75)
    client.stop()

    if len(samples) < 64:
        raise ActuatorError("not enough telemetry for resonance analysis")

    _progress(progress, "Analyzing resonance spectrum")
    analysis = analyze_resonance(
        samples,
        calibration.output_per_motor,
        calibration.output_offset_rad,
        start_frequency_hz=start_frequency_hz,
        end_frequency_hz=end_frequency_hz,
        max_allowed_peak_deflection_rad=max_deflection_rad,
    )
    updated = CalibrationConfig.from_dict(calibration.as_dict())
    updated.resonance_frequency_hz = analysis.peak_frequency_hz
    updated.resonance_derating_enabled = analysis.peak_frequency_hz is not None
    passed = analysis.pass_test and analysis.peak_frequency_hz is not None
    message = (
        f"resonance peak {analysis.peak_frequency_hz:.2f} Hz"
        if passed and analysis.peak_frequency_hz is not None
        else analysis.warning or "resonance test failed"
    )
    _progress(progress, message)
    return ResonanceTestResult(
        passed=passed,
        analysis=analysis,
        calibration=updated,
        sample_count=len(samples),
        message=message,
    )


def run_step_response_test(
    client: ActuatorClient,
    store: TelemetryStore,
    safety: SafetyLimits | None = None,
    progress: ProgressCallback | None = None,
    *,
    calibration: CalibrationConfig | None = None,
    step_rad: float = 0.5,
) -> StepResponseResult:
    limits = safety or SafetyLimits()
    limits.validate()
    velocity = limits.clamp_velocity(max(limits.calibration_velocity_rad_s, 2.0))
    accel = limits.clamp_accel(max(limits.calibration_accel_rad_s2, 20.0))
    delta = limits.clamp_delta(step_rad)

    _progress(progress, "Entering calibration mode")
    client.set_mode(ActuatorMode.CALIBRATION)
    client.start_stream()
    _abort_on_faults(client)
    start_index = store.total_samples

    _progress(progress, "Running step response move")
    client.move_rel(delta, velocity, accel)
    samples = _wait_after_move(client, store, start_index, abs(delta) / velocity + 1.5)
    client.stop()
    if len(samples) < 20:
        raise ActuatorError("not enough telemetry for step response analysis")
    result = analyze_step_response(
        samples,
        output_per_motor=None if calibration is None else calibration.output_per_motor,
        output_offset_rad=0.0 if calibration is None else calibration.output_offset_rad,
        commanded_step_rad=None if calibration is None else delta * calibration.output_per_motor,
    )
    _progress(progress, "step response passed" if result.pass_test else result.warning)
    return result


def run_velocity_ramp_test(
    client: ActuatorClient,
    store: TelemetryStore,
    calibration: CalibrationConfig,
    safety: SafetyLimits | None = None,
    progress: ProgressCallback | None = None,
    *,
    sweep_rad: float = 6.0,
) -> VelocityRampResult:
    limits = safety or SafetyLimits()
    limits.validate()
    velocity = limits.clamp_velocity(limits.max_velocity_rad_s)
    accel = limits.clamp_accel(limits.max_accel_rad_s2)
    sweep = abs(limits.clamp_delta(sweep_rad))

    _progress(progress, "Entering calibration mode")
    client.set_mode(ActuatorMode.CALIBRATION)
    client.start_stream()
    _abort_on_faults(client)
    start_index = store.total_samples

    _progress(progress, "Running velocity ramp sweep")
    client.move_rel(sweep, velocity, accel)
    _wait_after_move(client, store, start_index, sweep / max(velocity, 0.01) + 0.8)
    client.move_rel(-sweep, velocity, accel)
    samples = _wait_after_move(client, store, start_index, sweep / max(velocity, 0.01) + 0.8)
    client.stop()
    if len(samples) < 20:
        raise ActuatorError("not enough telemetry for velocity ramp analysis")
    result = analyze_velocity_ramp(
        samples,
        calibration.output_per_motor,
        calibration.output_offset_rad,
        default_velocity_limit_rad_s=limits.max_velocity_rad_s,
        default_accel_limit_rad_s2=limits.max_accel_rad_s2,
    )
    _progress(progress, "velocity ramp passed" if result.pass_test else result.warning)
    return result


def run_backlash_test(
    client: ActuatorClient,
    store: TelemetryStore,
    calibration: CalibrationConfig,
    safety: SafetyLimits | None = None,
    progress: ProgressCallback | None = None,
    *,
    reversal_rad: float = 0.8,
    cycles: int = 3,
) -> BacklashResult:
    limits = safety or SafetyLimits()
    limits.validate()
    velocity = limits.clamp_velocity(limits.calibration_velocity_rad_s)
    accel = limits.clamp_accel(limits.calibration_accel_rad_s2)
    delta = abs(limits.clamp_delta(reversal_rad))

    _progress(progress, "Entering calibration mode")
    client.set_mode(ActuatorMode.CALIBRATION)
    client.start_stream()
    _abort_on_faults(client)
    start_index = store.total_samples

    for cycle in range(max(1, int(cycles))):
        _progress(progress, f"Backlash reversal cycle {cycle + 1}")
        client.move_rel(delta, velocity, accel)
        _wait_after_move(client, store, start_index, delta / velocity + 0.25)
        client.move_rel(-delta, velocity, accel)
        _wait_after_move(client, store, start_index, delta / velocity + 0.25)
    client.stop()
    samples = store.samples_since(start_index)
    if len(samples) < 20:
        raise ActuatorError("not enough telemetry for backlash analysis")
    result = analyze_backlash(samples, calibration.output_per_motor)
    _progress(progress, "backlash test passed" if result.pass_test else result.warning)
    return result


def run_compliance_test(
    client: ActuatorClient,
    store: TelemetryStore,
    calibration: CalibrationConfig,
    progress: ProgressCallback | None = None,
    *,
    duration_s: float = 4.0,
    known_torque_nm: float | None = None,
) -> ComplianceResult:
    _progress(progress, "Collecting held-position compliance samples")
    client.start_stream()
    _abort_on_faults(client)
    start_index = store.total_samples
    samples = _wait_after_move(client, store, start_index, duration_s)
    if len(samples) < 3:
        raise ActuatorError("not enough telemetry for compliance analysis")
    result = analyze_compliance(
        samples,
        calibration.output_per_motor,
        calibration.output_offset_rad,
        known_torque_nm=known_torque_nm,
    )
    _progress(progress, "compliance test passed" if result.pass_test else result.warning)
    return result
