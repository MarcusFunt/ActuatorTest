"""Reflex web frontend for the actuator bench tool."""

from __future__ import annotations

import asyncio
import math
import os
import subprocess
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any

import reflex as rx

from actuator_tool.actuator_analysis import RatioFitResult, ResonanceResult
from actuator_tool.actuator_data import ActuatorInfo, TelemetrySample, TelemetryStore
from actuator_tool.actuator_protocol import ActuatorMode, FaultFlags
from actuator_tool.actuator_report import (
    SessionReport,
    create_session_folder,
    plot_deflection,
    plot_ratio,
    plot_resonance,
    write_summary,
)
from actuator_tool.actuator_serial import (
    ActuatorClient,
    ActuatorError,
    PySerialTransport,
    SimulatedTransport,
    scan_serial_ports,
)
from actuator_tool.actuator_tests import (
    run_backlash_test,
    run_compliance_test,
    run_detection,
    run_encoder_sanity_test,
    run_ratio_calibration,
    run_resonance_test,
    run_step_response_test,
    run_velocity_ramp_test,
)
from actuator_tool.config_schema import CalibrationConfig, SafetyLimits

from .bokeh_charts import BOKEH_PORT, launch_bokeh_thread


# Backend context (non-serialisable; lives outside Reflex state)

@dataclass
class BackendCtx:
    store: TelemetryStore = field(default_factory=TelemetryStore)
    client: ActuatorClient | None = None
    info: ActuatorInfo = field(default_factory=ActuatorInfo)
    safety: SafetyLimits = field(default_factory=SafetyLimits)
    calibration: CalibrationConfig = field(default_factory=CalibrationConfig)
    ratio_fit: RatioFitResult | None = None
    resonance_result: ResonanceResult | None = None
    report: SessionReport | None = None
    connected: bool = False
    warnings: list[str] = field(default_factory=list)
    logs: list[dict[str, str]] = field(default_factory=list)
    _status: str = "Disconnected"
    _log_seq: int = 0
    _lock: threading.Lock = field(default_factory=threading.Lock)

    def set_status(self, msg: str) -> None:
        with self._lock:
            self._status = msg

    def get_status(self) -> str:
        with self._lock:
            return self._status

    def require_client(self) -> ActuatorClient:
        if self.client is None or not self.connected:
            raise ActuatorError("not connected")
        return self.client

    def add_log(self, kind: str, tag: str, message: str) -> None:
        kind = kind.lower()
        with self._lock:
            self._log_seq += 1
            self.logs.append(
                {
                    "id": str(self._log_seq),
                    "time": datetime.now().strftime("%H:%M:%S"),
                    "kind": kind,
                    "class_name": f"log-tag {kind}",
                    "tag": tag.upper(),
                    "message": message,
                }
            )
            del self.logs[:-300]

    def clear_logs(self) -> None:
        with self._lock:
            self.logs.clear()

    def get_logs(self) -> list[dict[str, str]]:
        with self._lock:
            return [dict(entry) for entry in self.logs]


_ctx = BackendCtx()
_executor = ThreadPoolExecutor(max_workers=2, thread_name_prefix="actuator")

_bokeh_started = False


def _log(kind: str, tag: str, message: str) -> None:
    _ctx.add_log(kind, tag, message)


def _env_bool(name: str, default: bool) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


_DEFAULT_USE_SIM = _env_bool("ACTUATOR_GUI_USE_SIM", True)
_DEFAULT_PORT = os.environ.get("ACTUATOR_GUI_PORT", "")
_REPORT_ROOT = Path(
    os.environ.get(
        "ACTUATOR_GUI_REPORT_DIR",
        str(Path.home() / "Documents" / "ActuatorBenchReports"),
    )
)


def _new_report() -> SessionReport:
    return SessionReport(create_session_folder(_REPORT_ROOT))


def _float_text(value: float | int | None, digits: int = 4, suffix: str = "") -> str:
    if value is None:
        return "-"
    try:
        number = float(value)
    except (TypeError, ValueError):
        return "-"
    if not math.isfinite(number):
        return "-"
    return f"{number:.{digits}f}{suffix}"


def _parse_float(value: str, label: str) -> float:
    try:
        number = float(str(value).strip())
    except ValueError as exc:
        raise ValueError(f"{label} must be numeric") from exc
    if not math.isfinite(number):
        raise ValueError(f"{label} must be finite")
    return number


def _format_config_number(value: float | int | None, digits: int = 6) -> str:
    if value is None:
        return ""
    try:
        number = float(value)
    except (TypeError, ValueError):
        return ""
    if not math.isfinite(number):
        return ""
    return f"{number:.{digits}f}".rstrip("0").rstrip(".")


def _sync_calibration_from_config(ctx: BackendCtx, cfg: dict[str, Any]) -> None:
    cal = ctx.calibration
    if "output_per_motor" in cfg:
        cal.output_per_motor = float(cfg.get("output_per_motor") or 1.0)
        cal.motor_per_output = 1.0 / cal.output_per_motor
    if "output_offset_rad" in cfg:
        cal.output_offset_rad = float(cfg.get("output_offset_rad") or 0.0)
    if "pid_enabled" in cfg:
        cal.pid_enabled = bool(cfg.get("pid_enabled"))
    if "pid_kp" in cfg:
        cal.pid_kp = float(cfg.get("pid_kp") or 0.0)
    if "pid_ki" in cfg:
        cal.pid_ki = float(cfg.get("pid_ki") or 0.0)
    if "pid_kd" in cfg:
        cal.pid_kd = float(cfg.get("pid_kd") or 0.0)
    if "pid_i_limit_motor_rad" in cfg:
        cal.pid_i_limit_motor_rad = float(cfg.get("pid_i_limit_motor_rad") or 0.05)
    if "pid_output_limit_motor_rad" in cfg:
        cal.pid_output_limit_motor_rad = float(cfg.get("pid_output_limit_motor_rad") or 0.25)
    if "velocity_pid_kp" in cfg:
        cal.velocity_pid_kp = float(cfg.get("velocity_pid_kp") or 0.2)
    if "velocity_pid_ki" in cfg:
        cal.velocity_pid_ki = float(cfg.get("velocity_pid_ki") or 2.0)
    if "velocity_pid_i_limit_motor_rad" in cfg:
        cal.velocity_pid_i_limit_motor_rad = float(cfg.get("velocity_pid_i_limit_motor_rad") or 0.2)
    if "torque_proxy_kp" in cfg:
        cal.torque_proxy_kp = float(cfg.get("torque_proxy_kp") or 3.0)
    if "torque_proxy_limit_rad" in cfg:
        cal.torque_proxy_limit_rad = float(cfg.get("torque_proxy_limit_rad") or 0.12)
    if "torque_proxy_max_motor_velocity_rad_s" in cfg:
        cal.torque_proxy_max_motor_velocity_rad_s = float(cfg.get("torque_proxy_max_motor_velocity_rad_s") or 4.0)
    if "torque_proxy_timeout_s" in cfg:
        cal.torque_proxy_timeout_s = float(cfg.get("torque_proxy_timeout_s") or 3.0)
    if "missed_step_correction_enabled" in cfg:
        cal.missed_step_correction_enabled = bool(cfg.get("missed_step_correction_enabled"))
    if "missed_step_warn_motor_rad" in cfg:
        cal.missed_step_warn_motor_rad = float(cfg.get("missed_step_warn_motor_rad") or 0.05)
    if "missed_step_fault_motor_rad" in cfg:
        cal.missed_step_fault_motor_rad = float(cfg.get("missed_step_fault_motor_rad") or 0.25)
    if "missed_step_correction_rate" in cfg:
        cal.missed_step_correction_rate = float(cfg.get("missed_step_correction_rate") or 0.25)
    if "current_control_enabled" in cfg:
        cal.current_control_enabled = bool(cfg.get("current_control_enabled"))
    if "idle_current_ma" in cfg:
        cal.idle_current_ma = int(cfg.get("idle_current_ma") or 0)
    if "hold_current_ma" in cfg:
        cal.hold_current_ma = int(cfg.get("hold_current_ma") or 350)
    if "run_current_ma" in cfg:
        cal.run_current_ma = int(cfg.get("run_current_ma") or 1000)
    if "current_downshift_delay_s" in cfg:
        cal.current_downshift_delay_s = float(cfg.get("current_downshift_delay_s") or 0.5)
    if "autotune_max_amplitude_rad" in cfg:
        cal.autotune_max_amplitude_rad = float(cfg.get("autotune_max_amplitude_rad") or 0.4)
    if "autotune_max_duration_s" in cfg:
        cal.autotune_max_duration_s = float(cfg.get("autotune_max_duration_s") or 15.0)
    if "autotune_max_deflection_rad" in cfg:
        cal.autotune_max_deflection_rad = float(cfg.get("autotune_max_deflection_rad") or 0.25)
    if "backlash_motor_rad" in cfg:
        cal.backlash_motor_rad = float(cfg.get("backlash_motor_rad") or 0.0)
    if "backlash_comp_enabled" in cfg:
        cal.backlash_comp_enabled = bool(cfg.get("backlash_comp_enabled"))
    if "resonance_frequency_hz" in cfg:
        value = cfg.get("resonance_frequency_hz")
        cal.resonance_frequency_hz = None if value in (None, "", 0, 0.0) else float(value)
    if "resonance_derating_enabled" in cfg:
        cal.resonance_derating_enabled = bool(cfg.get("resonance_derating_enabled"))


def _read_device_config() -> dict[str, Any]:
    ctx = _ctx
    cfg = ctx.require_client().get_config(timeout=1.0)
    _sync_calibration_from_config(ctx, cfg)
    _log("rx", "RX", "GET_CONFIG read from actuator")
    ctx.set_status("Config read from actuator")
    return cfg


def _apply_ui_config(
    safety_values: dict[str, float],
    cal_values: dict[str, Any],
) -> None:
    ctx = _ctx
    safety = SafetyLimits(
        max_velocity_rad_s=safety_values["max_velocity_rad_s"],
        max_accel_rad_s2=safety_values["max_accel_rad_s2"],
        jog_step_rad=safety_values["jog_step_rad"],
        calibration_velocity_rad_s=safety_values["calibration_velocity_rad_s"],
        calibration_accel_rad_s2=safety_values["calibration_accel_rad_s2"],
        max_move_rad=safety_values["max_move_rad"],
    )
    safety.validate()
    ctx.safety = safety

    cal = ctx.calibration
    cal.max_safe_velocity_rad_s = safety.max_velocity_rad_s
    cal.max_safe_accel_rad_s2 = safety.max_accel_rad_s2
    cal.output_per_motor = cal_values["output_per_motor"]
    cal.motor_per_output = 1.0 / cal.output_per_motor
    cal.output_offset_rad = cal_values["output_offset_rad"]
    cal.pid_enabled = cal_values["pid_enabled"]
    cal.pid_kp = cal_values["pid_kp"]
    cal.pid_ki = cal_values["pid_ki"]
    cal.pid_kd = cal_values["pid_kd"]
    cal.pid_i_limit_motor_rad = cal_values["pid_i_limit_motor_rad"]
    cal.pid_output_limit_motor_rad = cal_values["pid_output_limit_motor_rad"]
    cal.backlash_motor_rad = cal_values["backlash_motor_rad"]
    cal.backlash_comp_enabled = cal_values["backlash_comp_enabled"]
    cal.resonance_frequency_hz = cal_values["resonance_frequency_hz"]
    cal.resonance_derating_enabled = cal_values["resonance_derating_enabled"]
    cal.validate()

    if ctx.client is not None:
        ctx.client.max_velocity_rad_s = safety.max_velocity_rad_s
        ctx.client.max_accel_rad_s2 = safety.max_accel_rad_s2

    if ctx.connected:
        client = ctx.require_client()
        for key, value in [
            ("output_per_motor", cal.output_per_motor),
            ("output_offset_rad", cal.output_offset_rad),
            ("pid_enabled", cal.pid_enabled),
            ("pid_kp", cal.pid_kp),
            ("pid_ki", cal.pid_ki),
            ("pid_kd", cal.pid_kd),
            ("pid_i_limit_motor_rad", cal.pid_i_limit_motor_rad),
            ("pid_output_limit_motor_rad", cal.pid_output_limit_motor_rad),
            ("velocity_pid_kp", cal.velocity_pid_kp),
            ("velocity_pid_ki", cal.velocity_pid_ki),
            ("velocity_pid_i_limit_motor_rad", cal.velocity_pid_i_limit_motor_rad),
            ("torque_proxy_kp", cal.torque_proxy_kp),
            ("torque_proxy_limit_rad", cal.torque_proxy_limit_rad),
            ("torque_proxy_max_motor_velocity_rad_s", cal.torque_proxy_max_motor_velocity_rad_s),
            ("torque_proxy_timeout_s", cal.torque_proxy_timeout_s),
            ("missed_step_correction_enabled", cal.missed_step_correction_enabled),
            ("missed_step_warn_motor_rad", cal.missed_step_warn_motor_rad),
            ("missed_step_fault_motor_rad", cal.missed_step_fault_motor_rad),
            ("missed_step_correction_rate", cal.missed_step_correction_rate),
            ("current_control_enabled", cal.current_control_enabled),
            ("idle_current_ma", cal.idle_current_ma),
            ("hold_current_ma", cal.hold_current_ma),
            ("run_current_ma", cal.run_current_ma),
            ("current_downshift_delay_s", cal.current_downshift_delay_s),
            ("autotune_max_amplitude_rad", cal.autotune_max_amplitude_rad),
            ("autotune_max_duration_s", cal.autotune_max_duration_s),
            ("autotune_max_deflection_rad", cal.autotune_max_deflection_rad),
            ("backlash_motor_rad", cal.backlash_motor_rad),
            ("backlash_comp_enabled", cal.backlash_comp_enabled),
            ("resonance_frequency_hz", cal.resonance_frequency_hz),
            ("resonance_derating_enabled", cal.resonance_derating_enabled),
        ]:
            client.set_config(key, value)
            _log("tx", "TX", f"SET_CONFIG {key}={value}")
        client.save_config()
        _log("tx", "TX", "SAVE_CONFIG")

    if ctx.report:
        ctx.report.save_calibration(cal)
        ctx.report.events.write("ui_config_save", cal.as_dict())
    ctx.set_status("Config saved")
    _log("event", "EVENT", "Config saved")


# Thread-pool work functions

def _connect(use_sim: bool, port: str) -> None:
    c = _ctx
    transport = SimulatedTransport() if use_sim else PySerialTransport(port)
    c.store.clear()
    client = ActuatorClient(transport, c.store)
    client.max_velocity_rad_s = c.safety.max_velocity_rad_s
    client.max_accel_rad_s2 = c.safety.max_accel_rad_s2
    client.connect()
    client.set_mode(ActuatorMode.DISABLED, timeout=0.5)
    info = client.info(timeout=1.0)
    try:
        cfg = client.get_config(timeout=0.5)
        _sync_calibration_from_config(c, cfg)
    except Exception as exc:
        _log("event", "EVENT", f"GET_CONFIG skipped: {exc}")
    client.start_stream(timeout=0.5)
    report = _new_report()
    report.save_actuator_info(info)
    report.events.write("connect", {"simulator": use_sim, "port": port})
    client.add_telemetry_callback(report.recorder.record_sample)
    c.client = client
    c.info = info
    c.report = report
    c.connected = True
    c.set_status(f"Connected to {info.actuator_id}")
    _log("event", "EVENT", f"connected via {'simulator' if use_sim else port}")
    _log("rx", "RX", f"INFO {info.actuator_id} {info.firmware_version} {info.hardware_revision}")


def _disconnect() -> None:
    c = _ctx
    if c.client is not None:
        c.client.disconnect()
    if c.report is not None:
        c.report.events.write("disconnect")
        c.report.close()
    c.connected = False
    c.client = None
    c.report = None
    c.set_status("Disconnected")
    _log("event", "EVENT", "disconnected")


def _run_detection() -> None:
    c = _ctx
    result = run_detection(c.require_client(), c.set_status)
    if result.info is not None:
        c.info = result.info
    if c.report:
        c.report.events.write("detection", result.as_dict())
    c.set_status(result.message)
    _log("event", "EVENT", f"Actuator Detection: {result.message}")


def _run_encoder_sanity() -> None:
    c = _ctx
    result = run_encoder_sanity_test(c.require_client(), c.store, c.safety, c.set_status)
    c.calibration.motor_encoder_sign = result.motor_encoder_sign
    c.calibration.output_encoder_sign = result.output_encoder_sign
    if result.warning:
        c.warnings.append(result.warning)
    if c.report:
        c.report.events.write("encoder_sanity", result.as_dict())
    c.set_status("Encoder sanity passed" if result.pass_test else result.warning or "Encoder sanity failed")
    _log("event", "EVENT", c.get_status())


def _run_ratio() -> None:
    c = _ctx
    idx = c.store.total_samples
    result = run_ratio_calibration(c.require_client(), c.store, c.info, c.safety, c.set_status)
    samples = c.store.samples_since(idx)
    c.ratio_fit = result.fit
    c.calibration = result.calibration
    if result.message and not result.passed:
        c.warnings.append(result.message)
    if c.report:
        c.report.events.write("ratio_calibration", result.as_dict())
        c.report.save_calibration(result.calibration)
        plot_ratio(samples, result.fit, c.report.artifacts.ratio_plot_png)
        plot_deflection(samples, result.fit.output_per_motor, result.fit.output_offset_rad, c.report.artifacts.deflection_plot_png)
        write_summary(c.report.artifacts.summary_txt, info=c.info, telemetry_store=c.store,
                      ratio_fit=result.fit, calibration=result.calibration, warnings=c.warnings)
    c.set_status(result.message)
    _log("event", "EVENT", f"Ratio Calibration: {result.message}")


def _run_resonance() -> None:
    c = _ctx
    idx = c.store.total_samples
    result = run_resonance_test(c.require_client(), c.store, c.calibration, c.safety, c.set_status)
    samples = c.store.samples_since(idx)
    c.resonance_result = result.analysis
    c.calibration = result.calibration
    if result.message and not result.passed:
        c.warnings.append(result.message)
    if c.report:
        c.report.events.write("resonance_test", result.as_dict())
        c.report.save_calibration(result.calibration)
        plot_resonance(samples, result.calibration.output_per_motor, result.calibration.output_offset_rad,
                       result.analysis, c.report.artifacts.resonance_plot_png)
        write_summary(c.report.artifacts.summary_txt, info=c.info, telemetry_store=c.store,
                      ratio_fit=c.ratio_fit, calibration=c.calibration, warnings=c.warnings)
    c.set_status(result.message)
    _log("event", "EVENT", f"Resonance Chirp: {result.message}")


def _run_backlash() -> None:
    c = _ctx
    result = run_backlash_test(c.require_client(), c.store, c.calibration, c.safety, c.set_status)
    c.calibration.backlash_motor_rad = result.backlash_motor_rad
    c.calibration.backlash_output_rad = result.backlash_output_rad
    c.calibration.backlash_comp_enabled = bool(result.pass_test and result.backlash_motor_rad > 1e-4)
    if result.warning:
        c.warnings.append(result.warning)
    if c.report:
        c.report.events.write("backlash_test", result.as_dict())
        c.report.save_calibration(c.calibration)
        write_summary(c.report.artifacts.summary_txt, info=c.info, telemetry_store=c.store,
                      ratio_fit=c.ratio_fit, calibration=c.calibration, warnings=c.warnings)
    c.set_status("Backlash passed" if result.pass_test else result.warning or "Backlash failed")
    _log("event", "EVENT", c.get_status())


def _run_step_response() -> None:
    c = _ctx
    result = run_step_response_test(c.require_client(), c.store, c.safety, c.set_status, calibration=c.calibration)
    c.calibration.settling_time_s = result.settling_time_s
    c.calibration.overshoot_percent = result.overshoot_percent
    if result.warning:
        c.warnings.append(result.warning)
    if c.report:
        c.report.events.write("step_response_test", result.as_dict())
        c.report.save_calibration(c.calibration)
        write_summary(c.report.artifacts.summary_txt, info=c.info, telemetry_store=c.store,
                      ratio_fit=c.ratio_fit, calibration=c.calibration, warnings=c.warnings)
    c.set_status("Step response passed" if result.pass_test else result.warning or "Step response failed")
    _log("event", "EVENT", c.get_status())


def _run_velocity_ramp() -> None:
    c = _ctx
    result = run_velocity_ramp_test(c.require_client(), c.store, c.calibration, c.safety, c.set_status)
    c.calibration.max_safe_velocity_rad_s = result.recommended_velocity_limit_rad_s
    c.calibration.max_safe_accel_rad_s2 = result.recommended_accel_limit_rad_s2
    if result.resonance_frequency_hz is not None:
        c.calibration.resonance_frequency_hz = result.resonance_frequency_hz
    if result.warning:
        c.warnings.append(result.warning)
    if c.report:
        c.report.events.write("velocity_ramp_test", result.as_dict())
        c.report.save_calibration(c.calibration)
        write_summary(c.report.artifacts.summary_txt, info=c.info, telemetry_store=c.store,
                      ratio_fit=c.ratio_fit, calibration=c.calibration, warnings=c.warnings)
    c.set_status("Velocity ramp passed" if result.pass_test else result.warning or "Velocity ramp failed")
    _log("event", "EVENT", c.get_status())


def _run_compliance() -> None:
    c = _ctx
    result = run_compliance_test(c.require_client(), c.store, c.calibration, c.set_status)
    c.calibration.compliance_nm_per_rad = result.stiffness_nm_per_rad
    if result.warning:
        c.warnings.append(result.warning)
    if c.report:
        c.report.events.write("compliance_test", result.as_dict())
        c.report.save_calibration(c.calibration)
        write_summary(c.report.artifacts.summary_txt, info=c.info, telemetry_store=c.store,
                      ratio_fit=c.ratio_fit, calibration=c.calibration, warnings=c.warnings)
    c.set_status("Compliance passed" if result.pass_test else result.warning or "Compliance failed")
    _log("event", "EVENT", c.get_status())


def _run_auto() -> None:
    _run_detection()
    _run_encoder_sanity()
    _run_ratio()
    _run_resonance()
    _run_backlash()
    _run_step_response()
    _run_velocity_ramp()
    _run_compliance()
    _save_config()
    if _ctx.report:
        _ctx.report.events.write("auto_characterization", {"scope": "full_hardware_suite"})
    _ctx.set_status("Full auto-characterization complete; calibration saved to actuator")
    _log("event", "EVENT", "Full auto-characterization complete")


def _save_config() -> None:
    c = _ctx
    client = c.require_client()
    cal = c.calibration
    for k, v in [
        ("output_per_motor", cal.output_per_motor),
        ("output_offset_rad", cal.output_offset_rad),
        ("pid_enabled", cal.pid_enabled),
        ("pid_kp", cal.pid_kp),
        ("pid_ki", cal.pid_ki),
        ("pid_kd", cal.pid_kd),
        ("pid_i_limit_motor_rad", cal.pid_i_limit_motor_rad),
        ("pid_output_limit_motor_rad", cal.pid_output_limit_motor_rad),
        ("velocity_pid_kp", cal.velocity_pid_kp),
        ("velocity_pid_ki", cal.velocity_pid_ki),
        ("velocity_pid_i_limit_motor_rad", cal.velocity_pid_i_limit_motor_rad),
        ("torque_proxy_kp", cal.torque_proxy_kp),
        ("torque_proxy_limit_rad", cal.torque_proxy_limit_rad),
        ("torque_proxy_max_motor_velocity_rad_s", cal.torque_proxy_max_motor_velocity_rad_s),
        ("torque_proxy_timeout_s", cal.torque_proxy_timeout_s),
        ("missed_step_correction_enabled", cal.missed_step_correction_enabled),
        ("missed_step_warn_motor_rad", cal.missed_step_warn_motor_rad),
        ("missed_step_fault_motor_rad", cal.missed_step_fault_motor_rad),
        ("missed_step_correction_rate", cal.missed_step_correction_rate),
        ("current_control_enabled", cal.current_control_enabled),
        ("idle_current_ma", cal.idle_current_ma),
        ("hold_current_ma", cal.hold_current_ma),
        ("run_current_ma", cal.run_current_ma),
        ("current_downshift_delay_s", cal.current_downshift_delay_s),
        ("autotune_max_amplitude_rad", cal.autotune_max_amplitude_rad),
        ("autotune_max_duration_s", cal.autotune_max_duration_s),
        ("autotune_max_deflection_rad", cal.autotune_max_deflection_rad),
        ("backlash_motor_rad", cal.backlash_motor_rad),
        ("backlash_comp_enabled", cal.backlash_comp_enabled),
        ("resonance_frequency_hz", cal.resonance_frequency_hz),
        ("resonance_derating_enabled", cal.resonance_derating_enabled),
    ]:
        client.set_config(k, v)
        _log("tx", "TX", f"SET_CONFIG {k}={v}")
    client.save_config()
    _log("tx", "TX", "SAVE_CONFIG")
    if c.report:
        c.report.events.write("save_config_to_actuator", cal.as_dict())
    c.set_status("Calibration saved to actuator")
    _log("event", "EVENT", "Calibration saved to actuator")


def _format_fault_flags(value: int) -> str:
    faults = FaultFlags(value)
    if faults == FaultFlags.NONE:
        return "NONE"
    names = [
        flag.name
        for flag in FaultFlags
        if flag != FaultFlags.NONE and flag in faults
    ]
    return " | ".join(names) if names else str(int(faults))


TEST_DEFS = [
    ("detection", "Actuator Detection", "PING/PONG, read INFO block", True, "test_detection_state", "test_detection_result", "run_detection"),
    ("encoder", "Encoder Sanity", "Verify motor and output encoder polarity", True, "test_encoder_state", "test_encoder_result", "run_encoder_sanity"),
    ("ratio", "Ratio Calibration", "Bi-directional sweep, fit output/motor ratio", True, "test_ratio_state", "test_ratio_result", "run_ratio"),
    ("resonance", "Resonance Chirp", "Frequency sweep, find peak Hz", True, "test_resonance_state", "test_resonance_result", "run_resonance"),
    ("backlash", "Backlash Test", "Reversal deadband measurement", False, "test_backlash_state", "test_backlash_result", "run_backlash"),
    ("step", "Step Response", "Settling time and overshoot characterisation", False, "test_step_state", "test_step_result", "run_step_response"),
    ("velocity", "Velocity Ramp", "Safe velocity and acceleration limits", False, "test_velocity_state", "test_velocity_result", "run_velocity_ramp"),
    ("compliance", "Compliance / Load", "Output stiffness in N*m/rad", False, "test_compliance_state", "test_compliance_result", "run_compliance"),
    ("save", "Save to Actuator Flash", "Write fitted calibration constants to flash", True, "test_save_state", "test_save_result", "save_config"),
]

TEST_ATTRS = {item[0]: (item[4], item[5]) for item in TEST_DEFS}


# Reflex state

class State(rx.State):
    # Connection
    use_sim: bool = _DEFAULT_USE_SIM
    port: str = _DEFAULT_PORT
    available_ports: list[str] = []
    connected: bool = False

    # Live status
    status: str = "Disconnected"
    actuator_id: str = ""
    firmware: str = ""
    hardware: str = ""
    mode_name: str = ""
    fault_text: str = ""

    # Live telemetry
    motor_angle_str: str = "-"
    output_angle_str: str = "-"
    deflection_str: str = "-"
    motor_velocity_str: str = "-"
    bus_voltage_str: str = "-"
    temperature_str: str = "-"
    diag_motor_x: str = "79"
    diag_motor_y: str = "51"
    diag_output_x: str = "224"
    diag_output_y: str = "39"
    diag_motor_label: str = "0.0000 rad"
    diag_output_label: str = "0.0000 rad"
    diag_deflection_label: str = "Delta +0.0000 rad"

    # Calibration / results
    ratio_str: str = "-"
    offset_str: str = "-"
    fit_error_str: str = "-"
    hysteresis_str: str = "-"
    resonance_str: str = "-"
    backlash_str: str = "-"
    sample_count: int = 0
    dropped: int = 0
    warnings_text: str = "none"

    # Motion params
    jog_step: float = 0.25
    move_delta: float = 1.0
    move_velocity: float = 1.2
    move_accel: float = 10.0

    # Config tab
    cfg_max_velocity: str = "40"
    cfg_max_accel: str = "1000"
    cfg_jog_step: str = "0.25"
    cfg_cal_velocity: str = "3.5"
    cfg_cal_accel: str = "80"
    cfg_max_move: str = "60"
    cfg_output_per_motor: str = "1"
    cfg_output_offset: str = "0"
    cfg_resonance_frequency: str = ""
    cfg_pid_enabled: bool = False
    cfg_pid_kp: str = "0"
    cfg_pid_ki: str = "0"
    cfg_pid_kd: str = "0"
    cfg_pid_i_limit: str = "0.05"
    cfg_pid_output_limit: str = "0.25"
    cfg_backlash_motor: str = "0"
    cfg_backlash_comp_enabled: bool = False
    cfg_resonance_derating_enabled: bool = False

    # Test pipeline
    test_detection_state: str = "idle"
    test_detection_result: str = ""
    test_encoder_state: str = "idle"
    test_encoder_result: str = ""
    test_ratio_state: str = "idle"
    test_ratio_result: str = ""
    test_resonance_state: str = "idle"
    test_resonance_result: str = ""
    test_backlash_state: str = "idle"
    test_backlash_result: str = ""
    test_step_state: str = "idle"
    test_step_result: str = ""
    test_velocity_state: str = "idle"
    test_velocity_result: str = ""
    test_compliance_state: str = "idle"
    test_compliance_result: str = ""
    test_save_state: str = "idle"
    test_save_result: str = ""

    # Report/log
    report_folder: str = ""
    notes: str = ""
    log_entries: list[dict[str, str]] = []
    log_filter: str = "all"
    log_auto_scroll: bool = True

    # UI flags
    active_tab: str = "overview"
    sidebar_collapsed: bool = False
    busy: bool = False

    @rx.var
    def can_control(self) -> bool:
        return self.connected and not self.busy

    @rx.var
    def can_test(self) -> bool:
        return self.connected and self.mode_name == "CALIBRATION" and not self.busy

    @rx.var
    def visible_log_entries(self) -> list[dict[str, str]]:
        if self.log_filter == "all":
            return self.log_entries[-180:]
        return [entry for entry in self.log_entries if entry["kind"] == self.log_filter][-180:]

    @rx.var
    def mode_display(self) -> str:
        return self.mode_name or "DISABLED"

    @rx.var
    def fault_display(self) -> str:
        return self.fault_text or "NONE"

    def set_use_sim(self, v: bool) -> None:
        self.use_sim = v

    def set_port(self, v: str) -> None:
        self.port = v

    def set_notes(self, v: str) -> None:
        self.notes = v

    def set_active_tab(self, tab: str) -> None:
        self.active_tab = tab

    def toggle_sidebar(self) -> None:
        self.sidebar_collapsed = not self.sidebar_collapsed

    def set_log_filter(self, value: str) -> None:
        self.log_filter = value

    def set_log_auto_scroll(self, value: bool) -> None:
        self.log_auto_scroll = value

    def clear_log(self) -> None:
        _ctx.clear_logs()
        self.log_entries = []
        self.status = "Log cleared"

    def set_jog_step(self, v: str) -> None:
        try:
            self.jog_step = float(v)
            self.cfg_jog_step = v
        except ValueError:
            pass

    def set_move_delta(self, v: str) -> None:
        try:
            self.move_delta = float(v)
        except ValueError:
            pass

    def set_move_velocity(self, v: str) -> None:
        try:
            self.move_velocity = float(v)
        except ValueError:
            pass

    def set_move_accel(self, v: str) -> None:
        try:
            self.move_accel = float(v)
        except ValueError:
            pass

    def set_cfg_max_velocity(self, v: str) -> None:
        self.cfg_max_velocity = v

    def set_cfg_max_accel(self, v: str) -> None:
        self.cfg_max_accel = v

    def set_cfg_jog_step(self, v: str) -> None:
        self.cfg_jog_step = v
        try:
            self.jog_step = float(v)
        except ValueError:
            pass

    def set_cfg_cal_velocity(self, v: str) -> None:
        self.cfg_cal_velocity = v

    def set_cfg_cal_accel(self, v: str) -> None:
        self.cfg_cal_accel = v

    def set_cfg_max_move(self, v: str) -> None:
        self.cfg_max_move = v

    def set_cfg_output_per_motor(self, v: str) -> None:
        self.cfg_output_per_motor = v

    def set_cfg_output_offset(self, v: str) -> None:
        self.cfg_output_offset = v

    def set_cfg_resonance_frequency(self, v: str) -> None:
        self.cfg_resonance_frequency = v

    def set_cfg_pid_kp(self, v: str) -> None:
        self.cfg_pid_kp = v

    def set_cfg_pid_ki(self, v: str) -> None:
        self.cfg_pid_ki = v

    def set_cfg_pid_kd(self, v: str) -> None:
        self.cfg_pid_kd = v

    def set_cfg_pid_i_limit(self, v: str) -> None:
        self.cfg_pid_i_limit = v

    def set_cfg_pid_output_limit(self, v: str) -> None:
        self.cfg_pid_output_limit = v

    def set_cfg_backlash_motor(self, v: str) -> None:
        self.cfg_backlash_motor = v

    def enable_pid(self) -> None:
        self.cfg_pid_enabled = True

    def disable_pid(self) -> None:
        self.cfg_pid_enabled = False

    def enable_backlash_comp(self) -> None:
        self.cfg_backlash_comp_enabled = True

    def disable_backlash_comp(self) -> None:
        self.cfg_backlash_comp_enabled = False

    def enable_resonance_derating(self) -> None:
        self.cfg_resonance_derating_enabled = True

    def disable_resonance_derating(self) -> None:
        self.cfg_resonance_derating_enabled = False

    def scan_ports(self) -> None:
        self.available_ports = scan_serial_ports()
        self.status = f"Found {len(self.available_ports)} port(s)"
        _log("event", "EVENT", self.status)
        self.log_entries = _ctx.get_logs()

    @rx.event(background=True)
    async def do_connect(self) -> None:
        async with self:
            self.busy = True
            self.status = "Connecting..."
            use_sim, port = self.use_sim, self.port
        loop = asyncio.get_event_loop()
        try:
            await loop.run_in_executor(_executor, lambda: _connect(use_sim, port))
            async with self:
                self.connected = True
                self.actuator_id = _ctx.info.actuator_id
                self.firmware = _ctx.info.firmware_version
                self.hardware = _ctx.info.hardware_revision
                self.status = _ctx.get_status()
                self._sync_results()
                self._sync_config()
                self.log_entries = _ctx.get_logs()
        except Exception as exc:
            _log("fault", "FAULT", f"Connect failed: {exc}")
            async with self:
                self.status = f"Connect failed: {exc}"
                self.log_entries = _ctx.get_logs()
        finally:
            async with self:
                self.busy = False

    @rx.event(background=True)
    async def do_disconnect(self) -> None:
        async with self:
            self.busy = True
        loop = asyncio.get_event_loop()
        try:
            await loop.run_in_executor(_executor, _disconnect)
            async with self:
                self.connected = False
                self.mode_name = ""
                self.fault_text = ""
                self.status = "Disconnected"
                self.log_entries = _ctx.get_logs()
        except Exception as exc:
            _log("fault", "FAULT", f"Disconnect error: {exc}")
            async with self:
                self.status = f"Disconnect error: {exc}"
                self.log_entries = _ctx.get_logs()
        finally:
            async with self:
                self.busy = False

    def _set_status(self, message: str, kind: str = "event", tag: str = "EVENT") -> None:
        _ctx.set_status(message)
        _log(kind, tag, message)
        self.status = message
        self.log_entries = _ctx.get_logs()

    def mode_disabled(self) -> None:
        try:
            _log("tx", "TX", "SET_MODE DISABLED")
            _ctx.require_client().set_mode(ActuatorMode.DISABLED)
            self._set_status("Mode: DISABLED")
        except Exception as exc:
            self._set_status(str(exc), "fault", "FAULT")

    def mode_calibration(self) -> None:
        try:
            _log("tx", "TX", "SET_MODE CALIBRATION")
            _ctx.require_client().set_mode(ActuatorMode.CALIBRATION)
            self._set_status("Mode: CALIBRATION")
        except Exception as exc:
            self._set_status(str(exc), "fault", "FAULT")

    def do_stop(self) -> None:
        try:
            _log("tx", "TX", "STOP")
            _ctx.require_client().stop()
            self._set_status("Stop sent")
        except Exception as exc:
            self._set_status(str(exc), "fault", "FAULT")

    def do_estop(self) -> None:
        try:
            _log("tx", "TX", "ESTOP")
            _ctx.require_client().estop()
            self._set_status("ESTOP sent", "fault", "FAULT")
        except Exception as exc:
            self._set_status(str(exc), "fault", "FAULT")

    def jog_pos(self) -> None:
        self._jog(1.0)

    def jog_neg(self) -> None:
        self._jog(-1.0)

    def _jog(self, sign: float) -> None:
        latest = _ctx.store.latest()
        if latest is None or latest.mode != int(ActuatorMode.CALIBRATION):
            self._set_status("Jog requires CALIBRATION mode", "event", "EVENT")
            return
        try:
            delta = sign * self.jog_step
            _log("tx", "TX", f"MOVE_REL {delta:.3f} rad")
            _ctx.require_client().move_rel(delta, self.move_velocity, self.move_accel)
            self._set_status(f"Jog {delta:.3f} rad")
        except Exception as exc:
            self._set_status(str(exc), "fault", "FAULT")

    @rx.event(background=True)
    async def do_move_rel(self) -> None:
        async with self:
            delta, vel, accel = self.move_delta, self.move_velocity, self.move_accel
        loop = asyncio.get_event_loop()
        try:
            _log("tx", "TX", f"MOVE_REL {delta:.3f} rad @ {vel:.3f} rad/s")
            await loop.run_in_executor(
                _executor,
                lambda: _ctx.require_client().move_rel(delta, vel, accel),
            )
            message = f"MOVE_REL {delta:.3f} rad"
            _ctx.set_status(message)
            _log("event", "EVENT", message)
            async with self:
                self.status = message
                self.log_entries = _ctx.get_logs()
        except Exception as exc:
            _ctx.set_status(str(exc))
            _log("fault", "FAULT", str(exc))
            async with self:
                self.status = str(exc)
                self.log_entries = _ctx.get_logs()

    def zero_motor(self) -> None:
        try:
            _log("tx", "TX", "ZERO_MOTOR_ENCODER")
            _ctx.require_client().zero_motor_encoder()
            self._set_status("Motor encoder zeroed")
        except Exception as exc:
            self._set_status(str(exc), "fault", "FAULT")

    def zero_output(self) -> None:
        try:
            _log("tx", "TX", "ZERO_OUTPUT_ENCODER")
            _ctx.require_client().zero_output_encoder()
            self._set_status("Output encoder zeroed")
        except Exception as exc:
            self._set_status(str(exc), "fault", "FAULT")

    @rx.event(background=True)
    async def run_detection(self) -> None:
        await self._worker("detection", "Actuator Detection", _run_detection, requires_test=False)

    @rx.event(background=True)
    async def run_encoder_sanity(self) -> None:
        await self._worker("encoder", "Encoder Sanity", _run_encoder_sanity)

    @rx.event(background=True)
    async def run_ratio(self) -> None:
        await self._worker("ratio", "Ratio Calibration", _run_ratio)

    @rx.event(background=True)
    async def run_resonance(self) -> None:
        await self._worker("resonance", "Resonance Chirp", _run_resonance)

    @rx.event(background=True)
    async def run_backlash(self) -> None:
        await self._worker("backlash", "Backlash Test", _run_backlash)

    @rx.event(background=True)
    async def run_step_response(self) -> None:
        await self._worker("step", "Step Response", _run_step_response)

    @rx.event(background=True)
    async def run_velocity_ramp(self) -> None:
        await self._worker("velocity", "Velocity Ramp", _run_velocity_ramp)

    @rx.event(background=True)
    async def run_compliance(self) -> None:
        await self._worker("compliance", "Compliance / Load", _run_compliance)

    @rx.event(background=True)
    async def run_auto(self) -> None:
        async with self:
            for key, *_rest in TEST_DEFS:
                self._set_test_ui(key, "idle", "")
        await self._worker("detection", "Auto-characterization", _run_auto)
        async with self:
            for key, *_rest in TEST_DEFS:
                state_attr, result_attr = TEST_ATTRS[key]
                if getattr(self, state_attr) == "idle":
                    setattr(self, state_attr, "pass")
                    setattr(self, result_attr, "Completed during auto-characterization")

    @rx.event(background=True)
    async def save_config(self) -> None:
        await self._worker("save", "Save to Actuator Flash", _save_config, requires_test=False)

    async def _worker(self, key: str, label: str, fn, requires_test: bool = True) -> None:
        async with self:
            can_run_test = self.connected and self.mode_name == "CALIBRATION" and not self.busy
            if requires_test and not can_run_test:
                self.status = "Requires CALIBRATION mode"
                self._set_test_ui(key, "fail", self.status)
                return
            self.busy = True
            self.status = f"{label} started..."
            self._set_test_ui(key, "running", "")
            start_warning_count = len(_ctx.warnings)
            _log("event", "EVENT", self.status)
        loop = asyncio.get_event_loop()
        try:
            await loop.run_in_executor(_executor, fn)
            async with self:
                self.status = _ctx.get_status()
                done_state = "warn" if len(_ctx.warnings) > start_warning_count else "pass"
                self._set_test_ui(key, done_state, self.status)
                self._sync_results()
                self._sync_config()
                self.log_entries = _ctx.get_logs()
        except Exception as exc:
            _log("fault", "FAULT", f"{label} failed: {exc}")
            async with self:
                self.status = f"{label} failed: {exc}"
                self._set_test_ui(key, "fail", str(exc))
                self.log_entries = _ctx.get_logs()
        finally:
            async with self:
                self.busy = False

    def _set_test_ui(self, key: str, state: str, result: str) -> None:
        state_attr, result_attr = TEST_ATTRS[key]
        setattr(self, state_attr, state)
        setattr(self, result_attr, result)

    def _config_payload(self) -> tuple[dict[str, float], dict[str, Any]]:
        resonance_text = self.cfg_resonance_frequency.strip()
        safety_values = {
            "max_velocity_rad_s": _parse_float(self.cfg_max_velocity, "Max velocity"),
            "max_accel_rad_s2": _parse_float(self.cfg_max_accel, "Max accel"),
            "jog_step_rad": _parse_float(self.cfg_jog_step, "Jog step"),
            "calibration_velocity_rad_s": _parse_float(self.cfg_cal_velocity, "Calibration velocity"),
            "calibration_accel_rad_s2": _parse_float(self.cfg_cal_accel, "Calibration accel"),
            "max_move_rad": _parse_float(self.cfg_max_move, "Max move delta"),
        }
        cal_values = {
            "output_per_motor": _parse_float(self.cfg_output_per_motor, "Output per motor"),
            "output_offset_rad": _parse_float(self.cfg_output_offset, "Output offset"),
            "resonance_frequency_hz": None if resonance_text == "" else _parse_float(resonance_text, "Resonance frequency"),
            "pid_enabled": self.cfg_pid_enabled,
            "pid_kp": _parse_float(self.cfg_pid_kp, "Kp"),
            "pid_ki": _parse_float(self.cfg_pid_ki, "Ki"),
            "pid_kd": _parse_float(self.cfg_pid_kd, "Kd"),
            "pid_i_limit_motor_rad": _parse_float(self.cfg_pid_i_limit, "I-limit"),
            "pid_output_limit_motor_rad": _parse_float(self.cfg_pid_output_limit, "Output limit"),
            "backlash_motor_rad": _parse_float(self.cfg_backlash_motor, "Backlash"),
            "backlash_comp_enabled": self.cfg_backlash_comp_enabled,
            "resonance_derating_enabled": self.cfg_resonance_derating_enabled,
        }
        return safety_values, cal_values

    @rx.event(background=True)
    async def save_ui_config(self) -> None:
        async with self:
            self.busy = True
            self.status = "Saving config..."
            try:
                safety_values, cal_values = self._config_payload()
            except Exception as exc:
                self.status = str(exc)
                self.busy = False
                return
        loop = asyncio.get_event_loop()
        try:
            await loop.run_in_executor(_executor, lambda: _apply_ui_config(safety_values, cal_values))
            async with self:
                self.status = _ctx.get_status()
                self._sync_results()
                self._sync_config()
                self.log_entries = _ctx.get_logs()
        except Exception as exc:
            _log("fault", "FAULT", f"Config save failed: {exc}")
            async with self:
                self.status = f"Config save failed: {exc}"
                self.log_entries = _ctx.get_logs()
        finally:
            async with self:
                self.busy = False

    @rx.event(background=True)
    async def reload_config(self) -> None:
        async with self:
            self.busy = True
            self.status = "Reading config..."
        loop = asyncio.get_event_loop()
        try:
            if _ctx.connected:
                await loop.run_in_executor(_executor, _read_device_config)
            async with self:
                self.status = _ctx.get_status() if _ctx.connected else "Local config restored"
                self._sync_config()
                self._sync_results()
                self.log_entries = _ctx.get_logs()
        except Exception as exc:
            _log("fault", "FAULT", f"Config read failed: {exc}")
            async with self:
                self.status = f"Config read failed: {exc}"
                self.log_entries = _ctx.get_logs()
        finally:
            async with self:
                self.busy = False

    def _sync_results(self) -> None:
        cal = _ctx.calibration
        stats = _ctx.store.stats
        self.ratio_str = f"{cal.output_per_motor:.8f}"
        self.offset_str = f"{cal.output_offset_rad:.8f} rad"
        self.hysteresis_str = f"{cal.hysteresis_rad:.8f} rad"
        self.backlash_str = f"{cal.backlash_motor_rad:.8f} rad"
        self.sample_count = stats.total_samples
        self.dropped = stats.dropped_samples
        if _ctx.ratio_fit is not None:
            self.fit_error_str = f"{_ctx.ratio_fit.residual_rms_rad:.8f} rad RMS"
        res = _ctx.resonance_result
        if res is not None and res.peak_frequency_hz is not None:
            self.resonance_str = f"{res.peak_frequency_hz:.2f} Hz"
        elif cal.resonance_frequency_hz is not None:
            self.resonance_str = f"{cal.resonance_frequency_hz:.2f} Hz"
        if _ctx.warnings:
            self.warnings_text = "\n".join(_ctx.warnings[-6:])
        else:
            self.warnings_text = "none"
        if _ctx.report is not None:
            self.report_folder = str(_ctx.report.artifacts.folder)

    def _sync_config(self) -> None:
        safety = _ctx.safety
        cal = _ctx.calibration
        self.cfg_max_velocity = _format_config_number(safety.max_velocity_rad_s, 4)
        self.cfg_max_accel = _format_config_number(safety.max_accel_rad_s2, 4)
        self.cfg_jog_step = _format_config_number(safety.jog_step_rad, 4)
        self.cfg_cal_velocity = _format_config_number(safety.calibration_velocity_rad_s, 4)
        self.cfg_cal_accel = _format_config_number(safety.calibration_accel_rad_s2, 4)
        self.cfg_max_move = _format_config_number(safety.max_move_rad, 4)
        self.cfg_output_per_motor = _format_config_number(cal.output_per_motor, 8)
        self.cfg_output_offset = _format_config_number(cal.output_offset_rad, 8)
        self.cfg_resonance_frequency = _format_config_number(cal.resonance_frequency_hz, 4)
        self.cfg_pid_enabled = cal.pid_enabled
        self.cfg_pid_kp = _format_config_number(cal.pid_kp, 6)
        self.cfg_pid_ki = _format_config_number(cal.pid_ki, 6)
        self.cfg_pid_kd = _format_config_number(cal.pid_kd, 6)
        self.cfg_pid_i_limit = _format_config_number(cal.pid_i_limit_motor_rad, 6)
        self.cfg_pid_output_limit = _format_config_number(cal.pid_output_limit_motor_rad, 6)
        self.cfg_backlash_motor = _format_config_number(cal.backlash_motor_rad, 8)
        self.cfg_backlash_comp_enabled = cal.backlash_comp_enabled
        self.cfg_resonance_derating_enabled = cal.resonance_derating_enabled
        self.jog_step = safety.jog_step_rad
        self.move_velocity = safety.calibration_velocity_rad_s
        self.move_accel = safety.calibration_accel_rad_s2

    def _sync_latest(self, latest: TelemetrySample | None) -> None:
        if latest is None:
            return
        cal = _ctx.calibration
        predicted = cal.output_per_motor * latest.motor_rad + cal.output_offset_rad
        deflection = latest.output_rad - predicted
        self.motor_angle_str = _float_text(latest.motor_rad, 4)
        self.output_angle_str = _float_text(latest.output_rad, 4)
        self.deflection_str = _float_text(deflection, 4)
        self.motor_velocity_str = _float_text(latest.motor_vel_rad_s, 3)
        self.bus_voltage_str = _float_text(latest.bus_voltage, 1)
        self.temperature_str = _float_text(latest.temperature, 1)
        motor_angle = latest.motor_rad - math.pi / 2
        output_angle = latest.output_rad - math.pi / 2
        self.diag_motor_x = f"{79 + 27 * math.cos(motor_angle):.1f}"
        self.diag_motor_y = f"{85 + 27 * math.sin(motor_angle):.1f}"
        self.diag_output_x = f"{224 + 37 * math.cos(output_angle):.1f}"
        self.diag_output_y = f"{85 + 37 * math.sin(output_angle):.1f}"
        self.diag_motor_label = f"{latest.motor_rad:.4f} rad"
        self.diag_output_label = f"{latest.output_rad:.4f} rad"
        self.diag_deflection_label = f"Delta {deflection:+.4f} rad"

    def export_report(self) -> None:
        if _ctx.report is None:
            self.status = "No active report session"
            return
        _ctx.report.save_notes(self.notes)
        write_summary(
            _ctx.report.artifacts.summary_txt,
            info=_ctx.info,
            telemetry_store=_ctx.store,
            ratio_fit=_ctx.ratio_fit,
            calibration=_ctx.calibration,
            warnings=_ctx.warnings,
        )
        self.status = f"Report updated: {_ctx.report.artifacts.folder}"
        _log("event", "EVENT", self.status)
        self._sync_results()
        self.log_entries = _ctx.get_logs()

    def open_folder(self) -> None:
        if _ctx.report is None:
            self.status = "No active report session"
            return
        subprocess.Popen(["explorer", str(_ctx.report.artifacts.folder.resolve())])

    @rx.event(background=True)
    async def start_polling(self) -> None:
        _start_bokeh()
        last_logged_sample = 0
        last_fault_text = ""
        while True:
            await asyncio.sleep(0.5)
            latest = _ctx.store.latest()
            stats = _ctx.store.stats
            if latest is not None and stats.total_samples - last_logged_sample >= 500:
                _log(
                    "rx",
                    "RX",
                    f"TELEMETRY seq={latest.seq} motor={latest.motor_rad:.4f} output={latest.output_rad:.4f}",
                )
                last_logged_sample = stats.total_samples
            fault_text = _format_fault_flags(latest.fault_flags) if latest is not None else ""
            if fault_text and fault_text != "NONE" and fault_text != last_fault_text:
                _log("fault", "FAULT", fault_text)
            if fault_text:
                last_fault_text = fault_text
            async with self:
                self.status = _ctx.get_status()
                self.connected = _ctx.connected
                if _ctx.connected:
                    self.actuator_id = _ctx.info.actuator_id
                    self.firmware = _ctx.info.firmware_version
                    self.hardware = _ctx.info.hardware_revision
                if latest is not None:
                    self.mode_name = latest.mode_name
                    self.fault_text = fault_text
                    self._sync_latest(latest)
                self.sample_count = stats.total_samples
                self.dropped = stats.dropped_samples
                self.log_entries = _ctx.get_logs()


# UI styles and helpers

DESIGN_CSS = """
* { box-sizing: border-box; }
html, body { margin: 0; min-height: 100%; overflow: hidden; background: #0d1117; }
body { font-family: 'Space Grotesk', system-ui, sans-serif; color: #e6edf3; font-size: 13px; }
.app-shell { display: flex; flex-direction: column; height: 100vh; width: 100%; background: #0d1117; color: #e6edf3; overflow: hidden; }
.hdr { height: 52px; background: #080b10; border-bottom: 1px solid #30363d; display: flex; align-items: center; padding: 0 14px; gap: 12px; flex-shrink: 0; z-index: 100; }
.hdr-logo { display: flex; align-items: center; gap: 8px; font-weight: 600; font-size: 14px; white-space: nowrap; color: #e6edf3; }
.hdr-logo svg { color: #2f81f7; }
.hdr-sep { width: 1px; height: 22px; background: #30363d; flex-shrink: 0; }
.hdr-group { display: flex; align-items: center; gap: 8px; min-width: 0; }
.hdr-spacer { flex: 1; }
.hdr-status { font-family: 'JetBrains Mono', monospace; font-size: 11px; color: #8b949e; max-width: 340px; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
.body { display: flex; flex: 1; overflow: hidden; min-height: 0; }
.sb { width: 264px; flex-shrink: 0; background: #080b10; border-right: 1px solid #30363d; display: flex; flex-direction: column; overflow-y: auto; overflow-x: hidden; transition: width .24s cubic-bezier(.4,0,.2,1); }
.sb::-webkit-scrollbar, .tab-pane::-webkit-scrollbar, .log-feed::-webkit-scrollbar { width: 4px; }
.sb::-webkit-scrollbar-thumb, .tab-pane::-webkit-scrollbar-thumb, .log-feed::-webkit-scrollbar-thumb { background: #30363d; border-radius: 2px; }
.sb.collapsed { width: 40px; }
.sb.collapsed .sb-sec { display: none; }
.sb-toggle { display: flex; align-items: center; justify-content: space-between; padding: 9px 10px 9px 12px; border: 0; border-bottom: 1px solid #21262d; background: transparent; color: #484f58; cursor: pointer; width: 100%; }
.sb-toggle:hover { color: #8b949e; }
.sb-toggle-label { font-size: 10px; font-weight: 600; letter-spacing: .09em; text-transform: uppercase; }
.sb-toggle-icon { width: 20px; height: 20px; display: flex; align-items: center; justify-content: center; border: 1px solid #30363d; border-radius: 4px; background: #1c2128; flex-shrink: 0; }
.sb.collapsed .sb-toggle { justify-content: center; padding: 9px 0; }
.sb.collapsed .sb-toggle-label { display: none; }
.sb-sec { padding: 12px; border-bottom: 1px solid #21262d; }
.sb-lbl, .card-title, .cc-title, .cfg-sec-title, .rep-sec-title, .mc-lbl { font-size: 10px; font-weight: 600; letter-spacing: .08em; text-transform: uppercase; color: #484f58; }
.sb-lbl { margin-bottom: 8px; }
.sb-row { display: flex; gap: 6px; align-items: center; width: 100%; }
.sb-row + .sb-row { margin-top: 6px; }
.lbl-sm { font-size: 11px; color: #8b949e; margin-bottom: 2px; }
.mt6 { margin-top: 6px; }
.mt8 { margin-top: 8px; }
.main { flex: 1; display: flex; flex-direction: column; overflow: hidden; min-width: 0; }
.tab-bar { display: flex; border-bottom: 1px solid #30363d; background: #080b10; padding: 0 14px; flex-shrink: 0; overflow-x: auto; }
.tab { padding: 12px 15px; font-size: 13px; font-weight: 500; color: #8b949e; cursor: pointer; border: 0; border-bottom: 2px solid transparent; background: transparent; white-space: nowrap; font-family: 'Space Grotesk', system-ui, sans-serif; }
.tab:hover { color: #e6edf3; }
.tab.on { color: #e6edf3; border-bottom-color: #2f81f7; }
.tab-wrap { flex: 1; overflow: hidden; position: relative; min-height: 0; }
.tab-pane { position: absolute; inset: 0; overflow-y: auto; padding: 14px; }
.metric-row { display: grid; grid-template-columns: repeat(6, minmax(118px, 1fr)); gap: 8px; margin-bottom: 12px; }
.mc, .cc, .diagram-card, .results-card, .cfg-sec, .rep-sec { background: #161b22; border: 1px solid #30363d; border-radius: 6px; padding: 10px 12px; }
.mc { min-width: 0; }
.mc.hl { border-color: #2f81f7; }
.mc.red { border-color: #f85149; }
.mc-val { font-family: 'JetBrains Mono', monospace; font-size: 19px; font-weight: 500; color: #e6edf3; line-height: 1.15; white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
.mc.hl .mc-val { color: #2f81f7; }
.mc.red .mc-val { color: #f85149; }
.mc-unit { font-family: 'JetBrains Mono', monospace; font-size: 11px; color: #484f58; }
.ov-grid { display: grid; grid-template-columns: minmax(420px, 1fr) 328px; gap: 10px; align-items: start; }
.chart-stack, .side-col { display: flex; flex-direction: column; gap: 8px; min-width: 0; }
.telemetry-frame { width: 100%; height: 430px; border: 0; display: block; border-radius: 4px; background: #080b10; }
.res-row, .rep-row, .cfg-row { display: flex; justify-content: space-between; align-items: center; gap: 10px; min-width: 0; }
.res-row, .rep-row { padding: 4px 0; border-bottom: 1px solid #21262d; }
.res-row:last-child, .rep-row:last-child { border-bottom: none; }
.res-k, .rep-k, .cfg-k { color: #8b949e; font-size: 12px; }
.res-v, .rep-v { font-family: 'JetBrains Mono', monospace; font-size: 12px; color: #e6edf3; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
.badge, .conn-pill { display: inline-flex; align-items: center; gap: 6px; border-radius: 4px; border: 1px solid #30363d; background: #1c2128; color: #8b949e; font-family: 'JetBrains Mono', monospace; font-size: 11px; white-space: nowrap; line-height: 1; }
.badge { padding: 4px 8px; }
.conn-pill { padding: 5px 10px; border-radius: 20px; font-family: 'Space Grotesk', system-ui, sans-serif; font-size: 12px; font-weight: 500; }
.badge.acc { border-color: #2f81f7; color: #2f81f7; background: rgba(47,129,247,.14); }
.badge.green, .conn-pill.live { border-color: #3fb950; color: #3fb950; background: rgba(63,185,80,.12); }
.badge.red { border-color: #f85149; color: #f85149; background: rgba(248,81,73,.12); }
.badge.amber { border-color: #d29922; color: #d29922; background: rgba(210,153,34,.12); }
.pill-dot { width: 6px; height: 6px; border-radius: 50%; background: currentColor; flex-shrink: 0; }
.btn { min-height: 30px; padding: 5px 12px; border-radius: 4px; border: 1px solid #30363d; background: #1c2128; color: #e6edf3; font-family: 'Space Grotesk', system-ui, sans-serif; font-size: 12px; font-weight: 500; cursor: pointer; white-space: nowrap; display: inline-flex; align-items: center; justify-content: center; gap: 6px; }
.btn:hover { border-color: #8b949e; background: #21262d; }
.btn:disabled { opacity: .35; cursor: not-allowed; pointer-events: none; }
.btn.pr { background: #2f81f7; border-color: #2f81f7; color: #fff; }
.btn.dn { background: rgba(248,81,73,.12); border-color: #f85149; color: #f85149; }
.btn.gn { background: rgba(63,185,80,.12); border-color: #3fb950; color: #3fb950; }
.btn.am { background: rgba(210,153,34,.12); border-color: #d29922; color: #d29922; }
.btn.estop { min-height: 34px; padding: 7px 16px; background: #ff3333; border-color: #ff3333; color: #fff; font-size: 13px; font-weight: 700; letter-spacing: .06em; }
.btn.fw { width: 100%; }
.jog-btn { width: 38px; height: 38px; padding: 0; border-radius: 4px; border: 1px solid #30363d; background: #1c2128; color: #e6edf3; font-size: 20px; line-height: 1; cursor: pointer; display: flex; align-items: center; justify-content: center; flex-shrink: 0; }
.jog-btn:hover { border-color: #2f81f7; color: #2f81f7; background: rgba(47,129,247,.14); }
.seg { display: flex; background: #161b22; border: 1px solid #30363d; border-radius: 4px; padding: 2px; gap: 2px; width: 100%; }
.seg-btn { flex: 1; padding: 5px 7px; text-align: center; border-radius: 3px; font-size: 12px; font-weight: 500; color: #8b949e; border: 0; background: transparent; font-family: 'Space Grotesk', system-ui, sans-serif; cursor: pointer; }
.seg-btn.on { background: #21262d; color: #e6edf3; box-shadow: 0 1px 3px rgba(0,0,0,.4); }
.sb-input, .sb-select, .cfg-inp, .notes-area { width: 100%; background: #161b22; border: 1px solid #30363d; border-radius: 4px; color: #e6edf3; font-family: 'JetBrains Mono', monospace; font-size: 12px; padding: 5px 8px; outline: none; }
.cfg-inp { width: 132px; text-align: right; background: #1c2128; flex-shrink: 0; }
.sb-input:focus, .sb-select:focus, .cfg-inp:focus, .notes-area:focus { border-color: #2f81f7; }
.chk-row { display: flex; align-items: center; gap: 7px; font-size: 12px; color: #8b949e; cursor: pointer; }
.chk-row input { accent-color: #2f81f7; width: 14px; height: 14px; }
.tests-header { display: flex; align-items: flex-start; justify-content: space-between; margin-bottom: 14px; gap: 12px; }
.tests-header h2 { font-size: 15px; line-height: 1.2; font-weight: 600; margin: 0; color: #e6edf3; }
.tests-header p { font-size: 12px; color: #8b949e; margin: 2px 0 0; }
.t-card { display: flex; align-items: center; gap: 12px; padding: 11px 14px; background: #161b22; border: 1px solid #30363d; border-radius: 6px; margin-bottom: 4px; }
.t-card.running { border-color: #2f81f7; background: rgba(47,129,247,.14); }
.t-card.pass { border-color: #3fb950; }
.t-card.fail { border-color: #f85149; }
.t-card.warn { border-color: #d29922; }
.t-num { width: 28px; height: 28px; border-radius: 50%; flex-shrink: 0; background: #21262d; border: 1px solid #30363d; display: flex; align-items: center; justify-content: center; font-family: 'JetBrains Mono', monospace; font-size: 11px; color: #8b949e; }
.t-info { flex: 1; min-width: 0; }
.t-name { font-weight: 500; font-size: 13px; display: flex; align-items: center; gap: 7px; color: #e6edf3; }
.t-opt { font-size: 10px; font-weight: 600; letter-spacing: .06em; color: #484f58; }
.t-desc { font-size: 11px; color: #8b949e; margin-top: 1px; }
.t-res { font-family: 'JetBrains Mono', monospace; font-size: 11px; margin-top: 3px; color: #8b949e; white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
.t-sbadge { padding: 2px 8px; border-radius: 10px; font-size: 11px; font-weight: 500; white-space: nowrap; border: 1px solid #30363d; background: #21262d; color: #484f58; }
.t-sbadge.running { background: rgba(47,129,247,.14); color: #2f81f7; border-color: #2f81f7; }
.t-sbadge.pass { background: rgba(63,185,80,.12); color: #3fb950; border-color: #3fb950; }
.t-sbadge.fail { background: rgba(248,81,73,.12); color: #f85149; border-color: #f85149; }
.t-sbadge.warn { background: rgba(210,153,34,.12); color: #d29922; border-color: #d29922; }
.cfg-grid, .rep-grid { display: grid; grid-template-columns: repeat(2, minmax(280px, 1fr)); gap: 10px; }
.cfg-sec-title, .rep-sec-title, .card-title, .cc-title { margin-bottom: 8px; }
.cfg-row { margin-bottom: 8px; }
.notes-area { min-height: 96px; resize: vertical; background: #1c2128; }
.log-toolbar { display: flex; align-items: center; justify-content: space-between; margin-bottom: 10px; gap: 10px; flex-wrap: wrap; }
.log-filters, .log-toolbar-right { display: flex; align-items: center; gap: 4px; }
.log-flt { padding: 4px 10px; border-radius: 20px; border: 1px solid #30363d; background: #1c2128; color: #8b949e; font-family: 'JetBrains Mono', monospace; font-size: 11px; cursor: pointer; }
.log-flt.on { background: #21262d; color: #e6edf3; border-color: #8b949e; }
.log-feed { background: #080b10; border: 1px solid #30363d; border-radius: 6px; font-family: 'JetBrains Mono', monospace; font-size: 12px; line-height: 1.7; height: calc(100vh - 196px); overflow-y: auto; padding: 6px 0; }
.log-entry { display: flex; align-items: baseline; padding: 1px 12px; min-width: 0; }
.log-entry:hover { background: #161b22; }
.log-ts { color: #484f58; min-width: 82px; flex-shrink: 0; font-size: 11px; }
.log-tag { min-width: 54px; flex-shrink: 0; font-weight: 500; font-size: 11px; margin-right: 10px; }
.log-tag.tx { color: #2f81f7; }
.log-tag.rx { color: #3fb950; }
.log-tag.event { color: #d29922; }
.log-tag.fault { color: #f85149; }
.log-msg { color: #e6edf3; flex: 1; min-width: 0; word-break: break-word; }
.empty-note { color: #484f58; text-align: center; padding: 32px; font-family: 'JetBrains Mono', monospace; }
@media (max-width: 980px) {
  .hdr { gap: 8px; padding: 0 10px; }
  .hdr-group.device { display: none; }
  .hdr-status { max-width: 180px; }
  .metric-row { grid-template-columns: repeat(2, minmax(0, 1fr)); }
  .ov-grid { grid-template-columns: 1fr; }
  .cfg-grid, .rep-grid { grid-template-columns: 1fr; }
}
@media (max-width: 720px) {
  .body { flex-direction: column; }
  .sb { width: 100%; max-height: 42vh; border-right: 0; border-bottom: 1px solid #30363d; }
  .sb.collapsed { width: 100%; max-height: 40px; }
  .hdr-logo { font-size: 12px; }
  .hdr .hdr-sep, .hdr-group.mode, .hdr-status { display: none; }
  .btn.estop { padding: 6px 10px; }
  .tab-pane { padding: 10px; }
}
"""


def _btn(label: str, handler, disabled=False, variant: str = "", full: bool = False, **props) -> rx.Component:
    classes = "btn"
    if variant:
        classes += f" {variant}"
    if full:
        classes += " fw"
    return rx.button(
        label,
        on_click=handler,
        disabled=disabled,
        class_name=classes,
        **props,
    )


def _icon_btn(label: str, icon: str, handler, disabled=False, variant: str = "", full: bool = False) -> rx.Component:
    return rx.button(
        rx.icon(icon, size=14),
        rx.text(label, as_="span"),
        on_click=handler,
        disabled=disabled,
        class_name=f"btn {variant}{' fw' if full else ''}".strip(),
    )


def _jog_btn(label: str, handler, disabled=False) -> rx.Component:
    return rx.button(label, on_click=handler, disabled=disabled, class_name="jog-btn")


def _badge(value, color: str = "") -> rx.Component:
    return rx.box(value, class_name=f"badge {color}".strip())


def _conn_pill() -> rx.Component:
    return rx.cond(
        State.connected,
        rx.box(rx.box(class_name="pill-dot"), rx.text("Connected"), class_name="conn-pill live"),
        rx.box(rx.box(class_name="pill-dot"), rx.text("Disconnected"), class_name="conn-pill"),
    )


def _mode_badge() -> rx.Component:
    return rx.cond(
        State.mode_name == "FAULT",
        _badge(State.mode_display, "red"),
        rx.cond(
            State.mode_name == "CALIBRATION",
            _badge(State.mode_display, "acc"),
            _badge(State.mode_display),
        ),
    )


def _fault_badge() -> rx.Component:
    return rx.cond(
        (State.fault_text == "NONE") | (State.fault_text == ""),
        _badge(State.fault_display, "green"),
        _badge(State.fault_display, "red"),
    )


def _seg_button(label: str, active, handler) -> rx.Component:
    return rx.button(
        label,
        on_click=handler,
        class_name=rx.cond(active, "seg-btn on", "seg-btn"),
    )


def _field_label(text: str) -> rx.Component:
    return rx.text(text, class_name="lbl-sm")


def _section_label(text: str) -> rx.Component:
    return rx.text(text, class_name="sb-lbl")


def _metric(label: str, value, unit: str, highlight: bool = False, red: bool = False) -> rx.Component:
    cls = "mc"
    if highlight:
        cls += " hl"
    if red:
        cls += " red"
    return rx.box(
        rx.text(label, class_name="mc-lbl"),
        rx.text(value, class_name="mc-val"),
        rx.text(unit, class_name="mc-unit"),
        class_name=cls,
    )


def _result_row(label: str, value) -> rx.Component:
    return rx.hstack(rx.text(label, class_name="res-k"), rx.text(value, class_name="res-v"), class_name="res-row")


def _report_row(label: str, value) -> rx.Component:
    return rx.hstack(rx.text(label, class_name="rep-k"), rx.text(value, class_name="rep-v"), class_name="rep-row")


def _config_input(label: str, value, setter, disabled=False) -> rx.Component:
    return rx.hstack(
        rx.text(label, class_name="cfg-k"),
        rx.input(value=value, on_change=setter, disabled=disabled, class_name="cfg-inp"),
        class_name="cfg-row",
    )


def _config_bool(label: str, enabled, on_handler, off_handler) -> rx.Component:
    return rx.hstack(
        rx.text(label, class_name="cfg-k"),
        rx.box(
            _seg_button("OFF", ~enabled, off_handler),
            _seg_button("ON", enabled, on_handler),
            class_name="seg",
            width="120px",
        ),
        class_name="cfg-row",
    )


def header_panel() -> rx.Component:
    return rx.box(
        rx.hstack(
            rx.hstack(rx.icon("settings", size=22), rx.text("Actuator Bench"), class_name="hdr-logo"),
            rx.box(class_name="hdr-sep"),
            rx.hstack(
                _conn_pill(),
                _badge(rx.cond(State.actuator_id != "", State.actuator_id, "-"), "acc"),
                _badge(rx.cond(State.firmware != "", State.firmware, "-"), "green"),
                _badge(rx.cond(State.hardware != "", State.hardware, "-")),
                class_name="hdr-group device",
            ),
            rx.box(class_name="hdr-sep"),
            rx.hstack(
                _mode_badge(),
                _fault_badge(),
                class_name="hdr-group mode",
            ),
            rx.box(class_name="hdr-spacer"),
            rx.text(State.status, class_name="hdr-status"),
            _icon_btn("ESTOP", "square", State.do_estop, disabled=~State.can_control, variant="estop"),
            class_name="hdr",
            width="100%",
        )
    )


def connection_section() -> rx.Component:
    return rx.box(
        _section_label("Connection"),
        rx.hstack(
            rx.checkbox("Simulator", checked=State.use_sim, on_change=State.set_use_sim, class_name="chk-row"),
            _btn("Scan", State.scan_ports),
            class_name="sb-row",
        ),
        rx.select(
            State.available_ports,
            value=State.port,
            on_change=State.set_port,
            placeholder="Select serial port...",
            class_name="sb-select mt6",
        ),
        rx.hstack(
            _btn("Connect", State.do_connect, disabled=State.connected | State.busy, variant="pr", full=True),
            _btn("Disconnect", State.do_disconnect, disabled=~State.connected | State.busy, full=True),
            class_name="sb-row mt8",
        ),
        class_name="sb-sec",
    )


def mode_section() -> rx.Component:
    return rx.box(
        _section_label("Mode"),
        rx.box(
            _seg_button("DISABLED", State.mode_name != "CALIBRATION", State.mode_disabled),
            _seg_button("CALIBRATION", State.mode_name == "CALIBRATION", State.mode_calibration),
            class_name="seg",
        ),
        class_name="sb-sec",
    )


def jog_section() -> rx.Component:
    return rx.box(
        _section_label("Jog"),
        rx.hstack(
            _jog_btn("-", State.jog_neg, disabled=~State.can_test),
            rx.box(
                _field_label("Step (rad)"),
                rx.input(value=State.jog_step.to_string(), on_change=State.set_jog_step, class_name="sb-input"),
                flex="1",
            ),
            _jog_btn("+", State.jog_pos, disabled=~State.can_test),
            class_name="sb-row",
        ),
        rx.text("Move Relative", class_name="sb-lbl mt8"),
        _field_label("Delta (rad)"),
        rx.input(value=State.move_delta.to_string(), on_change=State.set_move_delta, class_name="sb-input"),
        rx.box(_field_label("Velocity (rad/s)"), class_name="mt6"),
        rx.input(value=State.move_velocity.to_string(), on_change=State.set_move_velocity, class_name="sb-input"),
        rx.box(_field_label("Accel (rad/s2)"), class_name="mt6"),
        rx.input(value=State.move_accel.to_string(), on_change=State.set_move_accel, class_name="sb-input"),
        rx.box(_btn("Move Relative", State.do_move_rel, disabled=~State.can_test, full=True), class_name="mt8"),
        class_name="sb-sec",
    )


def encoder_section() -> rx.Component:
    return rx.box(
        _section_label("Encoders"),
        rx.hstack(
            _btn("Zero Motor", State.zero_motor, disabled=~State.can_test, full=True),
            _btn("Zero Output", State.zero_output, disabled=~State.can_test, full=True),
            class_name="sb-row",
        ),
        class_name="sb-sec",
    )


def safety_section() -> rx.Component:
    return rx.box(
        _section_label("Safety"),
        _btn("Stop", State.do_stop, disabled=~State.can_control, full=True, variant="am"),
        class_name="sb-sec",
    )


def sidebar() -> rx.Component:
    return rx.box(
        rx.button(
            rx.text("Controls", class_name="sb-toggle-label"),
            rx.box(rx.icon("chevron-left", size=12), class_name="sb-toggle-icon"),
            on_click=State.toggle_sidebar,
            class_name="sb-toggle",
        ),
        connection_section(),
        mode_section(),
        jog_section(),
        encoder_section(),
        safety_section(),
        class_name=rx.cond(State.sidebar_collapsed, "sb collapsed", "sb"),
    )


def _tab_button(key: str, label: str) -> rx.Component:
    return rx.button(
        label,
        on_click=lambda: State.set_active_tab(key),
        class_name=rx.cond(State.active_tab == key, "tab on", "tab"),
    )


def tab_bar() -> rx.Component:
    return rx.hstack(
        _tab_button("overview", "Overview"),
        _tab_button("tests", "Tests"),
        _tab_button("config", "Config"),
        _tab_button("report", "Report"),
        _tab_button("log", "Log"),
        class_name="tab-bar",
    )


def actuator_diagram() -> rx.Component:
    return rx.box(
        rx.text("Actuator Diagram", class_name="card-title"),
        rx.el.svg(
            rx.el.line(x1="79", y1="53", x2="219", y2="46", stroke="#30363d", stroke_width="3"),
            rx.el.line(x1="79", y1="117", x2="219", y2="124", stroke="#30363d", stroke_width="3"),
            rx.el.circle(cx="79", cy="85", r="34", fill="#1c2128", stroke="#30363d", stroke_width="1.5"),
            rx.el.line(x1="79", y1="85", x2=State.diag_motor_x, y2=State.diag_motor_y, stroke="#2f81f7", stroke_width="3", stroke_linecap="round"),
            rx.el.circle(cx="79", cy="85", r="5", fill="#2f81f7"),
            rx.el.text("Motor", x="79", y="133", text_anchor="middle", fill="#8b949e", font_size="10"),
            rx.el.text(State.diag_motor_label, x="79", y="145", text_anchor="middle", fill="#484f58", font_size="9", font_family="JetBrains Mono"),
            rx.el.circle(cx="224", cy="85", r="46", fill="#1c2128", stroke="#30363d", stroke_width="1.5"),
            rx.el.line(x1="224", y1="85", x2=State.diag_output_x, y2=State.diag_output_y, stroke="#d29922", stroke_width="3", stroke_linecap="round"),
            rx.el.circle(cx="224", cy="85", r="5", fill="#d29922"),
            rx.el.text("Output", x="224", y="147", text_anchor="middle", fill="#8b949e", font_size="10"),
            rx.el.text(State.diag_output_label, x="224", y="159", text_anchor="middle", fill="#484f58", font_size="9", font_family="JetBrains Mono"),
            rx.el.text(State.diag_deflection_label, x="150", y="16", text_anchor="middle", fill="#f85149", font_size="9", font_family="JetBrains Mono"),
            rx.el.line(x1="150", y1="19", x2="150", y2="40", stroke="#f85149", stroke_width="1", stroke_dasharray="2 2", opacity=".35"),
            viewBox="0 0 300 170",
            width="100%",
        ),
        class_name="diagram-card",
    )


def overview_tab() -> rx.Component:
    return rx.box(
        rx.box(
            _metric("Motor Angle", State.motor_angle_str, "rad"),
            _metric("Output Angle", State.output_angle_str, "rad"),
            _metric("Deflection", State.deflection_str, "rad", highlight=True),
            _metric("Motor Velocity", State.motor_velocity_str, "rad/s"),
            _metric("Bus Voltage", State.bus_voltage_str, "V"),
            _metric("Temperature", State.temperature_str, "C"),
            class_name="metric-row",
        ),
        rx.box(
            rx.box(
                rx.box(
                    rx.text("Live Telemetry", class_name="cc-title"),
                    rx.el.iframe(src=f"http://localhost:{BOKEH_PORT}", class_name="telemetry-frame"),
                    class_name="cc",
                ),
                class_name="chart-stack",
            ),
            rx.box(
                actuator_diagram(),
                rx.box(
                    rx.text("Calibration Results", class_name="card-title"),
                    _result_row("Ratio", State.ratio_str),
                    _result_row("Offset", State.offset_str),
                    _result_row("Fit RMS", State.fit_error_str),
                    _result_row("Hysteresis", State.hysteresis_str),
                    _result_row("Resonance", State.resonance_str),
                    _result_row("Backlash", State.backlash_str),
                    _result_row("Samples", State.sample_count),
                    _result_row("Dropped", State.dropped),
                    class_name="results-card",
                ),
                class_name="side-col",
            ),
            class_name="ov-grid",
        ),
        class_name="tab-pane",
    )


def _state_label(state) -> rx.Component:
    return rx.match(
        state,
        ("running", "Running"),
        ("pass", "Pass"),
        ("fail", "Fail"),
        ("warn", "Warning"),
        "Not run",
    )


def _test_card(num: int, label: str, desc: str, required: bool, state, result, handler) -> rx.Component:
    return rx.box(
        rx.box(str(num), class_name="t-num"),
        rx.box(
            rx.hstack(
                rx.text(label, as_="span"),
                rx.cond(required, rx.fragment(), rx.text("OPTIONAL", class_name="t-opt")),
                class_name="t-name",
            ),
            rx.text(desc, class_name="t-desc"),
            rx.cond(result != "", rx.text(result, class_name="t-res"), rx.fragment()),
            class_name="t-info",
        ),
        rx.box(_state_label(state), class_name=rx.cond(state == "idle", "t-sbadge", "t-sbadge " + state)),
        rx.button(
            "Run",
            on_click=handler,
            disabled=State.busy | ~State.connected,
            class_name=rx.cond(state == "fail", "btn dn", rx.cond(state == "pass", "btn gn", "btn")),
        ),
        class_name=rx.cond(state == "idle", "t-card", "t-card " + state),
    )


def tests_tab() -> rx.Component:
    return rx.box(
        rx.box(
            rx.box(
                rx.heading("Calibration Pipeline", as_="h2"),
                rx.text("Run tests in sequence. Required tests must pass before saving to flash."),
            ),
            _icon_btn("Auto-Characterize", "zap", State.run_auto, disabled=~State.can_test, variant="pr"),
            class_name="tests-header",
        ),
        _test_card(1, "Actuator Detection", "PING/PONG, read INFO block", True, State.test_detection_state, State.test_detection_result, State.run_detection),
        _test_card(2, "Encoder Sanity", "Verify motor and output encoder polarity", True, State.test_encoder_state, State.test_encoder_result, State.run_encoder_sanity),
        _test_card(3, "Ratio Calibration", "Bi-directional sweep, fit output/motor ratio", True, State.test_ratio_state, State.test_ratio_result, State.run_ratio),
        _test_card(4, "Resonance Chirp", "Frequency sweep, find peak Hz", True, State.test_resonance_state, State.test_resonance_result, State.run_resonance),
        _test_card(5, "Backlash Test", "Reversal deadband measurement", False, State.test_backlash_state, State.test_backlash_result, State.run_backlash),
        _test_card(6, "Step Response", "Settling time and overshoot characterisation", False, State.test_step_state, State.test_step_result, State.run_step_response),
        _test_card(7, "Velocity Ramp", "Safe velocity and acceleration limits", False, State.test_velocity_state, State.test_velocity_result, State.run_velocity_ramp),
        _test_card(8, "Compliance / Load", "Output stiffness in N*m/rad", False, State.test_compliance_state, State.test_compliance_result, State.run_compliance),
        _test_card(9, "Save to Actuator Flash", "Write fitted calibration constants to flash", True, State.test_save_state, State.test_save_result, State.save_config),
        class_name="tab-pane",
    )


def config_tab() -> rx.Component:
    return rx.box(
        rx.box(
            rx.box(
                rx.text("Safety Limits", class_name="cfg-sec-title"),
                _config_input("Max velocity (rad/s)", State.cfg_max_velocity, State.set_cfg_max_velocity),
                _config_input("Max accel (rad/s2)", State.cfg_max_accel, State.set_cfg_max_accel),
                _config_input("Jog step (rad)", State.cfg_jog_step, State.set_cfg_jog_step),
                _config_input("Cal. velocity (rad/s)", State.cfg_cal_velocity, State.set_cfg_cal_velocity),
                _config_input("Cal. accel (rad/s2)", State.cfg_cal_accel, State.set_cfg_cal_accel),
                _config_input("Max move delta (rad)", State.cfg_max_move, State.set_cfg_max_move),
                class_name="cfg-sec",
            ),
            rx.box(
                rx.text("PID Parameters", class_name="cfg-sec-title"),
                _config_bool("PID enabled", State.cfg_pid_enabled, State.enable_pid, State.disable_pid),
                _config_input("Kp", State.cfg_pid_kp, State.set_cfg_pid_kp),
                _config_input("Ki", State.cfg_pid_ki, State.set_cfg_pid_ki),
                _config_input("Kd", State.cfg_pid_kd, State.set_cfg_pid_kd),
                _config_input("I-limit (motor rad)", State.cfg_pid_i_limit, State.set_cfg_pid_i_limit),
                _config_input("Output limit (motor rad)", State.cfg_pid_output_limit, State.set_cfg_pid_output_limit),
                class_name="cfg-sec",
            ),
            rx.box(
                rx.text("Transmission", class_name="cfg-sec-title"),
                _config_input("Output per motor", State.cfg_output_per_motor, State.set_cfg_output_per_motor),
                _config_input("Output offset (rad)", State.cfg_output_offset, State.set_cfg_output_offset),
                _config_input("Resonance freq (Hz)", State.cfg_resonance_frequency, State.set_cfg_resonance_frequency),
                _config_input("Backlash motor (rad)", State.cfg_backlash_motor, State.set_cfg_backlash_motor),
                _config_bool("Backlash comp", State.cfg_backlash_comp_enabled, State.enable_backlash_comp, State.disable_backlash_comp),
                _config_bool("Resonance derating", State.cfg_resonance_derating_enabled, State.enable_resonance_derating, State.disable_resonance_derating),
                rx.hstack(
                    _btn("Save to Actuator", State.save_ui_config, disabled=State.busy | ~State.connected, variant="pr"),
                    _btn("Read / Revert", State.reload_config, disabled=State.busy),
                    class_name="sb-row mt8",
                ),
                class_name="cfg-sec",
            ),
            rx.box(
                rx.text("Encoder Signs", class_name="cfg-sec-title"),
                rx.hstack(rx.text("Motor encoder", class_name="cfg-k"), _badge("read-only", "amber"), _badge("+/- from sanity test"), class_name="cfg-row"),
                rx.hstack(rx.text("Output encoder", class_name="cfg-k"), _badge("read-only", "amber"), _badge("+/- from sanity test"), class_name="cfg-row"),
                rx.text("Encoder Zero", class_name="cfg-sec-title", margin_top="16px"),
                rx.hstack(
                    _btn("Zero Motor", State.zero_motor, disabled=~State.can_test),
                    _btn("Zero Output", State.zero_output, disabled=~State.can_test),
                    class_name="sb-row",
                ),
                class_name="cfg-sec",
            ),
            class_name="cfg-grid",
        ),
        class_name="tab-pane",
    )


def report_tab() -> rx.Component:
    return rx.box(
        rx.box(
            rx.box(
                rx.text("Session", class_name="rep-sec-title"),
                _report_row("Actuator ID", rx.cond(State.actuator_id != "", State.actuator_id, "-")),
                _report_row("Firmware", rx.cond(State.firmware != "", State.firmware, "-")),
                _report_row("Hardware", rx.cond(State.hardware != "", State.hardware, "-")),
                _report_row("Mode", State.mode_display),
                _report_row("Total samples", State.sample_count),
                _report_row("Dropped samples", State.dropped),
                class_name="rep-sec",
            ),
            rx.box(
                rx.text("Output Files", class_name="rep-sec-title"),
                _report_row("Folder", rx.cond(State.report_folder != "", State.report_folder, "-")),
                _report_row("Raw CSV", "raw_stream.csv"),
                _report_row("Fitted JSON", "fitted_params.json"),
                _report_row("Ratio plot", "ratio_plot.png"),
                _report_row("Resonance plot", "resonance_plot.png"),
                _report_row("Summary", "summary.txt"),
                rx.hstack(
                    _btn("Export Report", State.export_report, disabled=~State.connected, variant="pr"),
                    _btn("Open Folder", State.open_folder, disabled=State.report_folder == ""),
                    class_name="sb-row mt8",
                ),
                class_name="rep-sec",
            ),
            rx.box(
                rx.text("Session Notes", class_name="rep-sec-title"),
                rx.text_area(
                    value=State.notes,
                    on_change=State.set_notes,
                    placeholder="Add calibration notes, observations, or anomalies for the report...",
                    class_name="notes-area",
                ),
                rx.hstack(
                    rx.text("Saved with report on export.", color="#484f58", font_size="11px"),
                    rx.box(flex="1"),
                    _btn("Save Notes", State.export_report, disabled=~State.connected),
                    class_name="sb-row mt8",
                ),
                class_name="rep-sec",
                grid_column="span 2",
            ),
            class_name="rep-grid",
        ),
        class_name="tab-pane",
    )


def _log_filter(label: str, value: str) -> rx.Component:
    return rx.button(
        label,
        on_click=lambda: State.set_log_filter(value),
        class_name=rx.cond(State.log_filter == value, "log-flt on", "log-flt"),
    )


def _log_row(entry) -> rx.Component:
    return rx.hstack(
        rx.text(entry["time"], class_name="log-ts"),
        rx.text(entry["tag"], class_name=entry["class_name"]),
        rx.text(entry["message"], class_name="log-msg"),
        class_name="log-entry",
    )


def log_tab() -> rx.Component:
    return rx.box(
        rx.hstack(
            rx.hstack(
                _log_filter("ALL", "all"),
                _log_filter("TX", "tx"),
                _log_filter("RX", "rx"),
                _log_filter("EVENT", "event"),
                _log_filter("FAULT", "fault"),
                class_name="log-filters",
            ),
            rx.hstack(
                rx.checkbox("Auto-scroll", checked=State.log_auto_scroll, on_change=State.set_log_auto_scroll, class_name="chk-row"),
                _btn("Clear", State.clear_log),
                class_name="log-toolbar-right",
            ),
            class_name="log-toolbar",
        ),
        rx.box(
            rx.foreach(State.visible_log_entries, _log_row),
            class_name="log-feed",
            id="logFeed",
        ),
        class_name="tab-pane",
    )


def tab_content() -> rx.Component:
    return rx.box(
        rx.cond(State.active_tab == "overview", overview_tab(), rx.fragment()),
        rx.cond(State.active_tab == "tests", tests_tab(), rx.fragment()),
        rx.cond(State.active_tab == "config", config_tab(), rx.fragment()),
        rx.cond(State.active_tab == "report", report_tab(), rx.fragment()),
        rx.cond(State.active_tab == "log", log_tab(), rx.fragment()),
        class_name="tab-wrap",
    )


def index() -> rx.Component:
    return rx.fragment(
        rx.el.style(DESIGN_CSS),
        rx.box(
            header_panel(),
            rx.box(
                sidebar(),
                rx.box(
                    tab_bar(),
                    tab_content(),
                    class_name="main",
                ),
                class_name="body",
            ),
            class_name="app-shell",
        ),
    )


# Boot

def _start_bokeh() -> None:
    global _bokeh_started
    if _bokeh_started:
        return
    _bokeh_started = True
    launch_bokeh_thread(_ctx)


app = rx.App(
    stylesheets=[
        "https://fonts.googleapis.com/css2?family=Space+Grotesk:wght@400;500;600;700&family=JetBrains+Mono:wght@400;500&display=swap"
    ],
)
app.add_page(
    index,
    route="/",
    title="Actuator Bench Tool",
    on_load=[State.scan_ports, State.start_polling],
)
