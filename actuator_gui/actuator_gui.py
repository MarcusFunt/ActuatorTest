"""Reflex web frontend for the actuator bench tool."""

from __future__ import annotations

import asyncio
import os
import subprocess
import threading
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field

import reflex as rx

from actuator_tool.actuator_analysis import RatioFitResult, ResonanceResult
from actuator_tool.actuator_data import ActuatorInfo, TelemetryStore
from actuator_tool.actuator_protocol import ActuatorMode, FaultFlags
from actuator_tool.actuator_report import (
    SessionReport,
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


# ── Backend context (non-serialisable; lives outside Reflex state) ────────────

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
    _status: str = "Disconnected"
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


_ctx = BackendCtx()
_executor = ThreadPoolExecutor(max_workers=2, thread_name_prefix="actuator")

# One flag so hot-reload doesn't start duplicate pollers / Bokeh servers.
_polling_started = False
_bokeh_started = False


def _env_bool(name: str, default: bool) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


_DEFAULT_USE_SIM = _env_bool("ACTUATOR_GUI_USE_SIM", True)
_DEFAULT_PORT = os.environ.get("ACTUATOR_GUI_PORT", "")


# ── Thread-pool work functions ────────────────────────────────────────────────

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
        c.calibration.output_per_motor = float(cfg.get("output_per_motor", 1.0))
        c.calibration.motor_per_output = 1.0 / c.calibration.output_per_motor
        c.calibration.output_offset_rad = float(cfg.get("output_offset_rad", 0.0))
        c.calibration.resonance_frequency_hz = cfg.get("resonance_frequency_hz")
        c.calibration.pid_enabled = bool(cfg.get("pid_enabled", False))
        c.calibration.pid_kp = float(cfg.get("pid_kp", 0.0))
        c.calibration.pid_ki = float(cfg.get("pid_ki", 0.0))
        c.calibration.pid_kd = float(cfg.get("pid_kd", 0.0))
        c.calibration.pid_i_limit_motor_rad = float(cfg.get("pid_i_limit_motor_rad", 0.05))
        c.calibration.pid_output_limit_motor_rad = float(cfg.get("pid_output_limit_motor_rad", 0.25))
        c.calibration.backlash_motor_rad = float(cfg.get("backlash_motor_rad", 0.0))
        c.calibration.backlash_comp_enabled = bool(cfg.get("backlash_comp_enabled", False))
        c.calibration.resonance_derating_enabled = bool(cfg.get("resonance_derating_enabled", False))
    except Exception:
        pass
    client.start_stream(timeout=0.5)
    report = SessionReport()
    report.save_actuator_info(info)
    report.events.write("connect", {"simulator": use_sim, "port": port})
    client.add_telemetry_callback(report.recorder.record_sample)
    c.client = client
    c.info = info
    c.report = report
    c.connected = True
    c.set_status(f"Connected to {info.actuator_id}")


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


def _run_detection() -> None:
    c = _ctx
    result = run_detection(c.require_client(), c.set_status)
    if result.info is not None:
        c.info = result.info
    if c.report:
        c.report.events.write("detection", result.as_dict())
    c.set_status(result.message)


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


def _save_config() -> None:
    c = _ctx
    client = c.require_client()
    cal = c.calibration
    for k, v in [
        ("output_per_motor",           cal.output_per_motor),
        ("output_offset_rad",          cal.output_offset_rad),
        ("pid_enabled",                cal.pid_enabled),
        ("pid_kp",                     cal.pid_kp),
        ("pid_ki",                     cal.pid_ki),
        ("pid_kd",                     cal.pid_kd),
        ("pid_i_limit_motor_rad",      cal.pid_i_limit_motor_rad),
        ("pid_output_limit_motor_rad", cal.pid_output_limit_motor_rad),
        ("backlash_motor_rad",         cal.backlash_motor_rad),
        ("backlash_comp_enabled",      cal.backlash_comp_enabled),
        ("resonance_frequency_hz",     cal.resonance_frequency_hz),
        ("resonance_derating_enabled", cal.resonance_derating_enabled),
    ]:
        client.set_config(k, v)
    client.save_config()
    if c.report:
        c.report.events.write("save_config_to_actuator", cal.as_dict())
    c.set_status("Calibration saved to actuator")


# ── Reflex state ──────────────────────────────────────────────────────────────

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

    # Calibration / results
    ratio_str: str = "—"
    offset_str: str = "—"
    fit_error_str: str = "—"
    hysteresis_str: str = "—"
    resonance_str: str = "—"
    sample_count: int = 0
    dropped: int = 0
    warnings_text: str = "none"

    # Motion params
    jog_step: float = 0.25
    move_delta: float = 1.0
    move_velocity: float = 1.2
    move_accel: float = 10.0

    # Report
    report_folder: str = ""
    notes: str = ""

    # UI flags
    busy: bool = False

    # ── Computed vars ──────────────────────────────────────────────────────

    @rx.var
    def can_control(self) -> bool:
        return self.connected and not self.busy

    @rx.var
    def can_test(self) -> bool:
        return self.connected and self.mode_name == "CALIBRATION" and not self.busy

    # ── Simple setters (auto-generated for primitives; explicit for floats) ─

    def set_use_sim(self, v: bool) -> None:
        self.use_sim = v

    def set_port(self, v: str) -> None:
        self.port = v

    def set_notes(self, v: str) -> None:
        self.notes = v

    def set_jog_step(self, v: str) -> None:
        try:
            self.jog_step = float(v)
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

    # ── Port scan ─────────────────────────────────────────────────────────

    def scan_ports(self) -> None:
        self.available_ports = scan_serial_ports()
        self.status = f"Found {len(self.available_ports)} port(s)"

    # ── Connection ────────────────────────────────────────────────────────

    @rx.event(background=True)
    async def do_connect(self) -> None:
        async with self:
            self.busy = True
            self.status = "Connecting…"
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
        except Exception as exc:
            async with self:
                self.status = f"Connect failed: {exc}"
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
        except Exception as exc:
            async with self:
                self.status = f"Disconnect error: {exc}"
        finally:
            async with self:
                self.busy = False

    # ── Mode / motion ─────────────────────────────────────────────────────

    def mode_disabled(self) -> None:
        try:
            _ctx.require_client().set_mode(ActuatorMode.DISABLED)
            self.status = "Mode: DISABLED"
        except Exception as exc:
            self.status = str(exc)

    def mode_calibration(self) -> None:
        try:
            _ctx.require_client().set_mode(ActuatorMode.CALIBRATION)
            self.status = "Mode: CALIBRATION"
        except Exception as exc:
            self.status = str(exc)

    def do_stop(self) -> None:
        try:
            _ctx.require_client().stop()
            self.status = "Stop sent"
        except Exception as exc:
            self.status = str(exc)

    def do_estop(self) -> None:
        try:
            _ctx.require_client().estop()
            self.status = "ESTOP sent"
        except Exception as exc:
            self.status = str(exc)

    def jog_pos(self) -> None:
        self._jog(1.0)

    def jog_neg(self) -> None:
        self._jog(-1.0)

    def _jog(self, sign: float) -> None:
        latest = _ctx.store.latest()
        if latest is None or latest.mode != int(ActuatorMode.CALIBRATION):
            self.status = "Jog requires CALIBRATION mode"
            return
        try:
            _ctx.require_client().move_rel(sign * self.jog_step, self.move_velocity, self.move_accel)
        except Exception as exc:
            self.status = str(exc)

    @rx.event(background=True)
    async def do_move_rel(self) -> None:
        delta, vel, accel = self.move_delta, self.move_velocity, self.move_accel
        loop = asyncio.get_event_loop()
        try:
            await loop.run_in_executor(
                _executor,
                lambda: _ctx.require_client().move_rel(delta, vel, accel),
            )
            async with self:
                self.status = f"MOVE_REL {delta:.3f} rad"
        except Exception as exc:
            async with self:
                self.status = str(exc)

    def zero_motor(self) -> None:
        try:
            _ctx.require_client().zero_motor_encoder()
            self.status = "Motor encoder zeroed"
        except Exception as exc:
            self.status = str(exc)

    def zero_output(self) -> None:
        try:
            _ctx.require_client().zero_output_encoder()
            self.status = "Output encoder zeroed"
        except Exception as exc:
            self.status = str(exc)

    # ── Tests (all background) ────────────────────────────────────────────

    @rx.event(background=True)
    async def run_detection(self) -> None:
        await self._worker("detection", _run_detection)

    @rx.event(background=True)
    async def run_encoder_sanity(self) -> None:
        await self._worker("encoder sanity", _run_encoder_sanity)

    @rx.event(background=True)
    async def run_ratio(self) -> None:
        await self._worker("ratio calibration", _run_ratio)

    @rx.event(background=True)
    async def run_resonance(self) -> None:
        await self._worker("resonance test", _run_resonance)

    @rx.event(background=True)
    async def run_backlash(self) -> None:
        await self._worker("backlash test", _run_backlash)

    @rx.event(background=True)
    async def run_step_response(self) -> None:
        await self._worker("step response", _run_step_response)

    @rx.event(background=True)
    async def run_velocity_ramp(self) -> None:
        await self._worker("velocity ramp", _run_velocity_ramp)

    @rx.event(background=True)
    async def run_compliance(self) -> None:
        await self._worker("compliance test", _run_compliance)

    @rx.event(background=True)
    async def run_auto(self) -> None:
        await self._worker("auto-characterization", _run_auto)

    @rx.event(background=True)
    async def save_config(self) -> None:
        await self._worker("save config", _save_config)

    async def _worker(self, label: str, fn) -> None:
        async with self:
            self.busy = True
            self.status = f"{label} started…"
        loop = asyncio.get_event_loop()
        try:
            await loop.run_in_executor(_executor, fn)
            async with self:
                self.status = _ctx.get_status()
                self._sync_results()
        except Exception as exc:
            async with self:
                self.status = f"{label} failed: {exc}"
        finally:
            async with self:
                self.busy = False

    def _sync_results(self) -> None:
        """Pull calibration/stats from _ctx into serialisable state vars."""
        cal = _ctx.calibration
        stats = _ctx.store.stats
        self.ratio_str      = f"{cal.output_per_motor:.8f}"
        self.offset_str     = f"{cal.output_offset_rad:.8f} rad"
        self.hysteresis_str = f"{cal.hysteresis_rad:.8f} rad"
        self.sample_count   = stats.total_samples
        self.dropped        = stats.dropped_samples
        if _ctx.ratio_fit is not None:
            self.fit_error_str = f"{_ctx.ratio_fit.residual_rms_rad:.8f} rad RMS"
        res = _ctx.resonance_result
        if res is not None and res.peak_frequency_hz is not None:
            self.resonance_str = f"{res.peak_frequency_hz:.2f} Hz"
        elif cal.resonance_frequency_hz is not None:
            self.resonance_str = f"{cal.resonance_frequency_hz:.2f} Hz"
        if _ctx.warnings:
            self.warnings_text = "\n".join(_ctx.warnings[-6:])
        if _ctx.report is not None:
            self.report_folder = str(_ctx.report.artifacts.folder)

    # ── Report ────────────────────────────────────────────────────────────

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
        self._sync_results()

    def open_folder(self) -> None:
        if _ctx.report is None:
            self.status = "No active report session"
            return
        subprocess.Popen(["explorer", str(_ctx.report.artifacts.folder.resolve())])

    # ── Polling loop ──────────────────────────────────────────────────────

    @rx.event(background=True)
    async def start_polling(self) -> None:
        global _polling_started
        if _polling_started:
            return
        _polling_started = True

        while True:
            await asyncio.sleep(0.5)
            latest = _ctx.store.latest()
            stats = _ctx.store.stats
            async with self:
                self.status = _ctx.get_status()
                if latest is not None:
                    self.mode_name = latest.mode_name
                    faults = FaultFlags(latest.fault_flags)
                    self.fault_text = "NONE" if faults == FaultFlags.NONE else str(faults)
                self.sample_count = stats.total_samples
                self.dropped = stats.dropped_samples


# ── UI helpers ────────────────────────────────────────────────────────────────

def _lbl(text: str) -> rx.Component:
    return rx.text(text, color=rx.color("gray", 11), size="1", weight="medium")


def _val(v) -> rx.Component:
    return rx.text(v, size="2", font_family="monospace", color=rx.color("gray", 12))


def _sec(title: str) -> rx.Component:
    return rx.hstack(
        rx.text(title, weight="bold", size="2", color=rx.color("gray", 12)),
        rx.divider(orientation="horizontal", flex="1"),
        align="center",
        width="100%",
        padding_y="4px",
    )


def _btn(
    label: str,
    handler,
    disabled=False,
    color: str = "blue",
    size: str = "1",
    **extra_props,
) -> rx.Component:
    props = {
        "min_height": "30px",
    }
    props.update(extra_props)
    return rx.button(
        label,
        on_click=handler,
        disabled=disabled,
        color_scheme=color,
        size=size,
        **props,
    )


def _empty(v) -> rx.Component:
    return rx.cond(v != "", _val(v), rx.text("not connected", size="1", color=rx.color("gray", 9)))


def _panel(
    *children,
    flex: str | None = None,
    min_width: str = "0",
    **extra_props,
) -> rx.Component:
    props = {
        "padding": "14px",
        "border": f"1px solid {rx.color('gray', 6)}",
        "border_radius": "8px",
        "background": rx.color("gray", 2),
        "width": "100%",
        "min_width": min_width,
    }
    if flex is not None:
        props["flex"] = flex
    props.update(extra_props)
    return rx.box(
        rx.vstack(
            *children,
            spacing="3",
            align="start",
            width="100%",
        ),
        **props,
    )


def _chip(label: str, value, color: str = "gray") -> rx.Component:
    return rx.box(
        rx.hstack(
            rx.text(label, size="1", color=rx.color("gray", 10)),
            rx.text(value, size="1", weight="medium", color=rx.color("gray", 12)),
            spacing="2",
            align="center",
        ),
        padding_x="10px",
        height="30px",
        border=f"1px solid {rx.color(color, 6)}",
        border_radius="999px",
        background=rx.color(color, 3),
        display="flex",
        align_items="center",
    )


def header_panel() -> rx.Component:
    return rx.box(
        rx.vstack(
            rx.vstack(
                rx.text("Actuator Bench Tool", size="5", weight="bold", line_height="1.1"),
                rx.text(
                    "Calibration, characterization, and report capture",
                    size="2",
                    color=rx.color("gray", 10),
                ),
                spacing="1",
                align="start",
            ),
            rx.hstack(
                rx.cond(
                    State.connected,
                    _chip("State", "Connected", "green"),
                    _chip("State", "Disconnected", "gray"),
                ),
                _chip("Mode", rx.cond(State.mode_name != "", State.mode_name, "Idle")),
                _chip("Samples", State.sample_count),
                spacing="2",
                wrap="wrap",
                justify="start",
            ),
            width="100%",
            align="start",
            spacing="3",
        ),
        padding="16px",
        border=f"1px solid {rx.color('gray', 6)}",
        border_radius="8px",
        background=rx.color("gray", 2),
        width="100%",
    )


# ── Panels ────────────────────────────────────────────────────────────────────

def connection_panel() -> rx.Component:
    return _panel(
        _sec("Connection"),
        rx.hstack(
            rx.checkbox("Simulator", checked=State.use_sim, on_change=State.set_use_sim),
            _btn("Scan", State.scan_ports, color="gray", width="92px"),
            align="center",
            justify="between",
            width="100%",
        ),
        rx.select(
            State.available_ports,
            value=State.port,
            on_change=State.set_port,
            placeholder="Select serial port…",
            width="100%",
            size="2",
        ),
        rx.hstack(
            _btn("Connect",    State.do_connect,    disabled=State.connected | State.busy, flex="1", size="2"),
            _btn("Disconnect", State.do_disconnect, disabled=~State.connected | State.busy, color="red", flex="1", size="2"),
            spacing="2",
            width="100%",
        ),
        rx.divider(),
        rx.grid(
            _lbl("Actuator ID"), _empty(State.actuator_id),
            _lbl("Firmware"),    _empty(State.firmware),
            _lbl("Hardware"),    _empty(State.hardware),
            _lbl("Mode"),        _empty(State.mode_name),
            _lbl("Faults"),      _empty(State.fault_text),
            columns="2",
            spacing="2",
            width="100%",
        ),
    )


def control_panel() -> rx.Component:
    return _panel(
        _sec("Manual Control"),
        rx.hstack(
            _btn("Disabled",    State.mode_disabled,    disabled=~State.can_control, flex="1"),
            _btn("Calibration", State.mode_calibration, disabled=~State.can_control, flex="1"),
            spacing="2",
            width="100%",
        ),
        rx.hstack(
            _btn("Stop",  State.do_stop,  disabled=~State.can_control, color="orange", flex="1"),
            _btn("ESTOP", State.do_estop, disabled=~State.can_control, color="red", flex="1"),
            spacing="2",
            width="100%",
        ),
        rx.hstack(
            rx.input(
                value=State.jog_step.to_string(),
                on_change=State.set_jog_step,
                width="78px",
                size="1",
            ),
            rx.text("rad", size="1"),
            _btn("◀ Jog", State.jog_neg, disabled=~State.can_test, flex="1"),
            _btn("Jog ▶", State.jog_pos, disabled=~State.can_test, flex="1"),
            spacing="2",
            align="center",
            width="100%",
        ),
        rx.grid(
            _lbl("Move Δ (rad)"),    rx.input(value=State.move_delta.to_string(),    on_change=State.set_move_delta,    size="1", width="100%"),
            _lbl("Velocity (rad/s)"), rx.input(value=State.move_velocity.to_string(), on_change=State.set_move_velocity, size="1", width="100%"),
            _lbl("Accel (rad/s²)"),   rx.input(value=State.move_accel.to_string(),    on_change=State.set_move_accel,    size="1", width="100%"),
            columns="2",
            spacing="2",
            width="100%",
        ),
        rx.hstack(
            _btn("Move Relative", State.do_move_rel,  disabled=~State.can_test, flex="1"),
            _btn("Zero Motor",    State.zero_motor,   disabled=~State.can_test, flex="1"),
            _btn("Zero Output",   State.zero_output,  disabled=~State.can_test, flex="1"),
            spacing="2",
            wrap="wrap",
            width="100%",
        ),
    )


def tests_panel() -> rx.Component:
    btns = [
        ("Actuator Detection",       State.run_detection,     ~State.can_control),
        ("Encoder Sanity Test",      State.run_encoder_sanity, ~State.can_test),
        ("Ratio Calibration",        State.run_ratio,          ~State.can_test),
        ("Resonance Chirp Test",     State.run_resonance,      ~State.can_test),
        ("Backlash Test",            State.run_backlash,        ~State.can_test),
        ("Step Response Test",       State.run_step_response,   ~State.can_test),
        ("Velocity Ramp Test",       State.run_velocity_ramp,   ~State.can_test),
        ("Compliance / Load Test",   State.run_compliance,      ~State.can_test),
    ]
    return _panel(
        _sec("Calibration / Tests"),
        *[_btn(lbl, h, disabled=d, color="blue", width="100%") for lbl, h, d in btns],
        _btn("Full Auto-characterization",    State.run_auto,    disabled=~State.can_test, color="green", width="100%"),
        _btn("Save Calibration to Actuator",  State.save_config, disabled=~State.can_test, color="amber", width="100%"),
        _btn("Export Report",                 State.export_report, disabled=~State.connected, width="100%"),
    )


def results_panel() -> rx.Component:
    return _panel(
        _sec("Results"),
        rx.grid(
            _lbl("Ratio"),           _val(State.ratio_str),
            _lbl("Offset"),          _val(State.offset_str),
            _lbl("Fit error"),       _val(State.fit_error_str),
            _lbl("Hysteresis"),      _val(State.hysteresis_str),
            _lbl("Resonance"),       _val(State.resonance_str),
            _lbl("Samples / drop"),  rx.hstack(_val(State.sample_count), rx.text("/", size="1"), _val(State.dropped), spacing="1"),
            columns="2",
            spacing="2",
            width="100%",
        ),
        _lbl("Warnings"),
        rx.text(State.warnings_text, size="1", color=rx.color("amber", 11), white_space="pre-wrap"),
        flex="1",
        min_width="260px",
    )


def report_panel() -> rx.Component:
    return _panel(
        _sec("Report / Log"),
        rx.text(State.report_folder, size="1", font_family="monospace", color=rx.color("gray", 10)),
        rx.hstack(
            rx.text_area(
                value=State.notes,
                on_change=State.set_notes,
                placeholder="Notes…",
                rows="3",
                flex="1",
                size="1",
            ),
            rx.vstack(
                _btn("Open Folder", State.open_folder, color="gray"),
                spacing="1",
            ),
            align="start",
            width="100%",
            spacing="2",
        ),
        flex="1",
        min_width="260px",
    )


def plots_panel() -> rx.Component:
    return _panel(
        _sec("Live Telemetry"),
        rx.box(
            rx.el.iframe(
                src=f"http://localhost:{BOKEH_PORT}",
                width="100%",
                height="430px",
                style={"border": "none", "display": "block", "border_radius": "6px"},
            ),
            rx.cond(
                State.sample_count == 0,
                rx.box(
                    rx.vstack(
                        rx.text("No telemetry yet", size="3", weight="medium"),
                        rx.text("Connect to start the stream", size="1", color=rx.color("gray", 10)),
                        spacing="1",
                        align="center",
                    ),
                    position="absolute",
                    top="46%",
                    left="50%",
                    transform="translate(-50%, -50%)",
                    padding="12px 16px",
                    border=f"1px solid {rx.color('gray', 6)}",
                    border_radius="8px",
                    background=rx.color("gray", 2),
                    pointer_events="none",
                ),
                rx.fragment(),
            ),
            position="relative",
            width="100%",
        ),
        width="100%",
    )


def index() -> rx.Component:
    return rx.fragment(
        rx.color_mode.button(position="fixed", top="8px", right="8px"),
        rx.hstack(
            # ── Left sidebar ──────────────────────────────────────────
            rx.scroll_area(
                rx.vstack(
                    connection_panel(),
                    control_panel(),
                    tests_panel(),
                    padding="14px",
                    padding_bottom="48px",
                    spacing="3",
                    width="100%",
                ),
                type="auto",
                width="340px",
                min_width="340px",
                height="calc(100vh - 32px)",
                background=rx.color("gray", 1),
                style={"border_right": f"1px solid {rx.color('gray', 6)}"},
            ),
            # ── Right main area ───────────────────────────────────────
            rx.vstack(
                header_panel(),
                plots_panel(),
                rx.hstack(
                    results_panel(),
                    report_panel(),
                    spacing="3",
                    width="100%",
                    align="stretch",
                    wrap="wrap",
                ),
                flex="1",
                min_width="0",
                overflow_y="auto",
                padding="14px",
                padding_bottom="48px",
                spacing="3",
                align="start",
                height="calc(100vh - 32px)",
                background=rx.color("gray", 1),
            ),
            width="100%",
            min_width="0",
            spacing="0",
            align="start",
            background=rx.color("gray", 1),
        ),
        # ── Status bar ────────────────────────────────────────────────
        rx.box(
            rx.hstack(
                rx.cond(
                    State.busy,
                    rx.spinner(size="1"),
                    rx.cond(
                        State.connected,
                        rx.icon("circle", size=10, color="green"),
                        rx.icon("circle", size=10, color=rx.color("gray", 9)),
                    ),
                ),
                rx.text(State.status, size="1"),
                spacing="2",
                align="center",
            ),
            position="fixed",
            bottom="0",
            left="0",
            right="0",
            padding_x="12px",
            height="28px",
            background=rx.color("gray", 2),
            border_top=f"1px solid {rx.color('gray', 6)}",
            z_index="100",
            display="flex",
            align_items="center",
        ),
    )


# ── Boot ──────────────────────────────────────────────────────────────────────

def _start_bokeh() -> None:
    global _bokeh_started
    if _bokeh_started:
        return
    _bokeh_started = True
    launch_bokeh_thread(_ctx)


_start_bokeh()

app = rx.App(
    theme=rx.theme(appearance="dark", accent_color="blue", radius="small"),
)
app.add_page(
    index,
    route="/",
    title="Actuator Bench Tool",
    on_load=[State.scan_ports, State.start_polling],
)
