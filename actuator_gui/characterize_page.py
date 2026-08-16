"""Guided physical-plant characterization page for the actuator bench GUI."""

from __future__ import annotations

import asyncio
from dataclasses import dataclass, field
from datetime import datetime
import json
import math
from pathlib import Path
import threading
import time

import reflex as rx

from actuator_tool.actuator_tests import (
    run_compliance_test,
    run_ratio_calibration,
    run_resonance_test,
    run_step_response_test,
)
from actuator_tool.characterization_workflow import (
    GuidedCharacterizationSummary,
    apply_static_stiffness,
    capture_static_encoder_samples,
    characterize_encoders_from_samples,
    estimate_latency_from_samples,
    fit_damping_and_motor_inertia,
    fit_ringdown_from_samples,
    fit_torque_speed_text,
    mechanical_replay_validation,
    plant_from_calibration,
    run_friction_velocity_sweep,
    run_output_inertia_excitation,
    save_workflow_artifacts,
    update_plant,
)
from actuator_tool.plant_schema import ActuatorPlantParameters

from .actuator_gui import DESIGN_CSS, _REPORT_ROOT, _ctx, _executor, app


GUIDE_CSS = r"""
.characterize-shell { max-width: 1320px; margin: 0 auto; padding: 24px 24px 72px; }
.characterize-top { display:flex; align-items:flex-start; justify-content:space-between; gap:20px; margin-bottom:20px; }
.characterize-title { font-family:'Space Grotesk',sans-serif; font-size:30px; font-weight:700; letter-spacing:-0.04em; }
.characterize-sub { color:#94a3b8; max-width:850px; line-height:1.55; margin-top:5px; }
.workflow-rail { display:grid; grid-template-columns:repeat(5,minmax(0,1fr)); gap:8px; margin:18px 0 22px; }
.workflow-chip { border:1px solid #25324a; border-radius:10px; padding:10px 11px; background:#0d1423; min-height:58px; }
.workflow-chip.done { border-color:#166534; background:#071b13; }
.workflow-chip.running { border-color:#2563eb; background:#0a1833; }
.workflow-chip .n { color:#64748b; font:500 11px 'JetBrains Mono',monospace; }
.workflow-chip .t { color:#dbeafe; font:600 12px 'Space Grotesk',sans-serif; margin-top:3px; }
.guide-grid { display:grid; grid-template-columns:repeat(2,minmax(0,1fr)); gap:14px; }
.guide-card { border:1px solid #243149; background:#0b1220; border-radius:14px; padding:17px; min-width:0; }
.guide-card.full { grid-column:1 / -1; }
.guide-card h3 { font:600 17px 'Space Grotesk',sans-serif; color:#e5eefc; margin:0; }
.guide-card .desc { color:#94a3b8; font-size:13px; line-height:1.5; margin:7px 0 13px; }
.guide-result { margin-top:12px; padding:10px 11px; background:#070d18; border:1px solid #1d293d; border-radius:9px; color:#a5b4fc; font:500 12px/1.45 'JetBrains Mono',monospace; white-space:pre-wrap; overflow-wrap:anywhere; }
.guide-fields { display:grid; grid-template-columns:repeat(2,minmax(0,1fr)); gap:9px; margin:9px 0 12px; }
.guide-field label { display:block; color:#94a3b8; font-size:11px; margin-bottom:4px; }
.guide-warning { color:#fbbf24; background:#211707; border:1px solid #713f12; border-radius:9px; padding:10px 11px; font-size:12px; line-height:1.45; margin:9px 0; }
.guide-ok { color:#86efac; }
.guide-muted { color:#64748b; }
.guide-actions { display:flex; flex-wrap:wrap; gap:8px; align-items:center; }
.guide-status { border:1px solid #25324a; background:#08101e; border-radius:12px; padding:12px 14px; margin-bottom:14px; font:500 12px 'JetBrains Mono',monospace; color:#cbd5e1; }
@media (max-width: 900px) { .guide-grid { grid-template-columns:1fr; } .guide-card.full { grid-column:auto; } .workflow-rail { grid-template-columns:repeat(2,minmax(0,1fr)); } .characterize-top { flex-direction:column; } }
@media (max-width: 560px) { .characterize-shell { padding:16px 12px 56px; } .guide-fields { grid-template-columns:1fr; } .workflow-rail { grid-template-columns:1fr; } }
"""


@dataclass
class _WorkflowBackend:
    plant: ActuatorPlantParameters | None = None
    stiffness_nm_per_rad: float | None = None
    friction: object | None = None
    output_inertia: object | None = None
    motor_inertia_derived_kg_m2: float | None = None
    belt_damping: object | None = None
    torque_speed: object | None = None
    latency: object | None = None
    motor_encoder: object | None = None
    output_encoder: object | None = None
    step_samples: list = field(default_factory=list)
    validation: object | None = None
    plant_path: Path | None = None
    summary_path: Path | None = None
    lock: threading.Lock = field(default_factory=threading.Lock)

    def reset(self) -> None:
        with self.lock:
            self.plant = None
            self.stiffness_nm_per_rad = None
            self.friction = None
            self.output_inertia = None
            self.motor_inertia_derived_kg_m2 = None
            self.belt_damping = None
            self.torque_speed = None
            self.latency = None
            self.motor_encoder = None
            self.output_encoder = None
            self.step_samples = []
            self.validation = None
            self.plant_path = None
            self.summary_path = None


_flow = _WorkflowBackend()


def _progress(message: str) -> None:
    _ctx.set_status(message)
    _ctx.add_log("event", "CHAR", message)


def _report_event(name: str, details: dict | None = None) -> None:
    if _ctx.report is not None:
        _ctx.report.events.write(name, details or {})


def _output_folder() -> Path:
    if _ctx.report is not None:
        return _ctx.report.artifacts.folder
    root = Path(_REPORT_ROOT)
    folder = root / f"guided_characterization_{datetime.now().strftime('%Y-%m-%d_%H%M%S')}"
    folder.mkdir(parents=True, exist_ok=True)
    return folder


def _summary() -> GuidedCharacterizationSummary:
    if _flow.plant is None:
        raise ValueError("plant has not been initialized")
    return GuidedCharacterizationSummary(
        plant=_flow.plant,
        stiffness_nm_per_rad=_flow.stiffness_nm_per_rad,
        friction=_flow.friction,
        output_inertia=_flow.output_inertia,
        motor_inertia_derived_kg_m2=_flow.motor_inertia_derived_kg_m2,
        belt_damping=_flow.belt_damping,
        torque_speed=_flow.torque_speed,
        latency=_flow.latency,
        motor_encoder=_flow.motor_encoder,
        output_encoder=_flow.output_encoder,
        validation=_flow.validation,
    )


def _write_current_artifacts() -> tuple[Path, Path]:
    if _flow.plant is None:
        raise ValueError("no plant model to save")
    folder = _flow.plant_path.parent if _flow.plant_path is not None else _output_folder()
    plant_path, summary_path = save_workflow_artifacts(folder, _flow.plant, _summary())
    _flow.plant_path = plant_path
    _flow.summary_path = summary_path
    return plant_path, summary_path


def _parse_speeds(text: str) -> list[float]:
    values: list[float] = []
    for token in str(text).replace(";", ",").split(","):
        token = token.strip()
        if not token:
            continue
        value = float(token)
        if not math.isfinite(value):
            raise ValueError("friction speeds must be finite")
        values.append(value)
    if len(values) < 4 or not any(v < 0 for v in values) or not any(v > 0 for v in values):
        raise ValueError("friction sweep needs at least four speeds spanning both directions")
    return values


def _seed_plant(motor_inertia: float, output_inertia: float, ref_current: float | None, ref_voltage: float | None) -> None:
    _flow.plant = plant_from_calibration(
        _ctx.calibration,
        _ctx.info,
        motor_inertia_kg_m2=motor_inertia,
        output_inertia_kg_m2=output_inertia,
        torque_speed_reference_current_a=ref_current,
        torque_speed_reference_bus_voltage_v=ref_voltage,
    )


class CharacterizeState(rx.State):
    connected: bool = False
    busy: bool = False
    status: str = "Open the main Bench page and connect to hardware or simulator first."

    ratio_done: bool = False
    stiffness_done: bool = False
    friction_done: bool = False
    inertia_done: bool = False
    damping_done: bool = False
    torque_speed_done: bool = False
    latency_done: bool = False
    encoders_done: bool = False
    plant_done: bool = False
    validation_done: bool = False

    ratio_result: str = "Not run"
    stiffness_result: str = "Not run"
    friction_result: str = "Not run"
    inertia_result: str = "Not run"
    damping_result: str = "Not run"
    torque_speed_result: str = "Not run"
    latency_result: str = "Not run"
    encoder_result: str = "Not run"
    plant_result: str = "Not generated"
    validation_result: str = "Not run"

    stiffness_torque_nm: str = "0.50"
    motor_inertia_guess: str = "0.00001"
    output_inertia_guess: str = "0.001"
    friction_speeds: str = "-4,-2,-1,-0.5,0.5,1,2,4"
    friction_external_load_nm: str = "0"
    inertia_speed: str = "3.0"
    inertia_accel: str = "12.0"
    ringdown_duration_s: str = "5.0"
    torque_speed_points: str = ""
    torque_reference_current_a: str = "1.0"
    torque_reference_voltage_v: str = "24.0"
    encoder_motor_cpr: str = "4096"
    encoder_output_cpr: str = "4096"

    def refresh(self) -> None:
        self.connected = bool(_ctx.connected)
        self.status = _ctx.get_status() if _ctx.connected else "Not connected. Open Bench, connect, then return here."
        if _ctx.ratio_fit is not None:
            self.ratio_done = True
            self.ratio_result = (
                f"output/motor={_ctx.ratio_fit.output_per_motor:.8f}; "
                f"motor/output={_ctx.ratio_fit.motor_per_output:.5f}; "
                f"RMS residual={_ctx.ratio_fit.residual_rms_rad:.6f} rad"
            )

    def reset_workflow(self) -> None:
        _flow.reset()
        self.ratio_done = self.stiffness_done = self.friction_done = False
        self.inertia_done = self.damping_done = self.torque_speed_done = False
        self.latency_done = self.encoders_done = self.plant_done = self.validation_done = False
        self.ratio_result = self.stiffness_result = self.friction_result = "Not run"
        self.inertia_result = self.damping_result = self.torque_speed_result = "Not run"
        self.latency_result = self.encoder_result = "Not run"
        self.plant_result = "Not generated"
        self.validation_result = "Not run"
        self.status = "Guided characterization reset"

    def set_stiffness_torque_nm(self, value: str) -> None: self.stiffness_torque_nm = value
    def set_motor_inertia_guess(self, value: str) -> None: self.motor_inertia_guess = value
    def set_output_inertia_guess(self, value: str) -> None: self.output_inertia_guess = value
    def set_friction_speeds(self, value: str) -> None: self.friction_speeds = value
    def set_friction_external_load_nm(self, value: str) -> None: self.friction_external_load_nm = value
    def set_inertia_speed(self, value: str) -> None: self.inertia_speed = value
    def set_inertia_accel(self, value: str) -> None: self.inertia_accel = value
    def set_ringdown_duration_s(self, value: str) -> None: self.ringdown_duration_s = value
    def set_torque_speed_points(self, value: str) -> None: self.torque_speed_points = value
    def set_torque_reference_current_a(self, value: str) -> None: self.torque_reference_current_a = value
    def set_torque_reference_voltage_v(self, value: str) -> None: self.torque_reference_voltage_v = value
    def set_encoder_motor_cpr(self, value: str) -> None: self.encoder_motor_cpr = value
    def set_encoder_output_cpr(self, value: str) -> None: self.encoder_output_cpr = value

    def _require_connected(self) -> None:
        if not _ctx.connected:
            raise ValueError("not connected; use the main Bench page first")

    @rx.event(background=True)
    async def run_ratio(self) -> None:
        async with self:
            if self.busy: return
            self.busy = True
            self.status = "1/10 Ratio calibration running..."
        try:
            self._require_connected()
            loop = asyncio.get_event_loop()
            result = await loop.run_in_executor(
                _executor,
                lambda: run_ratio_calibration(_ctx.require_client(), _ctx.store, _ctx.info, _ctx.safety, _progress),
            )
            _ctx.ratio_fit = result.fit
            _ctx.calibration = result.calibration
            ref_current = float(self.torque_reference_current_a) if self.torque_reference_current_a.strip() else None
            ref_voltage = float(self.torque_reference_voltage_v) if self.torque_reference_voltage_v.strip() else None
            _seed_plant(float(self.motor_inertia_guess), float(self.output_inertia_guess), ref_current, ref_voltage)
            _report_event("guided_ratio", result.as_dict())
            async with self:
                self.ratio_done = bool(result.passed)
                self.ratio_result = (
                    f"N={result.fit.motor_per_output:.5f}:1; output/motor={result.fit.output_per_motor:.8f}; "
                    f"RMS={result.fit.residual_rms_rad:.6f} rad"
                )
                self.status = result.message
        except Exception as exc:
            async with self: self.status = f"Ratio failed: {exc}"
        finally:
            async with self: self.busy = False

    @rx.event(background=True)
    async def run_stiffness(self) -> None:
        async with self:
            if self.busy: return
            self.busy = True
            torque_text = self.stiffness_torque_nm
            self.status = "2/10 Static stiffness capture running..."
        try:
            self._require_connected()
            torque = float(torque_text)
            if torque == 0.0: raise ValueError("known applied torque must be non-zero")
            loop = asyncio.get_event_loop()
            result = await loop.run_in_executor(
                _executor,
                lambda: run_compliance_test(
                    _ctx.require_client(), _ctx.store, _ctx.calibration, _progress,
                    duration_s=4.0, known_torque_nm=torque,
                ),
            )
            if result.stiffness_nm_per_rad is None or result.stiffness_nm_per_rad <= 0.0:
                raise ValueError("stiffness could not be fitted; check applied torque and encoder deflection")
            _ctx.calibration.compliance_nm_per_rad = result.stiffness_nm_per_rad
            if _flow.plant is None:
                _seed_plant(float(self.motor_inertia_guess), float(self.output_inertia_guess), None, None)
            _flow.stiffness_nm_per_rad = result.stiffness_nm_per_rad
            _flow.plant = apply_static_stiffness(_flow.plant, result.stiffness_nm_per_rad)
            _report_event("guided_static_stiffness", result.as_dict())
            async with self:
                self.stiffness_done = True
                self.stiffness_result = (
                    f"K={result.stiffness_nm_per_rad:.4f} N*m/rad; "
                    f"mean deflection={result.mean_deflection_rad:.6f} rad; hysteresis={result.hysteresis_rad:.6f} rad"
                )
                self.status = "Static stiffness identified"
        except Exception as exc:
            async with self: self.status = f"Stiffness failed: {exc}"
        finally:
            async with self: self.busy = False

    @rx.event(background=True)
    async def run_friction(self) -> None:
        async with self:
            if self.busy: return
            self.busy = True
            speeds_text, load_text = self.friction_speeds, self.friction_external_load_nm
            self.status = "3/10 Friction sweep running..."
        try:
            self._require_connected()
            if _flow.plant is None: raise ValueError("run ratio and stiffness first")
            speeds = _parse_speeds(speeds_text)
            load = float(load_text)
            loop = asyncio.get_event_loop()
            fit, _samples = await loop.run_in_executor(
                _executor,
                lambda: run_friction_velocity_sweep(
                    _ctx.require_client(), _ctx.store, _flow.plant, _ctx.safety,
                    output_speeds_rad_s=speeds, external_load_torque_nm=load,
                ),
            )
            _flow.friction = fit
            _flow.plant = update_plant(_flow.plant, friction=fit)
            _report_event("guided_friction", fit.__dict__)
            async with self:
                self.friction_done = True
                self.friction_result = (
                    f"Coulomb={fit.coulomb_friction_nm:.5f} N*m; static={fit.static_friction_nm:.5f} N*m; "
                    f"viscous={fit.viscous_friction_nm_s_per_rad:.6f} N*m*s/rad; "
                    f"Vs={fit.stribeck_velocity_rad_s:.4f} rad/s; R2={fit.r_squared:.4f}"
                )
                self.status = "Friction model identified"
        except Exception as exc:
            async with self: self.status = f"Friction failed: {exc}"
        finally:
            async with self: self.busy = False

    @rx.event(background=True)
    async def run_inertia(self) -> None:
        async with self:
            if self.busy: return
            self.busy = True
            speed_text, accel_text = self.inertia_speed, self.inertia_accel
            self.status = "4/10 Inertia excitation running..."
        try:
            self._require_connected()
            if _flow.plant is None: raise ValueError("plant seed missing")
            loop = asyncio.get_event_loop()
            fit, _samples = await loop.run_in_executor(
                _executor,
                lambda: run_output_inertia_excitation(
                    _ctx.require_client(), _ctx.store, _flow.plant, _flow.friction, _ctx.safety,
                    target_output_speed_rad_s=float(speed_text), acceleration_rad_s2=float(accel_text),
                ),
            )
            if fit.inertia_kg_m2 <= 0.0: raise ValueError("inertia fit returned zero; excitation/load model is insufficient")
            _flow.output_inertia = fit
            _flow.plant = update_plant(_flow.plant, output_inertia=fit)
            _report_event("guided_output_inertia", fit.__dict__)
            async with self:
                self.inertia_done = True
                self.inertia_result = (
                    f"J_output={fit.inertia_kg_m2:.8g} kg*m^2; bias={fit.torque_bias_nm:.5f} N*m; "
                    f"R2={fit.r_squared:.4f}. Motor-side J is completed from the relative resonance mode in step 5."
                )
                self.status = "Output inertia identified"
        except Exception as exc:
            async with self: self.status = f"Inertia failed: {exc}"
        finally:
            async with self: self.busy = False

    @rx.event(background=True)
    async def run_chirp_damping(self) -> None:
        async with self:
            if self.busy: return
            self.busy = True
            self.status = "5/10 Chirp + step-response damping identification running..."
        try:
            self._require_connected()
            if _flow.plant is None: raise ValueError("plant seed missing")
            loop = asyncio.get_event_loop()
            resonance = await loop.run_in_executor(
                _executor,
                lambda: run_resonance_test(_ctx.require_client(), _ctx.store, _ctx.calibration, _ctx.safety, _progress),
            )
            _ctx.resonance_result = resonance.analysis
            _ctx.calibration = resonance.calibration
            start = _ctx.store.total_samples
            step = await loop.run_in_executor(
                _executor,
                lambda: run_step_response_test(
                    _ctx.require_client(), _ctx.store, _ctx.safety, _progress,
                    calibration=_ctx.calibration, step_rad=0.22, settle_capture_s=4.0,
                ),
            )
            _flow.step_samples = _ctx.store.samples_since(start)
            damping, motor_j = fit_damping_and_motor_inertia(_flow.plant, resonance.analysis, step_response=step)
            _flow.belt_damping = damping
            _flow.motor_inertia_derived_kg_m2 = motor_j
            _flow.plant = update_plant(_flow.plant, belt_damping=damping, motor_inertia_kg_m2=motor_j)
            _report_event("guided_resonance", resonance.as_dict())
            _report_event("guided_damping", damping.__dict__)
            async with self:
                self.damping_done = True
                motor_text = "unresolved" if motor_j is None else f"{motor_j:.8g} kg*m^2"
                self.damping_result = (
                    f"f_n={damping.natural_frequency_hz:.3f} Hz; zeta={damping.damping_ratio:.4f}; "
                    f"C={damping.damping_nm_s_per_rad:.6f} N*m*s/rad; motor J={motor_text}; source={damping.source}"
                )
                self.status = "Resonance and belt damping identified"
        except Exception as exc:
            async with self: self.status = f"Chirp/damping failed: {exc}"
        finally:
            async with self: self.busy = False

    @rx.event(background=True)
    async def run_ringdown(self) -> None:
        async with self:
            if self.busy: return
            self.busy = True
            duration_text = self.ringdown_duration_s
            self.status = "Ring-down capture: perturb the output and release now..."
        try:
            self._require_connected()
            if _flow.plant is None: raise ValueError("plant seed missing")
            duration = max(1.0, float(duration_text))
            loop = asyncio.get_event_loop()
            def capture():
                client = _ctx.require_client()
                client.set_mode(ActuatorMode.CALIBRATION)
                client.start_stream()
                start = _ctx.store.total_samples
                time.sleep(duration)
                return _ctx.store.samples_since(start)
            # Local import avoids broadening the main page's protocol imports.
            from actuator_tool.actuator_protocol import ActuatorMode
            samples = await loop.run_in_executor(_executor, capture)
            damping = fit_ringdown_from_samples(samples, _flow.plant, output_offset_rad=_ctx.calibration.output_offset_rad)
            _flow.belt_damping = damping
            _flow.plant = update_plant(_flow.plant, belt_damping=damping)
            _report_event("guided_ringdown", damping.__dict__)
            async with self:
                self.damping_done = True
                self.damping_result = (
                    f"Ring-down: f_n={damping.natural_frequency_hz:.3f} Hz; zeta={damping.damping_ratio:.4f}; "
                    f"C={damping.damping_nm_s_per_rad:.6f} N*m*s/rad"
                )
                self.status = "Ring-down damping fit complete"
        except Exception as exc:
            async with self: self.status = f"Ring-down failed: {exc}"
        finally:
            async with self: self.busy = False

    def fit_torque_speed(self) -> None:
        try:
            if _flow.plant is None: raise ValueError("plant seed missing")
            fit = fit_torque_speed_text(self.torque_speed_points)
            _flow.torque_speed = fit
            data = _flow.plant.as_dict()
            current = float(self.torque_reference_current_a) if self.torque_reference_current_a.strip() else None
            voltage = float(self.torque_reference_voltage_v) if self.torque_reference_voltage_v.strip() else None
            data["torque_speed_reference_current_a"] = current
            data["torque_speed_reference_bus_voltage_v"] = voltage
            _flow.plant = ActuatorPlantParameters.from_dict(data)
            _flow.plant = update_plant(_flow.plant, torque_speed=fit)
            self.torque_speed_done = True
            self.torque_speed_result = "\n".join(
                f"{p.speed_rad_s:.3f} rad/s -> {p.torque_nm:.4f} N*m" for p in fit.points
            )
            self.status = "Measured torque-speed map fitted"
            _report_event("guided_torque_speed", {"points": [p.__dict__ for p in fit.points]})
        except Exception as exc:
            self.status = f"Torque-speed fit failed: {exc}"

    def fit_latency(self) -> None:
        try:
            if _flow.plant is None: raise ValueError("plant seed missing")
            if len(_flow.step_samples) < 16: raise ValueError("run the chirp/step-response step first")
            fit = estimate_latency_from_samples(_flow.step_samples)
            _flow.latency = fit
            _flow.plant = update_plant(_flow.plant, latency=fit)
            self.latency_done = True
            self.latency_result = (
                f"command->motor lag={fit.latency_s*1000.0:.3f} ms; correlation={fit.correlation:.4f}; "
                f"sample period={fit.sample_period_s*1000.0:.3f} ms; timing jitter={fit.sample_jitter_std_s*1e6:.1f} us"
            )
            self.status = "Latency/jitter characterized from the step trace"
            _report_event("guided_latency", fit.__dict__)
        except Exception as exc:
            self.status = f"Latency fit failed: {exc}"

    @rx.event(background=True)
    async def run_encoder_characterization(self) -> None:
        async with self:
            if self.busy: return
            self.busy = True
            motor_cpr, output_cpr = self.encoder_motor_cpr, self.encoder_output_cpr
            self.status = "8/10 Static encoder/noise capture running..."
        try:
            self._require_connected()
            if _flow.plant is None: raise ValueError("plant seed missing")
            loop = asyncio.get_event_loop()
            samples = await loop.run_in_executor(
                _executor,
                lambda: capture_static_encoder_samples(_ctx.require_client(), _ctx.store, duration_s=2.5),
            )
            motor, output = characterize_encoders_from_samples(
                samples,
                motor_counts_per_rev=int(motor_cpr),
                output_counts_per_rev=int(output_cpr),
            )
            _flow.motor_encoder, _flow.output_encoder = motor, output
            _flow.plant = update_plant(_flow.plant, motor_encoder=motor, output_encoder=output)
            _report_event("guided_encoders", {"motor": motor.__dict__, "output": output.__dict__})
            async with self:
                self.encoders_done = True
                self.encoder_result = (
                    f"motor: q={motor.quantization_step_rad:.7f} rad, noise={motor.noise_std_rad:.7g} rad, "
                    f"period={motor.sample_period_s*1000:.3f} ms, jitter={motor.sample_jitter_std_s*1e6:.1f} us\n"
                    f"output: q={output.quantization_step_rad:.7f} rad, noise={output.noise_std_rad:.7g} rad, "
                    f"period={output.sample_period_s*1000:.3f} ms, jitter={output.sample_jitter_std_s*1e6:.1f} us"
                )
                self.status = "Encoder models characterized"
        except Exception as exc:
            async with self: self.status = f"Encoder characterization failed: {exc}"
        finally:
            async with self: self.busy = False

    def generate_plant(self) -> None:
        try:
            if _flow.plant is None: raise ValueError("plant seed missing")
            path, summary = _write_current_artifacts()
            self.plant_done = True
            self.plant_result = f"plant.json: {path}\nworkflow summary: {summary}"
            self.status = "plant.json generated"
            _report_event("guided_plant_saved", {"plant": str(path), "summary": str(summary)})
        except Exception as exc:
            self.status = f"plant.json generation failed: {exc}"

    @rx.event(background=True)
    async def run_validation(self) -> None:
        async with self:
            if self.busy: return
            self.busy = True
            self.status = "10/10 Running a fresh hold-out step and mechanical replay validation..."
        try:
            self._require_connected()
            if _flow.plant is None: raise ValueError("generate the plant first")
            loop = asyncio.get_event_loop()
            start = _ctx.store.total_samples
            await loop.run_in_executor(
                _executor,
                lambda: run_step_response_test(
                    _ctx.require_client(), _ctx.store, _ctx.safety, _progress,
                    calibration=_ctx.calibration, step_rad=0.173, settle_capture_s=4.5,
                ),
            )
            samples = _ctx.store.samples_since(start)
            _sim, report = mechanical_replay_validation(samples, _flow.plant)
            _flow.validation = report
            path, summary = _write_current_artifacts()
            validation_path = path.parent / "plant_validation.json"
            validation_path.write_text(json.dumps(report.as_dict(), indent=2) + "\n", encoding="utf-8")
            _report_event("guided_validation", report.as_dict())
            async with self:
                self.validation_done = True
                nrmse = report.mean_normalized_rmse
                nrmse_text = "n/a" if nrmse is None else f"{100*nrmse:.2f}%"
                output = report.metrics.get("output_angle_rad")
                out_text = "n/a" if output is None else f"{output.rmse:.6f} rad"
                self.validation_result = (
                    f"mean normalized RMSE={nrmse_text}; output-angle RMSE={out_text}; "
                    f"samples={report.sample_count}; report={validation_path}"
                )
                self.plant_done = True
                self.plant_result = f"plant.json: {path}\nworkflow summary: {summary}"
                self.status = "Guided characterization and hold-out validation complete"
        except Exception as exc:
            async with self: self.status = f"Validation failed: {exc}"
        finally:
            async with self: self.busy = False


def _field(label: str, value, handler, *, placeholder: str = "") -> rx.Component:
    return rx.box(
        rx.text(label, as_="label"),
        rx.input(value=value, on_change=handler, placeholder=placeholder, width="100%"),
        class_name="guide-field",
    )


def _result(text) -> rx.Component:
    return rx.box(text, class_name="guide-result")


def _card(number: int, title: str, description: str, body: rx.Component, result, done, *, full: bool = False) -> rx.Component:
    return rx.box(
        rx.hstack(
            rx.badge(f"{number:02d}", variant="soft"),
            rx.heading(title, size="4"),
            rx.spacer(),
            rx.cond(done, rx.badge("DONE", color_scheme="green"), rx.badge("PENDING", color_scheme="gray")),
            width="100%",
            align="center",
        ),
        rx.text(description, class_name="desc"),
        body,
        _result(result),
        class_name="guide-card full" if full else "guide-card",
    )


def _rail_chip(number: int, title: str, done) -> rx.Component:
    return rx.box(
        rx.text(f"{number:02d}", class_name="n"),
        rx.text(title, class_name="t"),
        class_name=rx.cond(done, "workflow-chip done", "workflow-chip"),
    )


def characterize_page() -> rx.Component:
    return rx.fragment(
        rx.el.style(DESIGN_CSS + GUIDE_CSS),
        rx.box(
            rx.box(
                rx.box(
                    rx.heading("Characterize actuator", class_name="characterize-title"),
                    rx.text(
                        "Guided system-identification workflow: ratio -> stiffness -> friction -> inertia -> "
                        "resonance/damping -> torque-speed -> latency -> encoders -> plant.json -> hold-out validation.",
                        class_name="characterize-sub",
                    ),
                ),
                rx.hstack(
                    rx.link(rx.button("Back to Bench", variant="outline"), href="/"),
                    rx.button("Refresh connection", on_click=CharacterizeState.refresh, variant="outline"),
                    rx.button("Reset workflow", on_click=CharacterizeState.reset_workflow, color_scheme="red", variant="soft"),
                    spacing="2",
                ),
                class_name="characterize-top",
            ),
            rx.box(
                rx.cond(CharacterizeState.connected, "Connected to the bench backend. ", "NOT CONNECTED. "),
                CharacterizeState.status,
                class_name="guide-status",
            ),
            rx.box(
                _rail_chip(1, "Ratio", CharacterizeState.ratio_done),
                _rail_chip(2, "Stiffness", CharacterizeState.stiffness_done),
                _rail_chip(3, "Friction", CharacterizeState.friction_done),
                _rail_chip(4, "Inertia", CharacterizeState.inertia_done),
                _rail_chip(5, "Damping", CharacterizeState.damping_done),
                _rail_chip(6, "Torque-speed", CharacterizeState.torque_speed_done),
                _rail_chip(7, "Latency", CharacterizeState.latency_done),
                _rail_chip(8, "Encoders", CharacterizeState.encoders_done),
                _rail_chip(9, "plant.json", CharacterizeState.plant_done),
                _rail_chip(10, "Validation", CharacterizeState.validation_done),
                class_name="workflow-rail",
            ),
            rx.box(
                _card(
                    1, "Ratio calibration",
                    "Runs the existing bidirectional ratio fit and seeds the physical plant. Initial inertia values are only starting guesses; output inertia is fitted later and motor inertia can be inferred from the relative resonance mode.",
                    rx.box(
                        rx.box(
                            _field("Motor inertia starting guess [kg*m^2]", CharacterizeState.motor_inertia_guess, CharacterizeState.set_motor_inertia_guess),
                            _field("Output inertia starting guess [kg*m^2]", CharacterizeState.output_inertia_guess, CharacterizeState.set_output_inertia_guess),
                            class_name="guide-fields",
                        ),
                        rx.button("Run ratio", on_click=CharacterizeState.run_ratio, loading=CharacterizeState.busy),
                    ),
                    CharacterizeState.ratio_result, CharacterizeState.ratio_done,
                ),
                _card(
                    2, "Static stiffness",
                    "Apply a known output torque and hold it steady during the four-second capture. The dual encoders directly measure transmission deflection, giving belt/drivetrain stiffness.",
                    rx.box(
                        rx.box("Physically apply the stated torque before starting. A load cell or known mass on a measured lever arm is strongly preferred.", class_name="guide-warning"),
                        _field("Known applied output torque [N*m]", CharacterizeState.stiffness_torque_nm, CharacterizeState.set_stiffness_torque_nm),
                        rx.button("Capture stiffness", on_click=CharacterizeState.run_stiffness, loading=CharacterizeState.busy),
                    ),
                    CharacterizeState.stiffness_result, CharacterizeState.stiffness_done,
                ),
                _card(
                    3, "Friction sweep",
                    "Runs steady velocity segments in both directions. Belt torque from the encoder deflection is combined with acceleration to fit Coulomb + viscous + Stribeck friction.",
                    rx.box(
                        rx.box(
                            _field("Output speeds [rad/s], comma separated", CharacterizeState.friction_speeds, CharacterizeState.set_friction_speeds),
                            _field("Known constant external load [N*m]", CharacterizeState.friction_external_load_nm, CharacterizeState.set_friction_external_load_nm),
                            class_name="guide-fields",
                        ),
                        rx.button("Run friction sweep", on_click=CharacterizeState.run_friction, loading=CharacterizeState.busy),
                    ),
                    CharacterizeState.friction_result, CharacterizeState.friction_done,
                ),
                _card(
                    4, "Inertia test",
                    "Alternating velocity commands create positive and negative acceleration. Fitted belt torque minus friction gives output inertia. Motor-side inertia is refined in the next step from the measured two-inertia resonance.",
                    rx.box(
                        rx.box(
                            _field("Target output speed [rad/s]", CharacterizeState.inertia_speed, CharacterizeState.set_inertia_speed),
                            _field("Acceleration [rad/s^2]", CharacterizeState.inertia_accel, CharacterizeState.set_inertia_accel),
                            class_name="guide-fields",
                        ),
                        rx.button("Run inertia excitation", on_click=CharacterizeState.run_inertia, loading=CharacterizeState.busy),
                    ),
                    CharacterizeState.inertia_result, CharacterizeState.inertia_done,
                ),
                _card(
                    5, "Chirp / ring-down and belt damping",
                    "The automatic path runs the existing chirp and a step response, then converts resonance Q or step damping into physical belt damping. A manual free ring-down capture is available as an independent check.",
                    rx.box(
                        rx.hstack(
                            rx.button("Run chirp + step", on_click=CharacterizeState.run_chirp_damping, loading=CharacterizeState.busy),
                            rx.button("Capture ring-down", on_click=CharacterizeState.run_ringdown, loading=CharacterizeState.busy, variant="outline"),
                            spacing="2", wrap="wrap",
                        ),
                        _field("Ring-down capture duration [s]", CharacterizeState.ringdown_duration_s, CharacterizeState.set_ringdown_duration_s),
                    ),
                    CharacterizeState.damping_result, CharacterizeState.damping_done,
                ),
                _card(
                    6, "Measured NEMA17 torque-speed map",
                    "Paste measured motor-side speed/available-torque points. The fitter bins noisy samples and enforces a non-increasing envelope used by the digital twin.",
                    rx.box(
                        rx.box(
                            "Do not use electrical input power as torque. These points should come from a load cell/dynamometer or another calibrated load measurement at the stated current and bus voltage.",
                            class_name="guide-warning",
                        ),
                        rx.text_area(
                            value=CharacterizeState.torque_speed_points,
                            on_change=CharacterizeState.set_torque_speed_points,
                            placeholder="0,0.48\n25,0.46\n50,0.41\n100,0.30\n150,0.19",
                            width="100%", min_height="150px",
                        ),
                        rx.box(
                            _field("Reference phase/current setting [A]", CharacterizeState.torque_reference_current_a, CharacterizeState.set_torque_reference_current_a),
                            _field("Reference bus voltage [V]", CharacterizeState.torque_reference_voltage_v, CharacterizeState.set_torque_reference_voltage_v),
                            class_name="guide-fields",
                        ),
                        rx.button("Fit measured map", on_click=CharacterizeState.fit_torque_speed),
                    ),
                    CharacterizeState.torque_speed_result, CharacterizeState.torque_speed_done,
                ),
                _card(
                    7, "Control latency and timing jitter",
                    "Uses the fresh step-response trace from step 5 and cross-correlates commanded motor position against measured motor motion. This includes command/firmware/actuator/sensor delay visible in telemetry.",
                    rx.button("Estimate latency", on_click=CharacterizeState.fit_latency),
                    CharacterizeState.latency_result, CharacterizeState.latency_done,
                ),
                _card(
                    8, "Encoder resolution / noise / timing",
                    "Captures the actuator at rest, then measures quantization, residual angle noise, sample period and timestamp jitter independently for motor and output encoders.",
                    rx.box(
                        rx.box(
                            _field("Motor encoder CPR", CharacterizeState.encoder_motor_cpr, CharacterizeState.set_encoder_motor_cpr),
                            _field("Output encoder CPR", CharacterizeState.encoder_output_cpr, CharacterizeState.set_encoder_output_cpr),
                            class_name="guide-fields",
                        ),
                        rx.button("Capture encoder statistics", on_click=CharacterizeState.run_encoder_characterization, loading=CharacterizeState.busy),
                    ),
                    CharacterizeState.encoder_result, CharacterizeState.encoders_done,
                ),
                _card(
                    9, "Generate plant.json",
                    "Serializes the identified reduction, two inertias, belt stiffness/damping, friction model, torque-speed envelope, latency/jitter and encoder models into the shared ActuatorPlantParameters schema.",
                    rx.button("Generate plant.json", on_click=CharacterizeState.generate_plant),
                    CharacterizeState.plant_result, CharacterizeState.plant_done,
                    full=True,
                ),
                _card(
                    10, "Hold-out digital-twin validation",
                    "Runs a fresh, deliberately different step and replays encoder-inferred motor torque through the two-inertia simulator. This validates the mechanical plant on data not used for the fits. It is not yet a full electrical current-to-torque validation.",
                    rx.box(
                        rx.box(
                            "Mechanical replay intentionally avoids pretending TMC2209 current equals stepper shaft torque. A future calibrated electrical/rotor-angle model can extend this to command-to-motion validation.",
                            class_name="guide-warning",
                        ),
                        rx.button("Run hold-out validation", on_click=CharacterizeState.run_validation, loading=CharacterizeState.busy),
                    ),
                    CharacterizeState.validation_result, CharacterizeState.validation_done,
                    full=True,
                ),
                class_name="guide-grid",
            ),
            class_name="characterize-shell",
        ),
    )


app.add_page(
    characterize_page,
    route="/characterize",
    title="Characterize Actuator | Actuator Bench Tool",
    on_load=CharacterizeState.refresh,
)
