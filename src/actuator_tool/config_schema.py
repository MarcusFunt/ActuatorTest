"""Calibration/config structures and validation."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
import json
from pathlib import Path
from typing import Any

CALIBRATION_SCHEMA_VERSION = 2


def utc_timestamp() -> str:
    return datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds")


@dataclass
class SafetyLimits:
    max_velocity_rad_s: float = 40.0
    max_accel_rad_s2: float = 1000.0
    jog_step_rad: float = 0.25
    calibration_velocity_rad_s: float = 3.5
    calibration_accel_rad_s2: float = 80.0
    max_move_rad: float = 60.0

    def validate(self) -> None:
        for name, value in asdict(self).items():
            if float(value) <= 0:
                raise ValueError(f"{name} must be positive")

    def clamp_velocity(self, velocity: float) -> float:
        return max(0.01, min(abs(float(velocity)), self.max_velocity_rad_s))

    def clamp_accel(self, accel: float) -> float:
        return max(0.01, min(abs(float(accel)), self.max_accel_rad_s2))

    def clamp_delta(self, delta: float) -> float:
        return max(-self.max_move_rad, min(float(delta), self.max_move_rad))


@dataclass
class CalibrationConfig:
    calibration_version: int = CALIBRATION_SCHEMA_VERSION
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
    velocity_pid_kp: float = 0.2
    velocity_pid_ki: float = 2.0
    velocity_pid_i_limit_motor_rad: float = 0.2
    torque_proxy_kp: float = 3.0
    torque_proxy_limit_rad: float = 0.12
    torque_proxy_max_motor_velocity_rad_s: float = 4.0
    torque_proxy_timeout_s: float = 3.0
    missed_step_correction_enabled: bool = True
    missed_step_warn_motor_rad: float = 0.05
    missed_step_fault_motor_rad: float = 0.25
    missed_step_correction_rate: float = 0.25
    current_control_enabled: bool = True
    idle_current_ma: int = 0
    hold_current_ma: int = 350
    run_current_ma: int = 1000
    current_downshift_delay_s: float = 0.5
    autotune_max_amplitude_rad: float = 0.4
    autotune_max_duration_s: float = 15.0
    autotune_max_deflection_rad: float = 0.25
    backlash_comp_enabled: bool = False
    resonance_derating_enabled: bool = False
    calibrated_at: str = field(default_factory=utc_timestamp)

    def validate(self) -> None:
        if self.calibration_version != CALIBRATION_SCHEMA_VERSION:
            raise ValueError(f"unsupported calibration version {self.calibration_version}")
        if self.motor_encoder_sign not in (-1, 1):
            raise ValueError("motor_encoder_sign must be -1 or 1")
        if self.output_encoder_sign not in (-1, 1):
            raise ValueError("output_encoder_sign must be -1 or 1")
        if abs(self.output_per_motor) < 1e-9:
            raise ValueError("output_per_motor must be non-zero")
        if abs(self.motor_per_output) < 1e-9:
            raise ValueError("motor_per_output must be non-zero")
        if self.max_safe_velocity_rad_s <= 0:
            raise ValueError("max_safe_velocity_rad_s must be positive")
        if self.max_safe_accel_rad_s2 <= 0:
            raise ValueError("max_safe_accel_rad_s2 must be positive")
        if self.pid_i_limit_motor_rad < 0:
            raise ValueError("pid_i_limit_motor_rad must be non-negative")
        if self.pid_output_limit_motor_rad < 0:
            raise ValueError("pid_output_limit_motor_rad must be non-negative")
        if self.velocity_pid_kp < 0 or self.velocity_pid_ki < 0:
            raise ValueError("velocity PID gains must be non-negative")
        if self.velocity_pid_i_limit_motor_rad < 0:
            raise ValueError("velocity_pid_i_limit_motor_rad must be non-negative")
        if self.torque_proxy_kp < 0:
            raise ValueError("torque_proxy_kp must be non-negative")
        if self.torque_proxy_limit_rad <= 0:
            raise ValueError("torque_proxy_limit_rad must be positive")
        if self.torque_proxy_max_motor_velocity_rad_s <= 0:
            raise ValueError("torque_proxy_max_motor_velocity_rad_s must be positive")
        if self.torque_proxy_timeout_s <= 0:
            raise ValueError("torque_proxy_timeout_s must be positive")
        if self.missed_step_warn_motor_rad < 0:
            raise ValueError("missed_step_warn_motor_rad must be non-negative")
        if self.missed_step_fault_motor_rad <= 0:
            raise ValueError("missed_step_fault_motor_rad must be positive")
        if self.missed_step_warn_motor_rad > self.missed_step_fault_motor_rad:
            raise ValueError("missed_step_warn_motor_rad cannot exceed missed_step_fault_motor_rad")
        if not 0 <= self.missed_step_correction_rate <= 1:
            raise ValueError("missed_step_correction_rate must be between 0 and 1")
        for name in ("idle_current_ma", "hold_current_ma", "run_current_ma"):
            if int(getattr(self, name)) < 0:
                raise ValueError(f"{name} must be non-negative")
        if self.run_current_ma < self.hold_current_ma:
            raise ValueError("run_current_ma must be at least hold_current_ma")
        if self.current_downshift_delay_s < 0:
            raise ValueError("current_downshift_delay_s must be non-negative")
        if self.autotune_max_amplitude_rad <= 0:
            raise ValueError("autotune_max_amplitude_rad must be positive")
        if self.autotune_max_duration_s <= 0:
            raise ValueError("autotune_max_duration_s must be positive")
        if self.autotune_max_deflection_rad <= 0:
            raise ValueError("autotune_max_deflection_rad must be positive")

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)

    def to_json(self, path: str | Path) -> None:
        self.validate()
        Path(path).write_text(json.dumps(self.as_dict(), indent=2) + "\n", encoding="utf-8")

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "CalibrationConfig":
        normalized = dict(data)
        version = int(normalized.get("calibration_version", 1))
        if version == 1:
            normalized["calibration_version"] = CALIBRATION_SCHEMA_VERSION
            normalized.setdefault("velocity_pid_kp", 0.2)
            normalized.setdefault("velocity_pid_ki", 2.0)
            normalized.setdefault("velocity_pid_i_limit_motor_rad", 0.2)
            normalized.setdefault("torque_proxy_kp", 3.0)
            normalized.setdefault("torque_proxy_limit_rad", 0.12)
            normalized.setdefault("torque_proxy_max_motor_velocity_rad_s", 4.0)
            normalized.setdefault("torque_proxy_timeout_s", 3.0)
            normalized.setdefault("missed_step_correction_enabled", True)
            normalized.setdefault("missed_step_warn_motor_rad", 0.05)
            normalized.setdefault("missed_step_fault_motor_rad", 0.25)
            normalized.setdefault("missed_step_correction_rate", 0.25)
            normalized.setdefault("current_control_enabled", True)
            normalized.setdefault("idle_current_ma", 0)
            normalized.setdefault("hold_current_ma", 350)
            normalized.setdefault("run_current_ma", 1000)
            normalized.setdefault("current_downshift_delay_s", 0.5)
            normalized.setdefault("autotune_max_amplitude_rad", 0.4)
            normalized.setdefault("autotune_max_duration_s", 15.0)
            normalized.setdefault("autotune_max_deflection_rad", 0.25)
        known = {field.name for field in cls.__dataclass_fields__.values()}
        config = cls(**{key: value for key, value in normalized.items() if key in known})
        config.validate()
        return config

    @classmethod
    def from_json(cls, path: str | Path) -> "CalibrationConfig":
        return cls.from_dict(json.loads(Path(path).read_text(encoding="utf-8")))

    @classmethod
    def from_fit(
        cls,
        *,
        actuator_id: str,
        hardware_revision: str,
        firmware_version: str,
        motor_encoder_sign: int,
        output_encoder_sign: int,
        output_per_motor: float,
        output_offset_rad: float,
        hysteresis_rad: float,
        max_safe_velocity_rad_s: float,
        max_safe_accel_rad_s2: float,
    ) -> "CalibrationConfig":
        return cls(
            actuator_id=actuator_id,
            hardware_revision=hardware_revision,
            firmware_version=firmware_version,
            motor_encoder_sign=motor_encoder_sign,
            output_encoder_sign=output_encoder_sign,
            output_per_motor=output_per_motor,
            motor_per_output=1.0 / output_per_motor,
            output_offset_rad=output_offset_rad,
            hysteresis_rad=hysteresis_rad,
            max_safe_velocity_rad_s=max_safe_velocity_rad_s,
            max_safe_accel_rad_s2=max_safe_accel_rad_s2,
        )
