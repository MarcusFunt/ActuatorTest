"""Parameter schema for a measured belted actuator plant.

The plant model intentionally separates measured/calibrated parameters from the
UI/firmware calibration schema.  Values here describe the physical actuator and
its measurement/command path, so the same file can drive the standalone digital
twin, MuJoCo, or Project Chrono adapters.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
import bisect
import json
import math
from pathlib import Path
from typing import Any

PLANT_SCHEMA_VERSION = 1


def _utc_timestamp() -> str:
    return datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds")


@dataclass(frozen=True)
class TorqueSpeedPoint:
    """One measured point on the available motor-torque envelope."""

    speed_rad_s: float
    torque_nm: float

    def validate(self) -> None:
        if not math.isfinite(self.speed_rad_s) or self.speed_rad_s < 0.0:
            raise ValueError("speed_rad_s must be finite and non-negative")
        if not math.isfinite(self.torque_nm) or self.torque_nm < 0.0:
            raise ValueError("torque_nm must be finite and non-negative")


@dataclass
class EncoderModelParameters:
    """Reduced sensor model for one shaft encoder."""

    counts_per_rev: int = 4096
    noise_std_rad: float = 0.0
    bias_rad: float = 0.0
    sample_period_s: float = 0.004
    latency_s: float = 0.0
    jitter_std_s: float = 0.0

    def validate(self) -> None:
        if int(self.counts_per_rev) < 2:
            raise ValueError("counts_per_rev must be at least 2")
        for name in ("noise_std_rad", "sample_period_s", "latency_s", "jitter_std_s"):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")
        if not math.isfinite(float(self.bias_rad)):
            raise ValueError("bias_rad must be finite")

    @property
    def quantization_step_rad(self) -> float:
        return 2.0 * math.pi / float(self.counts_per_rev)


@dataclass
class ActuatorPlantParameters:
    """Compact physical model of a motor -> reduction -> elastic belt -> output.

    ``gear_ratio_motor_per_output`` is positive and defined as motor speed divided
    by output speed.  For a 3:1 reduction this value is ``3.0``.

    Friction is represented as an output-side equivalent.  Motor electrical and
    high-speed losses should therefore be captured primarily by the measured
    torque-speed envelope, while bearing/belt/load losses can be fit here.
    """

    plant_version: int = PLANT_SCHEMA_VERSION
    actuator_id: str = ""
    source: str = ""

    gear_ratio_motor_per_output: float = 3.0
    motor_inertia_kg_m2: float = 1.0e-5
    output_inertia_kg_m2: float = 1.0e-3

    belt_stiffness_nm_per_rad: float = 20.0
    belt_damping_nm_s_per_rad: float = 0.05

    coulomb_friction_nm: float = 0.0
    viscous_friction_nm_s_per_rad: float = 0.0
    static_friction_nm: float = 0.0
    stribeck_velocity_rad_s: float = 0.5
    friction_smoothing_velocity_rad_s: float = 0.01

    motor_torque_speed_map: list[TorqueSpeedPoint] = field(
        default_factory=lambda: [
            TorqueSpeedPoint(0.0, 0.45),
            TorqueSpeedPoint(50.0, 0.40),
            TorqueSpeedPoint(100.0, 0.30),
            TorqueSpeedPoint(200.0, 0.12),
        ]
    )
    torque_speed_reference_current_a: float | None = None
    torque_speed_reference_bus_voltage_v: float | None = None

    command_latency_s: float = 0.0
    command_jitter_std_s: float = 0.0

    motor_encoder: EncoderModelParameters = field(default_factory=EncoderModelParameters)
    output_encoder: EncoderModelParameters = field(default_factory=EncoderModelParameters)

    integration_step_s: float = 0.00025
    random_seed: int = 42
    identified_at: str = field(default_factory=_utc_timestamp)

    def validate(self) -> None:
        if self.plant_version != PLANT_SCHEMA_VERSION:
            raise ValueError(f"unsupported plant schema version {self.plant_version}")
        for name in (
            "gear_ratio_motor_per_output",
            "motor_inertia_kg_m2",
            "output_inertia_kg_m2",
            "belt_stiffness_nm_per_rad",
            "integration_step_s",
        ):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        for name in (
            "belt_damping_nm_s_per_rad",
            "coulomb_friction_nm",
            "viscous_friction_nm_s_per_rad",
            "static_friction_nm",
            "stribeck_velocity_rad_s",
            "friction_smoothing_velocity_rad_s",
            "command_latency_s",
            "command_jitter_std_s",
        ):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")
        if self.static_friction_nm + 1e-12 < self.coulomb_friction_nm:
            raise ValueError("static_friction_nm cannot be smaller than coulomb_friction_nm")
        if self.stribeck_velocity_rad_s <= 0.0:
            raise ValueError("stribeck_velocity_rad_s must be positive")
        if self.friction_smoothing_velocity_rad_s <= 0.0:
            raise ValueError("friction_smoothing_velocity_rad_s must be positive")
        if not self.motor_torque_speed_map:
            raise ValueError("motor_torque_speed_map must contain at least one point")
        previous_speed = -1.0
        for point in self.motor_torque_speed_map:
            point.validate()
            if point.speed_rad_s <= previous_speed:
                raise ValueError("motor_torque_speed_map speeds must be strictly increasing")
            previous_speed = point.speed_rad_s
        for name in ("torque_speed_reference_current_a", "torque_speed_reference_bus_voltage_v"):
            value = getattr(self, name)
            if value is not None and (not math.isfinite(float(value)) or float(value) <= 0.0):
                raise ValueError(f"{name} must be positive when provided")
        self.motor_encoder.validate()
        self.output_encoder.validate()

    @property
    def output_per_motor(self) -> float:
        return 1.0 / self.gear_ratio_motor_per_output

    @property
    def reflected_motor_inertia_output_side_kg_m2(self) -> float:
        ratio = self.gear_ratio_motor_per_output
        return self.motor_inertia_kg_m2 * ratio * ratio

    @property
    def relative_mode_inertia_kg_m2(self) -> float:
        """Equivalent inertia seen by the belt's relative/torsional mode."""
        jm = self.reflected_motor_inertia_output_side_kg_m2
        jo = self.output_inertia_kg_m2
        return (jm * jo) / (jm + jo)

    @property
    def predicted_relative_resonance_hz(self) -> float:
        j_eq = self.relative_mode_inertia_kg_m2
        return math.sqrt(self.belt_stiffness_nm_per_rad / j_eq) / (2.0 * math.pi)

    def torque_limit_nm(self, motor_speed_rad_s: float, current_a: float | None = None) -> float:
        """Interpolate the measured motor-side torque envelope.

        If a reference current is known, an optional current scales the envelope
        linearly.  Bus-voltage scaling is deliberately *not* guessed because the
        high-speed stepper curve is strongly driver/voltage dependent.
        """
        speed = abs(float(motor_speed_rad_s))
        points = self.motor_torque_speed_map
        speeds = [point.speed_rad_s for point in points]
        if speed <= speeds[0]:
            torque = points[0].torque_nm
        elif speed >= speeds[-1]:
            torque = points[-1].torque_nm
        else:
            index = bisect.bisect_right(speeds, speed)
            lo = points[index - 1]
            hi = points[index]
            fraction = (speed - lo.speed_rad_s) / (hi.speed_rad_s - lo.speed_rad_s)
            torque = lo.torque_nm + fraction * (hi.torque_nm - lo.torque_nm)
        if current_a is not None and self.torque_speed_reference_current_a:
            torque *= max(0.0, float(current_a)) / self.torque_speed_reference_current_a
        return max(0.0, float(torque))

    def output_friction_torque_nm(self, output_speed_rad_s: float) -> float:
        """Return a signed friction torque opposing output motion."""
        velocity = float(output_speed_rad_s)
        abs_velocity = abs(velocity)
        smooth_sign = math.tanh(velocity / self.friction_smoothing_velocity_rad_s)
        stribeck = self.coulomb_friction_nm + (
            self.static_friction_nm - self.coulomb_friction_nm
        ) * math.exp(-((abs_velocity / self.stribeck_velocity_rad_s) ** 2))
        return smooth_sign * stribeck + self.viscous_friction_nm_s_per_rad * velocity

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)

    def to_json(self, path: str | Path) -> None:
        self.validate()
        Path(path).write_text(json.dumps(self.as_dict(), indent=2) + "\n", encoding="utf-8")

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "ActuatorPlantParameters":
        normalized = dict(data)
        points = normalized.get("motor_torque_speed_map")
        if points is not None:
            normalized["motor_torque_speed_map"] = [
                point if isinstance(point, TorqueSpeedPoint) else TorqueSpeedPoint(**point)
                for point in points
            ]
        for key in ("motor_encoder", "output_encoder"):
            value = normalized.get(key)
            if value is not None and not isinstance(value, EncoderModelParameters):
                normalized[key] = EncoderModelParameters(**value)
        known = {field.name for field in cls.__dataclass_fields__.values()}
        params = cls(**{key: value for key, value in normalized.items() if key in known})
        params.validate()
        return params

    @classmethod
    def from_json(cls, path: str | Path) -> "ActuatorPlantParameters":
        return cls.from_dict(json.loads(Path(path).read_text(encoding="utf-8")))
