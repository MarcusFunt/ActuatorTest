"""Telemetry data structures, ring buffers, and timestamp handling."""

from __future__ import annotations

from collections import deque
from dataclasses import asdict, dataclass, field
import threading
import time
from typing import Generic, Iterable, TypeVar

from .actuator_protocol import ActuatorMode, FaultFlags, TelemetryPayload

T = TypeVar("T")


@dataclass
class ActuatorInfo:
    actuator_id: str = ""
    firmware_version: str = ""
    hardware_revision: str = ""
    telemetry_schema_version: int = 1

    @classmethod
    def from_dict(cls, data: dict[str, str | int]) -> "ActuatorInfo":
        return cls(
            actuator_id=str(data.get("actuator_id", "")),
            firmware_version=str(data.get("firmware_version", "")),
            hardware_revision=str(data.get("hardware_revision", "")),
            telemetry_schema_version=int(data.get("telemetry_schema_version", 1)),
        )

    def as_dict(self) -> dict[str, str | int]:
        return asdict(self)


@dataclass
class TelemetrySample:
    t_us: int
    seq: int
    cmd_pos: float
    cmd_vel: float
    motor_enc_raw: int
    output_enc_raw: int
    motor_rad: float
    output_rad: float
    motor_vel_rad_s: float
    output_vel_rad_s: float
    driver_current: float
    bus_voltage: float
    temperature: float
    fault_flags: int
    mode: int
    pc_time_s: float = field(default_factory=time.time)
    sample_index: int = 0

    @classmethod
    def from_payload(
        cls, payload: TelemetryPayload, pc_time_s: float | None = None
    ) -> "TelemetrySample":
        return cls(
            t_us=int(payload.t_us),
            seq=int(payload.seq),
            cmd_pos=float(payload.cmd_pos),
            cmd_vel=float(payload.cmd_vel),
            motor_enc_raw=int(payload.motor_enc_raw),
            output_enc_raw=int(payload.output_enc_raw),
            motor_rad=float(payload.motor_rad),
            output_rad=float(payload.output_rad),
            motor_vel_rad_s=float(payload.motor_vel_rad_s),
            output_vel_rad_s=float(payload.output_vel_rad_s),
            driver_current=float(payload.driver_current),
            bus_voltage=float(payload.bus_voltage),
            temperature=float(payload.temperature),
            fault_flags=int(payload.fault_flags),
            mode=int(payload.mode),
            pc_time_s=time.time() if pc_time_s is None else float(pc_time_s),
        )

    def actuator_time_s(self, base_t_us: int | None = None) -> float:
        base = self.t_us if base_t_us is None else base_t_us
        return (self.t_us - base) / 1_000_000.0

    @property
    def mode_name(self) -> str:
        try:
            return ActuatorMode(self.mode).name
        except ValueError:
            return f"UNKNOWN_{self.mode}"

    @property
    def fault_names(self) -> str:
        faults = FaultFlags(self.fault_flags)
        if faults == FaultFlags.NONE:
            return "NONE"
        return "|".join(flag.name for flag in FaultFlags if flag in faults and flag != FaultFlags.NONE)

    def csv_row(self) -> dict[str, int | float | str]:
        return {
            "sample_index": self.sample_index,
            "pc_time_s": self.pc_time_s,
            "t_us": self.t_us,
            "seq": self.seq,
            "cmd_pos": self.cmd_pos,
            "cmd_vel": self.cmd_vel,
            "motor_enc_raw": self.motor_enc_raw,
            "output_enc_raw": self.output_enc_raw,
            "motor_rad": self.motor_rad,
            "output_rad": self.output_rad,
            "motor_vel_rad_s": self.motor_vel_rad_s,
            "output_vel_rad_s": self.output_vel_rad_s,
            "driver_current": self.driver_current,
            "bus_voltage": self.bus_voltage,
            "temperature": self.temperature,
            "fault_flags": self.fault_flags,
            "mode": self.mode,
            "mode_name": self.mode_name,
        }


class RingBuffer(Generic[T]):
    def __init__(self, maxlen: int) -> None:
        self._items: deque[T] = deque(maxlen=maxlen)
        self._lock = threading.RLock()

    def append(self, item: T) -> None:
        with self._lock:
            self._items.append(item)

    def clear(self) -> None:
        with self._lock:
            self._items.clear()

    def snapshot(self, limit: int | None = None) -> list[T]:
        with self._lock:
            items = list(self._items)
        if limit is not None and limit >= 0:
            return items[-limit:]
        return items

    def __len__(self) -> int:
        with self._lock:
            return len(self._items)


@dataclass
class TelemetryStats:
    total_samples: int = 0
    dropped_samples: int = 0
    stream_pauses: int = 0
    last_seq: int | None = None
    last_t_us: int | None = None
    first_t_us: int | None = None
    last_pc_time_s: float | None = None

    def as_dict(self) -> dict[str, int | float | None]:
        return asdict(self)


class TelemetryStore:
    """Thread-safe full-rate recent telemetry storage with sequence/timing diagnostics."""

    def __init__(self, max_samples: int = 200_000, stream_pause_us: int = 500_000) -> None:
        self._buffer: RingBuffer[TelemetrySample] = RingBuffer(max_samples)
        self._lock = threading.RLock()
        self._stats = TelemetryStats()
        self._stream_pause_us = int(stream_pause_us)

    def add(self, sample: TelemetrySample) -> None:
        with self._lock:
            if self._stats.last_seq is not None:
                diff = (sample.seq - self._stats.last_seq) & 0xFFFFFFFF
                if diff > 1 and diff < 0x80000000:
                    self._stats.dropped_samples += diff - 1

            if self._stats.last_t_us is not None:
                gap = sample.t_us - self._stats.last_t_us
                if gap > self._stream_pause_us:
                    self._stats.stream_pauses += 1

            self._stats.total_samples += 1
            sample.sample_index = self._stats.total_samples
            self._stats.last_seq = sample.seq
            self._stats.last_t_us = sample.t_us
            self._stats.last_pc_time_s = sample.pc_time_s
            if self._stats.first_t_us is None:
                self._stats.first_t_us = sample.t_us

        self._buffer.append(sample)

    def extend(self, samples: Iterable[TelemetrySample]) -> None:
        for sample in samples:
            self.add(sample)

    def clear(self) -> None:
        with self._lock:
            self._stats = TelemetryStats()
        self._buffer.clear()

    def snapshot(self, limit: int | None = None) -> list[TelemetrySample]:
        return self._buffer.snapshot(limit)

    def samples_since(self, sample_index: int) -> list[TelemetrySample]:
        return [sample for sample in self.snapshot() if sample.sample_index > sample_index]

    def latest(self) -> TelemetrySample | None:
        samples = self.snapshot(1)
        return samples[-1] if samples else None

    @property
    def stats(self) -> TelemetryStats:
        with self._lock:
            return TelemetryStats(**asdict(self._stats))

    @property
    def total_samples(self) -> int:
        return self.stats.total_samples

    def relative_times(self, samples: Iterable[TelemetrySample]) -> list[float]:
        sample_list = list(samples)
        if not sample_list:
            return []
        base = sample_list[0].t_us
        return [(sample.t_us - base) / 1_000_000.0 for sample in sample_list]
