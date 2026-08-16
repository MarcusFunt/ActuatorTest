"""Non-serializable actuator session state shared by Reflex pages."""

from __future__ import annotations

import threading
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
from datetime import datetime

from actuator_tool.actuator_analysis import RatioFitResult, ResonanceResult
from actuator_tool.actuator_data import ActuatorInfo, TelemetryStore
from actuator_tool.actuator_report import SessionReport
from actuator_tool.actuator_serial import ActuatorClient, ActuatorError
from actuator_tool.config_schema import CalibrationConfig, SafetyLimits


@dataclass
class BackendCtx:
    """Single owner for a bench connection, report session, and lifecycle state."""

    store: TelemetryStore = field(default_factory=TelemetryStore)
    client: ActuatorClient | None = None
    info: ActuatorInfo = field(default_factory=ActuatorInfo)
    safety: SafetyLimits = field(default_factory=SafetyLimits)
    calibration: CalibrationConfig = field(default_factory=CalibrationConfig)
    ratio_fit: RatioFitResult | None = None
    resonance_result: ResonanceResult | None = None
    report: SessionReport | None = None
    connected: bool = False
    lifecycle: str = "idle"
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

    def set_lifecycle(self, state: str) -> None:
        with self._lock:
            self.lifecycle = state

    def mark_fault(self, message: str) -> None:
        self.set_lifecycle("faulted")
        self.set_status(message)

    def add_log(self, kind: str, tag: str, message: str) -> None:
        with self._lock:
            self._log_seq += 1
            self.logs.append(
                {
                    "id": str(self._log_seq),
                    "time": datetime.now().strftime("%H:%M:%S"),
                    "kind": kind.lower(),
                    "class_name": f"log-tag {kind.lower()}",
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


ctx = BackendCtx()
# UI sequences are serialized; the client also serializes protocol exchanges.
executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="actuator")
