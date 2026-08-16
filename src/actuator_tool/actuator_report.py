"""Session folders, raw CSV recording, plots, JSON parameters, and summaries."""

from __future__ import annotations

import csv
import hashlib
import importlib.metadata
from dataclasses import dataclass, field
from datetime import datetime
import json
from pathlib import Path
import platform
import subprocess
import sys
import threading
from typing import Any, Iterable

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from .actuator_analysis import RatioFitResult, ResonanceResult, compute_deflection, predicted_output
from .actuator_data import ActuatorInfo, TelemetrySample, TelemetryStore
from .config_schema import CalibrationConfig, SafetyLimits


RAW_FIELDNAMES = [
    "sample_index",
    "pc_time_s",
    "t_us",
    "seq",
    "cmd_pos",
    "cmd_vel",
    "motor_enc_raw",
    "output_enc_raw",
    "motor_rad",
    "output_rad",
    "motor_vel_rad_s",
    "output_vel_rad_s",
    "driver_current",
    "bus_voltage",
    "temperature",
    "fault_flags",
    "mode",
    "mode_name",
    "output_target_rad",
    "torque_proxy_rad",
    "motor_slip_rad",
    "commanded_current",
    "control_state",
    "telemetry_schema_version",
]


@dataclass
class SessionArtifacts:
    folder: Path
    raw_csv: Path
    events_csv: Path
    fitted_params_json: Path
    actuator_info_json: Path
    ratio_plot_png: Path
    deflection_plot_png: Path
    resonance_plot_png: Path
    summary_txt: Path
    notes_md: Path
    run_manifest_json: Path

    def as_dict(self) -> dict[str, str]:
        return {key: str(value) for key, value in self.__dict__.items()}


def create_session_folder(root: str | Path = "reports") -> SessionArtifacts:
    base = Path(root)
    timestamp = datetime.now().strftime("%Y-%m-%d_%H%M%S")
    folder = base / f"actuator_calibration_{timestamp}"
    suffix = 1
    while folder.exists():
        folder = base / f"actuator_calibration_{timestamp}_{suffix}"
        suffix += 1
    folder.mkdir(parents=True, exist_ok=False)
    return SessionArtifacts(
        folder=folder,
        raw_csv=folder / "raw_stream.csv",
        events_csv=folder / "events.csv",
        fitted_params_json=folder / "fitted_params.json",
        actuator_info_json=folder / "actuator_info.json",
        ratio_plot_png=folder / "ratio_plot.png",
        deflection_plot_png=folder / "deflection_plot.png",
        resonance_plot_png=folder / "resonance_plot.png",
        summary_txt=folder / "summary.txt",
        notes_md=folder / "notes.md",
        run_manifest_json=folder / "run_manifest.json",
    )


class CsvTelemetryRecorder:
    """Thread-safe raw telemetry recorder. Attach `record_sample` as a telemetry callback."""

    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()
        self._file = self.path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=RAW_FIELDNAMES)
        self._writer.writeheader()
        self._closed = False

    def record_sample(self, sample: TelemetrySample) -> None:
        with self._lock:
            if self._closed:
                return
            self._writer.writerow(sample.csv_row())

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._file.flush()
            self._file.close()
            self._closed = True

    def __enter__(self) -> "CsvTelemetryRecorder":
        return self

    def __exit__(self, *_exc) -> None:
        self.close()


class EventCsvWriter:
    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()
        self._file = self.path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=["pc_time", "event", "details"])
        self._writer.writeheader()
        self._closed = False

    def write(self, event: str, details: str | dict[str, Any] = "") -> None:
        detail_text = json.dumps(details, sort_keys=True) if isinstance(details, dict) else str(details)
        with self._lock:
            if self._closed:
                return
            self._writer.writerow(
                {
                    "pc_time": datetime.now().isoformat(timespec="seconds"),
                    "event": event,
                    "details": detail_text,
                }
            )
            self._file.flush()

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._file.flush()
            self._file.close()
            self._closed = True


@dataclass
class SessionReport:
    artifacts: SessionArtifacts = field(default_factory=create_session_folder)
    recorder: CsvTelemetryRecorder = field(init=False)
    events: EventCsvWriter = field(init=False)

    def __post_init__(self) -> None:
        self.recorder = CsvTelemetryRecorder(self.artifacts.raw_csv)
        self.events = EventCsvWriter(self.artifacts.events_csv)
        self.artifacts.notes_md.write_text("", encoding="utf-8")

    def write_run_manifest(
        self,
        *,
        connection: dict[str, Any],
        info: ActuatorInfo | None,
        safety: SafetyLimits | None,
        calibration: CalibrationConfig | None,
    ) -> str:
        """Write a session-start provenance manifest and return its content hash."""
        config = {
            "safety": safety.__dict__ if safety is not None else {},
            "calibration": calibration.as_dict() if calibration is not None else {},
        }
        manifest = {
            "schema_version": 1,
            "created_at": datetime.now().astimezone().isoformat(timespec="seconds"),
            "git_revision": _git_revision(),
            "host": {
                "python": sys.version,
                "platform": platform.platform(),
                "packages": _package_versions(),
            },
            "connection": connection,
            "actuator": info.as_dict() if info is not None else {},
            "configuration": config,
            "configuration_sha256": _sha256_json(config),
        }
        manifest["manifest_sha256"] = _sha256_json(manifest)
        self.artifacts.run_manifest_json.write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        return str(manifest["manifest_sha256"])

    def close(self) -> None:
        self.recorder.close()
        self.events.close()

    def save_actuator_info(self, info: ActuatorInfo) -> None:
        self.artifacts.actuator_info_json.write_text(
            json.dumps(info.as_dict(), indent=2) + "\n",
            encoding="utf-8",
        )

    def save_calibration(self, calibration: CalibrationConfig) -> None:
        calibration.to_json(self.artifacts.fitted_params_json)

    def save_notes(self, notes: str) -> None:
        self.artifacts.notes_md.write_text(notes, encoding="utf-8")


def _times(samples: list[TelemetrySample]) -> list[float]:
    if not samples:
        return []
    base = samples[0].t_us
    return [(sample.t_us - base) / 1_000_000.0 for sample in samples]


def plot_ratio(samples: Iterable[TelemetrySample], fit: RatioFitResult, path: str | Path) -> None:
    sample_list = list(samples)
    if not sample_list:
        return
    motor = [sample.motor_rad for sample in sample_list]
    output = [sample.output_rad for sample in sample_list]
    fitted = [fit.output_per_motor * value + fit.output_offset_rad for value in motor]

    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(motor, output, ".", markersize=2, label="measured output")
    ax.plot(motor, fitted, "-", linewidth=1.5, label="linear fit")
    ax.set_xlabel("Motor angle [rad]")
    ax.set_ylabel("Output angle [rad]")
    ax.set_title("Transmission ratio fit")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def plot_deflection(
    samples: Iterable[TelemetrySample],
    output_per_motor: float,
    output_offset_rad: float,
    path: str | Path,
) -> None:
    sample_list = list(samples)
    if not sample_list:
        return
    times = _times(sample_list)
    predicted = predicted_output(sample_list, output_per_motor, output_offset_rad)
    deflection = compute_deflection(sample_list, output_per_motor, output_offset_rad)

    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    axes[0].plot(times, [sample.motor_rad for sample in sample_list], label="motor")
    axes[0].plot(times, [sample.output_rad for sample in sample_list], label="output")
    axes[0].plot(times, predicted, label="predicted output")
    axes[0].set_ylabel("Angle [rad]")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()
    axes[1].plot(times, deflection, color="#d62728", linewidth=1.4, label="deflection")
    axes[1].set_xlabel("Time [s]")
    axes[1].set_ylabel("Deflection [rad]")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()
    fig.suptitle("Transmission deflection")
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def plot_resonance(
    samples: Iterable[TelemetrySample],
    output_per_motor: float,
    output_offset_rad: float,
    result: ResonanceResult,
    path: str | Path,
) -> None:
    sample_list = list(samples)
    if not sample_list:
        return
    times = _times(sample_list)
    deflection = compute_deflection(sample_list, output_per_motor, output_offset_rad)

    fig, axes = plt.subplots(2, 1, figsize=(10, 7))
    axes[0].plot(times, [sample.motor_rad for sample in sample_list], label="motor")
    axes[0].plot(times, [sample.output_rad for sample in sample_list], label="output")
    axes[0].set_xlabel("Time [s]")
    axes[0].set_ylabel("Angle [rad]")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()

    axes[1].plot(times, deflection, color="#d62728", linewidth=1.2, label="deflection")
    if result.peak_frequency_hz is not None:
        axes[1].set_title(f"Resonance peak: {result.peak_frequency_hz:.2f} Hz")
    axes[1].set_xlabel("Time [s]")
    axes[1].set_ylabel("Deflection [rad]")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def write_summary(
    path: str | Path,
    *,
    info: ActuatorInfo | None,
    telemetry_store: TelemetryStore,
    ratio_fit: RatioFitResult | None,
    calibration: CalibrationConfig | None,
    warnings: Iterable[str] = (),
    manifest_path: str | Path | None = None,
) -> None:
    stats = telemetry_store.stats
    resolved_manifest = Path(manifest_path) if manifest_path is not None else Path(path).parent / "run_manifest.json"
    manifest_hash = ""
    if resolved_manifest.exists():
        try:
            manifest_hash = str(json.loads(resolved_manifest.read_text(encoding="utf-8")).get("manifest_sha256", ""))
        except (OSError, json.JSONDecodeError):
            manifest_hash = "unreadable"
    lines = [
        "Actuator calibration summary",
        "============================",
        f"Generated: {datetime.now().isoformat(timespec='seconds')}",
        "",
        f"Actuator ID: {info.actuator_id if info else ''}",
        f"Hardware revision: {info.hardware_revision if info else ''}",
        f"Firmware version: {info.firmware_version if info else ''}",
        f"Telemetry schema: {info.telemetry_schema_version if info else ''}",
        f"Run manifest: {resolved_manifest}",
        f"Run manifest SHA-256: {manifest_hash}",
        "",
        f"Raw sample count: {stats.total_samples}",
        f"Dropped sample count: {stats.dropped_samples}",
        f"Stream pauses: {stats.stream_pauses}",
        "",
    ]
    if ratio_fit is not None:
        lines.extend(
            [
                f"Output per motor: {ratio_fit.output_per_motor:.8f}",
                f"Motor per output: {ratio_fit.motor_per_output:.8f}",
                f"Output offset rad: {ratio_fit.output_offset_rad:.8f}",
                f"Fit RMS rad: {ratio_fit.residual_rms_rad:.8f}",
                f"Fit max abs rad: {ratio_fit.residual_max_abs_rad:.8f}",
                f"Hysteresis rad: {ratio_fit.hysteresis_rad:.8f}",
                f"Ratio pass: {ratio_fit.pass_fit}",
                "",
            ]
        )
    if calibration is not None:
        lines.extend(
            [
                f"Recommended max velocity rad/s: {calibration.max_safe_velocity_rad_s:.4f}",
                f"Recommended max accel rad/s^2: {calibration.max_safe_accel_rad_s2:.4f}",
                f"Resonance frequency Hz: {calibration.resonance_frequency_hz if calibration.resonance_frequency_hz is not None else ''}",
                "",
            ]
        )
    warning_list = [warning for warning in warnings if warning]
    lines.append("Warnings:")
    if warning_list:
        lines.extend(f"- {warning}" for warning in warning_list)
    else:
        lines.append("- none")
    lines.append("")
    Path(path).write_text("\n".join(lines), encoding="utf-8")


def _sha256_json(value: Any) -> str:
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":"), default=str).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _git_revision() -> str:
    root = Path(__file__).resolve().parents[2]
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=root, text=True, stderr=subprocess.DEVNULL
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def _package_versions() -> dict[str, str]:
    names = ("actuator-bench-tool", "bokeh", "matplotlib", "numpy", "pyserial", "reflex", "scipy")
    versions: dict[str, str] = {}
    for name in names:
        try:
            versions[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            versions[name] = "not-installed"
    return versions
