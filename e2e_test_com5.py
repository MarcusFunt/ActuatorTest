"""End-to-end hardware test for the actuator on COM5.

Runs every test routine in sequence, saves a full session report (CSV, plots,
JSON calibration, summary), and prints a pass/fail table to stdout.

Usage:
    python e2e_test_com5.py [--port COM5] [--baud 1000000]
"""

from __future__ import annotations

import argparse
import sys
import time
import traceback
from pathlib import Path
from typing import Callable

# Make sure the src package is importable when running from the project root.
sys.path.insert(0, str(Path(__file__).parent / "src"))

from actuator_tool.actuator_data import TelemetryStore
from actuator_tool.actuator_protocol import ActuatorMode
from actuator_tool.actuator_serial import (
    ActuatorClient,
    ActuatorError,
    PySerialTransport,
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
from actuator_tool.actuator_report import (
    SessionReport,
    create_session_folder,
    plot_deflection,
    plot_ratio,
    write_summary,
)


# ---------------------------------------------------------------------------
# Conservative safety limits tuned for this actuator
# The last calibration run recommended max vel ~1.42 rad/s.
# We stay well below that for safety margin.
# ---------------------------------------------------------------------------
SAFETY = SafetyLimits(
    max_velocity_rad_s=1.5,
    max_accel_rad_s2=100.0,
    jog_step_rad=0.15,
    calibration_velocity_rad_s=0.8,
    calibration_accel_rad_s2=40.0,
    max_move_rad=10.0,
)


SEPARATOR = "=" * 70


def progress(msg: str) -> None:
    ts = time.strftime("%H:%M:%S")
    print(f"  [{ts}] {msg}")


def section(title: str) -> None:
    print(f"\n{SEPARATOR}")
    print(f"  {title}")
    print(SEPARATOR)


def result_line(name: str, passed: bool, detail: str = "") -> None:
    status = "PASS" if passed else "FAIL"
    marker = "[+]" if passed else "[-]"
    suffix = f"  -- {detail}" if detail else ""
    print(f"  {marker} {status}  {name}{suffix}")


def try_estop(client: ActuatorClient) -> None:
    """Attempt an emergency stop without raising."""
    try:
        client.estop(timeout=0.5)
    except Exception:
        pass


def connect(port: str, baud: int) -> tuple[ActuatorClient, TelemetryStore]:
    print(f"\nAvailable serial ports: {scan_serial_ports() or ['(none found)']}")
    print(f"Connecting to {port} @ {baud} baud ...")
    transport = PySerialTransport(port, baud)
    store = TelemetryStore(max_samples=500_000)
    client = ActuatorClient(transport, store)
    client.max_velocity_rad_s = SAFETY.max_velocity_rad_s
    client.max_accel_rad_s2 = SAFETY.max_accel_rad_s2
    client.connect()
    time.sleep(0.3)  # let the reader thread settle
    return client, store


def safe_stop(client: ActuatorClient) -> None:
    """Graceful stop and disable; never throws."""
    for fn in (
        lambda: client.stop(timeout=0.5),
        lambda: client.set_mode(ActuatorMode.DISABLED, timeout=0.5),
        lambda: client.stop_stream(timeout=0.5),
    ):
        try:
            fn()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Individual test wrappers – each returns (passed: bool, detail: str)
# ---------------------------------------------------------------------------

def step_detection(client: ActuatorClient, store: TelemetryStore) -> tuple[bool, dict]:
    r = run_detection(client, progress=progress)
    detail = {
        "actuator_id": r.info.actuator_id if r.info else "N/A",
        "firmware": r.info.firmware_version if r.info else "N/A",
        "hardware": r.info.hardware_revision if r.info else "N/A",
        "schema": r.info.telemetry_schema_version if r.info else "N/A",
        "message": r.message,
    }
    return r.passed, detail


def step_ping_roundtrip(client: ActuatorClient) -> tuple[bool, dict]:
    """Measure ping round-trip latency with 10 pings."""
    latencies = []
    for _ in range(10):
        t0 = time.perf_counter()
        ok = client.ping(timeout=1.0)
        dt_ms = (time.perf_counter() - t0) * 1000
        if ok:
            latencies.append(dt_ms)
    if not latencies:
        return False, {"error": "all pings failed"}
    avg = sum(latencies) / len(latencies)
    mx = max(latencies)
    detail = {"count": len(latencies), "avg_ms": round(avg, 2), "max_ms": round(mx, 2)}
    return True, detail


def step_self_test(client: ActuatorClient) -> tuple[bool, dict]:
    try:
        result = client.self_test(timeout=3.0)
        passed = result == b"OK"
        return passed, {"response": result.decode("utf-8", errors="replace")}
    except Exception as exc:
        return False, {"error": str(exc)}


def step_faults(client: ActuatorClient) -> tuple[bool, dict]:
    faults = client.faults(timeout=1.0)
    if faults:
        progress(f"Clearing active faults: {faults:#010x}")
        client.clear_faults(timeout=1.0)
        faults_after = client.faults(timeout=1.0)
        return faults_after == 0, {"faults_before": hex(faults), "faults_after": hex(faults_after)}
    return True, {"faults": "none"}


def step_get_config(client: ActuatorClient) -> tuple[bool, dict]:
    try:
        config = client.get_config(timeout=1.0)
        return True, {"keys": len(config), "output_per_motor": config.get("output_per_motor")}
    except Exception as exc:
        return False, {"error": str(exc)}


def step_encoder_sanity(client: ActuatorClient, store: TelemetryStore) -> tuple[bool, dict]:
    r = run_encoder_sanity_test(client, store, safety=SAFETY, progress=progress)
    safe_stop(client)
    detail = {
        "motor_delta_rad": round(r.motor_delta_rad, 5),
        "output_delta_rad": round(r.output_delta_rad, 5),
        "motor_noise_rad": round(r.motor_noise_rad, 6),
        "output_noise_rad": round(r.output_noise_rad, 6),
        "frozen_motor": r.frozen_motor,
        "frozen_output": r.frozen_output,
        "motor_discontinuity": r.motor_discontinuity,
        "output_discontinuity": r.output_discontinuity,
        "samples": r.sample_count,
        "warning": r.warning,
    }
    return r.pass_test, detail


def step_ratio_calibration(
    client: ActuatorClient, store: TelemetryStore, info
) -> tuple[bool, dict, CalibrationConfig | None]:
    r = run_ratio_calibration(
        client,
        store,
        actuator_info=info,
        safety=SAFETY,
        progress=progress,
        motor_sweep_rad=4.0,
        residual_threshold_rad=0.04,
    )
    safe_stop(client)
    detail = {
        "output_per_motor": round(r.fit.output_per_motor, 8),
        "motor_per_output": round(r.fit.motor_per_output, 8),
        "output_offset_rad": round(r.fit.output_offset_rad, 8),
        "residual_rms_rad": round(r.fit.residual_rms_rad, 8),
        "residual_max_abs_rad": round(r.fit.residual_max_abs_rad, 8),
        "hysteresis_rad": round(r.fit.hysteresis_rad, 8),
        "samples": r.sample_count,
        "message": r.message,
    }
    calibration = r.calibration if r.passed else None
    return r.passed, detail, calibration


def step_resonance(
    client: ActuatorClient,
    store: TelemetryStore,
    calibration: CalibrationConfig,
) -> tuple[bool, dict, CalibrationConfig]:
    r = run_resonance_test(
        client,
        store,
        calibration,
        safety=SAFETY,
        progress=progress,
        amplitude_rad=0.12,
        start_frequency_hz=0.5,
        end_frequency_hz=12.0,
        duration_s=14.0,
        max_deflection_rad=0.15,
    )
    safe_stop(client)
    a = r.analysis
    detail = {
        "peak_frequency_hz": round(a.peak_frequency_hz, 4) if a.peak_frequency_hz else None,
        "peak_gain": round(a.peak_gain, 4) if a.peak_gain else None,
        "peak_prominence_db": round(a.peak_prominence_db, 3) if a.peak_prominence_db else None,
        "bandwidth_hz": round(a.bandwidth_hz, 4) if a.bandwidth_hz else None,
        "q_factor": round(a.q_factor, 3) if a.q_factor else None,
        "rms_deflection_rad": round(a.rms_deflection_rad, 6),
        "peak_deflection_rad": round(a.peak_deflection_rad, 6),
        "sample_rate_hz": round(a.sample_rate_hz, 2),
        "samples": a.sample_count,
        "message": r.message,
    }
    return r.passed, detail, r.calibration


def step_step_response(
    client: ActuatorClient,
    store: TelemetryStore,
    calibration: CalibrationConfig | None,
) -> tuple[bool, dict]:
    r = run_step_response_test(
        client,
        store,
        safety=SAFETY,
        progress=progress,
        calibration=calibration,
        step_rad=0.15,
        settle_capture_s=8.0,
    )
    safe_stop(client)
    detail = {
        "initial_output_rad": round(r.initial_output_rad, 5),
        "final_output_rad": round(r.final_output_rad, 5),
        "commanded_step_rad": round(r.commanded_step_rad, 5),
        "rise_time_s": round(r.rise_time_s, 4) if r.rise_time_s is not None else None,
        "settling_time_s": round(r.settling_time_s, 4) if r.settling_time_s is not None else None,
        "overshoot_percent": round(r.overshoot_percent, 3),
        "undershoot_percent": round(r.undershoot_percent, 3),
        "output_lag_s": round(r.output_lag_s, 5) if r.output_lag_s is not None else None,
        "oscillation_hz": round(r.oscillation_frequency_hz, 3) if r.oscillation_frequency_hz else None,
        "damping": round(r.damping_estimate, 4) if r.damping_estimate else None,
        "stable": r.stable,
        "samples": r.sample_count,
        "warning": r.warning,
    }
    return r.pass_test, detail


def step_velocity_ramp(
    client: ActuatorClient,
    store: TelemetryStore,
    calibration: CalibrationConfig,
) -> tuple[bool, dict]:
    r = run_velocity_ramp_test(
        client,
        store,
        calibration,
        safety=SAFETY,
        progress=progress,
        sweep_rad=3.0,
    )
    safe_stop(client)
    detail = {
        "max_abs_velocity_rad_s": round(r.max_abs_velocity_rad_s, 4),
        "rms_tracking_error_rad_s": round(r.rms_velocity_tracking_error_rad_s, 5),
        "p95_tracking_error_rad_s": round(r.p95_velocity_tracking_error_rad_s, 5),
        "peak_tracking_error_rad_s": round(r.peak_velocity_tracking_error_rad_s, 5),
        "bad_tracking_fraction": round(r.bad_tracking_error_fraction, 4),
        "rms_deflection_rad": round(r.rms_deflection_rad, 6),
        "peak_deflection_rad": round(r.peak_deflection_rad, 6),
        "resonance_detected": r.resonance_detected,
        "resonance_frequency_hz": round(r.resonance_frequency_hz, 3) if r.resonance_frequency_hz else None,
        "recommended_vel_rad_s": round(r.recommended_velocity_limit_rad_s, 4),
        "recommended_accel_rad_s2": round(r.recommended_accel_limit_rad_s2, 4),
        "samples": r.sample_count,
        "warning": r.warning,
    }
    return r.pass_test, detail


def step_backlash(
    client: ActuatorClient,
    store: TelemetryStore,
    calibration: CalibrationConfig,
) -> tuple[bool, dict]:
    r = run_backlash_test(
        client,
        store,
        calibration,
        safety=SAFETY,
        progress=progress,
        reversal_rad=0.6,
        cycles=4,
    )
    safe_stop(client)
    detail = {
        "backlash_motor_rad": round(r.backlash_motor_rad, 6),
        "backlash_output_rad": round(r.backlash_output_rad, 6),
        "direction_asymmetry_rad": round(r.direction_asymmetry_rad, 6),
        "repeatability_rad": round(r.repeatability_rad, 6),
        "reversal_count": r.reversal_count,
        "samples": r.sample_count,
        "warning": r.warning,
    }
    return r.pass_test, detail


def step_compliance(
    client: ActuatorClient,
    store: TelemetryStore,
    calibration: CalibrationConfig,
) -> tuple[bool, dict]:
    """Static hold test: measure compliance/stiffness at rest."""
    r = run_compliance_test(
        client,
        store,
        calibration,
        progress=progress,
        duration_s=5.0,
        known_torque_nm=None,
    )
    detail = {
        "mean_deflection_rad": round(r.mean_deflection_rad, 6),
        "relative_compliance_rad": round(r.relative_compliance_rad, 6),
        "hysteresis_rad": round(r.hysteresis_rad, 6),
        "creep_rad": round(r.creep_rad, 6),
        "torque_nm": r.torque_nm,
        "stiffness_nm_per_rad": r.stiffness_nm_per_rad,
        "samples": r.sample_count,
        "warning": r.warning,
    }
    return r.pass_test, detail


def step_stream_statistics(client: ActuatorClient, store: TelemetryStore) -> tuple[bool, dict]:
    """Start stream, collect for 3 s, check sample rate and drop rate."""
    safe_stop(client)
    client.set_mode(ActuatorMode.CALIBRATION)
    client.start_stream()
    before = store.total_samples
    time.sleep(3.0)
    after = store.total_samples
    safe_stop(client)
    n = after - before
    stats = store.stats
    rate_hz = n / 3.0
    drop_fraction = stats.dropped_samples / max(stats.total_samples, 1)
    passed = n >= 100 and drop_fraction < 0.02
    detail = {
        "samples_in_3s": n,
        "est_rate_hz": round(rate_hz, 1),
        "total_dropped": stats.dropped_samples,
        "drop_fraction": round(drop_fraction, 5),
        "stream_pauses": stats.stream_pauses,
    }
    return passed, detail


# ---------------------------------------------------------------------------
# Main runner
# ---------------------------------------------------------------------------

def run_all(port: str, baud: int) -> int:
    results: list[tuple[str, bool, dict]] = []
    calibration: CalibrationConfig | None = None
    detected_info = None

    session = create_session_folder("reports")
    print(f"\nSession folder: {session.folder}")

    client, store = connect(port, baud)
    recorder = None

    try:
        from actuator_tool.actuator_report import CsvTelemetryRecorder, EventCsvWriter
        recorder = CsvTelemetryRecorder(session.raw_csv)
        events = EventCsvWriter(session.events_csv)
        client.add_telemetry_callback(recorder.record_sample)

        # ── 1. PING round-trip latency ──────────────────────────────────────
        section("TEST 1 — Ping round-trip latency")
        passed, detail = step_ping_roundtrip(client)
        result_line("Ping (10×)", passed, str(detail))
        results.append(("Ping latency", passed, detail))

        # ── 2. Detection (INFO) ─────────────────────────────────────────────
        section("TEST 2 — Device detection (PING + INFO)")
        passed, detail = step_detection(client, store)
        result_line("Device detection", passed, detail.get("message", ""))
        results.append(("Device detection", passed, detail))
        if passed:
            from actuator_tool.actuator_data import ActuatorInfo
            detected_info = ActuatorInfo(
                actuator_id=detail["actuator_id"],
                firmware_version=detail["firmware"],
                hardware_revision=detail["hardware"],
                telemetry_schema_version=int(detail["schema"]),
            )
            session.folder.joinpath("actuator_info.json").write_text(
                __import__("json").dumps(detected_info.as_dict(), indent=2),
                encoding="utf-8",
            )
            print(f"\n    Actuator ID  : {detail['actuator_id']}")
            print(f"    Firmware     : {detail['firmware']}")
            print(f"    Hardware rev : {detail['hardware']}")
            print(f"    Schema ver   : {detail['schema']}")
        events.write("detection", detail)

        # ── 3. Self-test ────────────────────────────────────────────────────
        section("TEST 3 — Self-test")
        passed, detail = step_self_test(client)
        result_line("Self-test", passed, str(detail))
        results.append(("Self-test", passed, detail))
        events.write("self_test", detail)

        # ── 4. Fault check ──────────────────────────────────────────────────
        section("TEST 4 — Fault register check & clear")
        passed, detail = step_faults(client)
        result_line("Fault check/clear", passed, str(detail))
        results.append(("Fault check", passed, detail))

        # ── 5. GET_CONFIG ───────────────────────────────────────────────────
        section("TEST 5 — Read device configuration")
        passed, detail = step_get_config(client)
        result_line("GET_CONFIG", passed, str(detail))
        results.append(("GET_CONFIG", passed, detail))

        # ── 6. Stream statistics ─────────────────────────────────────────────
        section("TEST 6 — Telemetry stream statistics (3 s)")
        try:
            passed, detail = step_stream_statistics(client, store)
            result_line("Stream stats", passed, str(detail))
        except Exception as exc:
            passed, detail = False, {"error": str(exc)}
            result_line("Stream stats", False, str(exc))
        results.append(("Stream stats", passed, detail))
        events.write("stream_stats", detail)

        # ── 7. Encoder sanity ────────────────────────────────────────────────
        section("TEST 7 — Encoder sanity (positive + negative sweep)")
        try:
            passed, detail = step_encoder_sanity(client, store)
            result_line("Encoder sanity", passed, detail.get("warning") or "ok")
        except Exception as exc:
            passed, detail = False, {"error": str(exc)}
            result_line("Encoder sanity", False, str(exc))
        results.append(("Encoder sanity", passed, detail))
        events.write("encoder_sanity", detail)
        for k, v in detail.items():
            print(f"    {k}: {v}")

        # ── 8. Ratio calibration ─────────────────────────────────────────────
        section("TEST 8 — Transmission ratio calibration")
        try:
            passed, detail, calibration = step_ratio_calibration(client, store, detected_info)
            result_line("Ratio calibration", passed, detail.get("message", ""))
        except Exception as exc:
            passed, detail = False, {"error": str(exc)}
            calibration = None
            result_line("Ratio calibration", False, str(exc))
        results.append(("Ratio calibration", passed, detail))
        events.write("ratio_calibration", detail)
        for k, v in detail.items():
            print(f"    {k}: {v}")

        if calibration is not None:
            calibration.to_json(session.fitted_params_json)
            # Save ratio plot
            try:
                from actuator_tool.actuator_analysis import fit_ratio
                snaps = store.snapshot()
                if snaps:
                    fit = fit_ratio(snaps[-min(3000, len(snaps)):])
                    plot_ratio(snaps[-min(3000, len(snaps)):], fit, session.ratio_plot_png)
                    progress(f"Ratio plot saved: {session.ratio_plot_png}")
                    plot_deflection(
                        snaps[-min(3000, len(snaps)):],
                        calibration.output_per_motor,
                        calibration.output_offset_rad,
                        session.deflection_plot_png,
                    )
                    progress(f"Deflection plot saved: {session.deflection_plot_png}")
            except Exception as exc:
                progress(f"Plot error (non-fatal): {exc}")

        # ── 9. Resonance sweep ───────────────────────────────────────────────
        section("TEST 9 — Resonance frequency sweep (software oscillation)")
        if calibration is None:
            progress("Skipping resonance test — no calibration available")
            results.append(("Resonance sweep", False, {"skipped": "no calibration"}))
        else:
            try:
                passed, detail, calibration = step_resonance(client, store, calibration)
                result_line("Resonance sweep", passed, detail.get("message", ""))
            except Exception as exc:
                passed, detail = False, {"error": str(exc)}
                result_line("Resonance sweep", False, str(exc))
            results.append(("Resonance sweep", passed, detail))
            events.write("resonance", detail)
            for k, v in detail.items():
                print(f"    {k}: {v}")
            if calibration is not None:
                calibration.to_json(session.fitted_params_json)

        # ── 10. Step response ────────────────────────────────────────────────
        section("TEST 10 — Step response (0.15 rad, 8 s capture)")
        try:
            passed, detail = step_step_response(client, store, calibration)
            result_line("Step response", passed, detail.get("warning") or "ok")
        except Exception as exc:
            passed, detail = False, {"error": str(exc)}
            result_line("Step response", False, str(exc))
        results.append(("Step response", passed, detail))
        events.write("step_response", detail)
        for k, v in detail.items():
            print(f"    {k}: {v}")

        # ── 11. Velocity ramp ─────────────────────────────────────────────────
        section("TEST 11 — Velocity ramp (3 rad sweep)")
        if calibration is None:
            progress("Skipping velocity ramp — no calibration available")
            results.append(("Velocity ramp", False, {"skipped": "no calibration"}))
        else:
            try:
                passed, detail = step_velocity_ramp(client, store, calibration)
                result_line("Velocity ramp", passed, detail.get("warning") or "ok")
            except Exception as exc:
                passed, detail = False, {"error": str(exc)}
                result_line("Velocity ramp", False, str(exc))
            results.append(("Velocity ramp", passed, detail))
            events.write("velocity_ramp", detail)
            for k, v in detail.items():
                print(f"    {k}: {v}")

        # ── 12. Backlash ──────────────────────────────────────────────────────
        section("TEST 12 — Backlash measurement (4 cycles, 0.6 rad)")
        if calibration is None:
            progress("Skipping backlash test — no calibration available")
            results.append(("Backlash", False, {"skipped": "no calibration"}))
        else:
            try:
                passed, detail = step_backlash(client, store, calibration)
                result_line("Backlash", passed, detail.get("warning") or "ok")
            except Exception as exc:
                passed, detail = False, {"error": str(exc)}
                result_line("Backlash", False, str(exc))
            results.append(("Backlash", passed, detail))
            events.write("backlash", detail)
            for k, v in detail.items():
                print(f"    {k}: {v}")

        # ── 13. Compliance / static stiffness ─────────────────────────────────
        section("TEST 13 — Compliance (5 s static hold)")
        if calibration is None:
            progress("Skipping compliance test — no calibration available")
            results.append(("Compliance", False, {"skipped": "no calibration"}))
        else:
            try:
                passed, detail = step_compliance(client, store, calibration)
                result_line("Compliance", passed, detail.get("warning") or "ok")
            except Exception as exc:
                passed, detail = False, {"error": str(exc)}
                result_line("Compliance", False, str(exc))
            results.append(("Compliance", passed, detail))
            events.write("compliance", detail)
            for k, v in detail.items():
                print(f"    {k}: {v}")

    except KeyboardInterrupt:
        print("\n\n  Interrupted by user — sending ESTOP …")
        try_estop(client)
    except Exception as exc:
        print(f"\n  FATAL: {exc}")
        traceback.print_exc()
        try_estop(client)
    finally:
        safe_stop(client)
        client.disconnect()
        if recorder is not None:
            recorder.close()

    # ── Summary ───────────────────────────────────────────────────────────────
    section("SUMMARY")
    passed_count = sum(1 for _, p, _ in results if p)
    total = len(results)
    for name, passed, _ in results:
        result_line(name, passed)
    print()
    print(f"  {passed_count} / {total} tests passed")

    # Write text summary
    warnings = [
        detail.get("warning", "") or detail.get("error", "") or detail.get("message", "")
        for _, passed, detail in results
        if not passed and isinstance(detail, dict)
    ]
    write_summary(
        session.summary_txt,
        info=detected_info,
        telemetry_store=store,
        ratio_fit=None,
        calibration=calibration,
        warnings=[w for w in warnings if w],
    )

    print(f"\n  Session saved to: {session.folder}")
    print(f"  Raw CSV: {session.raw_csv}")
    print(f"  Summary: {session.summary_txt}")
    if calibration is not None:
        print(f"  Calibration JSON: {session.fitted_params_json}")
    print()

    return 0 if passed_count == total else 1


def main() -> None:
    parser = argparse.ArgumentParser(description="End-to-end actuator test on a real COM port")
    parser.add_argument("--port", default="COM5", help="Serial port (default: COM5)")
    parser.add_argument("--baud", type=int, default=1_000_000, help="Baud rate (default: 1000000)")
    args = parser.parse_args()

    print(SEPARATOR)
    print("  Actuator End-to-End Test Suite")
    print(f"  Port: {args.port}   Baud: {args.baud}")
    print(f"  Safety limits: vel={SAFETY.max_velocity_rad_s} rad/s  "
          f"accel={SAFETY.max_accel_rad_s2} rad/s^2  "
          f"cal_vel={SAFETY.calibration_velocity_rad_s} rad/s")
    print(SEPARATOR)

    sys.exit(run_all(args.port, args.baud))


if __name__ == "__main__":
    main()
