import inspect
import math
import time

import pytest

from actuator_tool.actuator_data import ActuatorInfo, TelemetryStore
from actuator_tool.actuator_protocol import ActuatorMode, CommandID, FaultFlags, ResponseStatus
from actuator_tool.actuator_serial import (
    ActuatorClient,
    ActuatorCommandError,
    ActuatorTimeoutError,
    CommandResponse,
    SimulatedTransport,
)
from actuator_tool.actuator_tests import (
    _abort_on_faults,
    run_detection,
    run_ratio_calibration,
    run_resonance_test,
    run_step_response_test,
)
from actuator_tool.config_schema import SafetyLimits


def wait_for_samples(store: TelemetryStore, count: int, timeout_s: float = 2.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if store.total_samples >= count:
            return
        time.sleep(0.02)
    raise AssertionError(f"timed out waiting for {count} samples")


def wait_for(predicate, timeout_s: float = 2.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(0.02)
    raise AssertionError("timed out waiting for condition")


def test_resonance_defaults_are_aggressive():
    runner_defaults = inspect.signature(run_resonance_test).parameters
    client_defaults = inspect.signature(ActuatorClient.start_chirp).parameters

    assert runner_defaults["amplitude_rad"].default >= 0.18
    assert runner_defaults["start_frequency_hz"].default <= 0.8
    assert runner_defaults["end_frequency_hz"].default == 70.0
    assert runner_defaults["duration_s"].default <= 12.0
    assert runner_defaults["max_deflection_rad"].default >= 0.25
    assert client_defaults["amplitude_rad"].default == runner_defaults["amplitude_rad"].default
    assert client_defaults["end_frequency_hz"].default == runner_defaults["end_frequency_hz"].default


def test_characterization_defaults_match_production_targets():
    ratio_defaults = inspect.signature(run_ratio_calibration).parameters
    step_defaults = inspect.signature(run_step_response_test).parameters

    assert ratio_defaults["motor_sweep_rad"].default == pytest.approx(8.0 * math.pi)
    assert step_defaults["settle_capture_s"].default == 10.0


def test_detection_accepts_telemetry_schema_v2():
    store = TelemetryStore()
    client = ActuatorClient(SimulatedTransport(), store)
    client.connect()
    try:
        result = run_detection(client)
    finally:
        client.disconnect()

    assert result.passed
    assert result.info is not None
    assert result.info.telemetry_schema_version == 2


def test_detection_rejects_unknown_telemetry_schema():
    class Client:
        def ping(self, timeout=1.0):
            return True

        def info(self, timeout=1.0):
            return ActuatorInfo(
                actuator_id="future",
                firmware_version="future",
                hardware_revision="future",
                telemetry_schema_version=99,
            )

    result = run_detection(Client())

    assert not result.passed
    assert result.message == "unsupported telemetry schema version"


def test_abort_on_faults_retries_transient_timeout():
    class Client:
        def __init__(self):
            self.calls = 0
            self.timeouts = []

        def faults(self, timeout=0.0):
            self.calls += 1
            self.timeouts.append(timeout)
            if self.calls == 1:
                raise ActuatorTimeoutError("transient timeout")
            return 0

    client = Client()

    _abort_on_faults(client)

    assert client.calls == 2
    assert client.timeouts == [1.5, 1.5]


def test_simulator_connection_and_motion_flow():
    store = TelemetryStore()
    client = ActuatorClient(SimulatedTransport(sample_hz=80.0), store)
    client.connect()
    try:
        assert client.ping()
        info = client.info()
        assert info.actuator_id == "sim_actuator_001"
        client.set_mode(ActuatorMode.CALIBRATION)
        client.start_stream()
        wait_for_samples(store, 5)
        client.move_rel(0.5, 2.0, 10.0)
        wait_for_samples(store, 20)
        assert store.latest() is not None
        assert store.latest().mode == int(ActuatorMode.CALIBRATION)
        client.stop()
    finally:
        client.disconnect()


def test_ratio_calibration_against_simulator():
    store = TelemetryStore()
    client = ActuatorClient(SimulatedTransport(sample_hz=120.0), store)
    client.connect()
    try:
        info = client.info()
        limits = SafetyLimits(calibration_velocity_rad_s=3.0, calibration_accel_rad_s2=20.0)
        result = run_ratio_calibration(
            client,
            store,
            info,
            limits,
            motor_sweep_rad=4.0,
            residual_threshold_rad=0.06,
        )

        assert result.sample_count > 50
        assert abs(result.fit.output_per_motor - 0.25) < 0.02
        assert abs(result.fit.output_offset_rad - 0.0132) < 0.03
    finally:
        client.disconnect()


def test_client_wait_response_requires_matching_sequence():
    client = ActuatorClient(SimulatedTransport())
    client._responses.put(
        CommandResponse(
            command=CommandID.PING,
            status=ResponseStatus.OK,
            data=b"stale",
            frame_sequence=2,
        )
    )
    client._responses.put(
        CommandResponse(
            command=CommandID.PING,
            status=ResponseStatus.OK,
            data=b"PONG",
            frame_sequence=3,
        )
    )

    response = client._wait_response(CommandID.PING, 3, timeout=0.2)

    assert response.data == b"PONG"
    stale = client._responses.get_nowait()
    assert stale.frame_sequence == 2


def test_resonance_test_against_resonant_simulator():
    store = TelemetryStore()
    client = ActuatorClient(SimulatedTransport(sample_hz=250.0, resonance_frequency_hz=8.0), store)
    client.connect()
    try:
        limits = SafetyLimits(max_velocity_rad_s=20.0, calibration_velocity_rad_s=3.0)
        calibration = run_ratio_calibration(
            client,
            store,
            client.info(),
            limits,
            motor_sweep_rad=2.0,
            residual_threshold_rad=0.2,
        ).calibration
        result = run_resonance_test(
            client,
            store,
            calibration,
            limits,
            amplitude_rad=0.06,
            start_frequency_hz=2.0,
            end_frequency_hz=15.0,
            duration_s=6.0,
            max_deflection_rad=0.5,
        )

        assert result.analysis.peak_frequency_hz is not None
        assert abs(result.analysis.peak_frequency_hz - 8.0) < 1.0
        assert result.calibration.resonance_frequency_hz == result.analysis.peak_frequency_hz
        assert result.calibration.resonance_derating_enabled is True
    finally:
        client.disconnect()


def test_resonant_simulator_output_stays_bounded_after_scheduler_stall():
    transport = SimulatedTransport(sample_hz=200.0, resonance_frequency_hz=8.0)
    client = ActuatorClient(transport, TelemetryStore())
    client.connect()
    try:
        client.set_mode(ActuatorMode.CALIBRATION)
        with transport._lock:
            transport._motor_rad = 0.4
            transport._target_motor_rad = 0.4
            transport._base_target_motor_rad = 0.4
            transport._output_rad = transport.output_offset_rad
            transport._output_vel = 0.0
        transport._update_motion(0.08)
        with transport._lock:
            deflection = transport._output_rad - (
                transport.output_per_motor * transport._motor_rad + transport.output_offset_rad
            )
        assert math.isfinite(deflection)
        assert abs(deflection) < 0.25
    finally:
        client.disconnect()


def test_resonance_test_reports_abort_threshold_failure():
    store = TelemetryStore()
    client = ActuatorClient(SimulatedTransport(sample_hz=200.0, resonance_frequency_hz=8.0), store)
    client.connect()
    try:
        limits = SafetyLimits(max_velocity_rad_s=20.0, calibration_velocity_rad_s=3.0)
        calibration = run_ratio_calibration(
            client,
            store,
            client.info(),
            limits,
            motor_sweep_rad=2.0,
            residual_threshold_rad=0.2,
        ).calibration

        try:
            result = run_resonance_test(
                client,
                store,
                calibration,
                limits,
                amplitude_rad=0.08,
                start_frequency_hz=2.0,
                end_frequency_hz=15.0,
                duration_s=3.0,
                max_deflection_rad=0.0005,
            )
        except (ActuatorCommandError, ValueError):
            return
        assert not result.passed
    finally:
        client.disconnect()


def test_resonance_test_returns_structured_failure_when_analysis_rejects_data(monkeypatch):
    store = TelemetryStore()
    client = ActuatorClient(SimulatedTransport(sample_hz=200.0), store)
    client.connect()
    try:
        limits = SafetyLimits(max_velocity_rad_s=20.0, calibration_velocity_rad_s=3.0)
        calibration = run_ratio_calibration(
            client,
            store,
            client.info(),
            limits,
            motor_sweep_rad=2.0,
            residual_threshold_rad=0.2,
        ).calibration

        def reject_resonance(*_args, **_kwargs):
            raise ValueError("resonance peak is not prominent enough")

        monkeypatch.setattr("actuator_tool.actuator_tests.analyze_resonance", reject_resonance)
        result = run_resonance_test(
            client,
            store,
            calibration,
            limits,
            amplitude_rad=0.06,
            start_frequency_hz=2.0,
            end_frequency_hz=15.0,
            duration_s=3.0,
            max_deflection_rad=0.5,
        )

        assert not result.passed
        assert not result.analysis.pass_test
        assert result.analysis.peak_frequency_hz is None
        assert result.sample_count == result.analysis.sample_count
        assert result.calibration.resonance_frequency_hz is None
        assert result.calibration.resonance_derating_enabled is False
        assert result.message
    finally:
        client.disconnect()


def test_output_move_requires_position_mode_and_pid_config_round_trips():
    store = TelemetryStore()
    client = ActuatorClient(SimulatedTransport(sample_hz=100.0), store)
    client.connect()
    try:
        with pytest.raises(ActuatorCommandError):
            client.move_output_rel(0.1, 1.0, 5.0)

        client.set_config("pid_enabled", True)
        client.set_config("pid_kp", 0.4)
        client.set_config("pid_ki", 0.05)
        client.set_config("pid_kd", 0.01)
        client.set_config("pid_i_limit_motor_rad", 0.02)
        client.set_config("pid_output_limit_motor_rad", 0.12)
        client.set_config("backlash_motor_rad", 0.03)
        client.set_config("backlash_comp_enabled", True)
        client.set_config("resonance_frequency_hz", 8.0)
        client.set_config("resonance_derating_enabled", True)
        config = client.get_config()

        assert config["pid_enabled"] is True
        assert config["pid_kp"] == pytest.approx(0.4)
        assert config["pid_i_limit_motor_rad"] == pytest.approx(0.02)
        assert config["backlash_comp_enabled"] is True
        assert config["resonance_frequency_hz"] == pytest.approx(8.0)

        client.set_mode(ActuatorMode.POSITION)
        client.start_stream()
        client.move_output_rel(0.1, 1.0, 5.0)
        wait_for_samples(store, 10)
        assert store.latest() is not None
        assert store.latest().mode == int(ActuatorMode.POSITION)
    finally:
        client.disconnect()


def test_config_sanitization_matches_firmware_contract():
    client = ActuatorClient(SimulatedTransport())
    client.connect()
    try:
        with pytest.raises(ActuatorCommandError):
            client.set_config("unknown_config_key", 1.0)
        with pytest.raises(ActuatorCommandError):
            client.set_config("output_per_motor", 0.0)
        with pytest.raises(ActuatorCommandError):
            client.set_config("pid_enabled", 1)

        client.set_config("pid_kp", -2.0)
        client.set_config("pid_ki", float("nan"))
        client.set_config("pid_i_limit_motor_rad", -12.0)
        client.set_config("resonance_frequency_hz", None)
        config = client.get_config()

        assert config["pid_kp"] == pytest.approx(0.0)
        assert config["pid_ki"] == pytest.approx(0.0)
        assert config["pid_i_limit_motor_rad"] == pytest.approx(10.0)
        assert config["resonance_frequency_hz"] == pytest.approx(0.0)
    finally:
        client.disconnect()


def test_resonance_derating_slows_output_space_moves():
    store = TelemetryStore()
    client = ActuatorClient(SimulatedTransport(sample_hz=300.0), store)
    client.max_velocity_rad_s = 20.0
    client.connect()
    try:
        client.set_mode(ActuatorMode.POSITION)
        client.start_stream()

        start = store.total_samples
        client.move_output_rel(0.5, 4.0, 20.0)
        time.sleep(0.35)
        unbounded = max(abs(sample.motor_vel_rad_s) for sample in store.samples_since(start))

        client.set_config("resonance_frequency_hz", 8.0)
        client.set_config("resonance_derating_enabled", True)
        start = store.total_samples
        client.move_output_rel(-0.5, 4.0, 20.0)
        time.sleep(0.45)
        derated = max(abs(sample.motor_vel_rad_s) for sample in store.samples_since(start))

        assert unbounded > 12.0
        assert derated < unbounded * 0.75
        assert derated == pytest.approx(8.0, abs=1.0)
    finally:
        client.disconnect()


def test_position_target_supports_absolute_and_relative_output_targets():
    store = TelemetryStore()
    client = ActuatorClient(SimulatedTransport(sample_hz=150.0), store)
    client.connect()
    try:
        client.set_mode(ActuatorMode.POSITION)
        client.start_stream()
        client.set_position_target(0.20, 2.0, 20.0)
        sample = wait_for(
            lambda: store.latest() if store.latest() and abs(store.latest().output_rad - 0.20) < 0.04 else None,
            timeout_s=2.0,
        )
        assert sample.output_target_rad == pytest.approx(0.20, abs=0.05)

        client.set_position_target(0.10, 2.0, 20.0, relative=True)
        sample = wait_for(
            lambda: store.latest() if store.latest() and abs(store.latest().output_rad - 0.30) < 0.05 else None,
            timeout_s=2.0,
        )
        assert sample.mode == int(ActuatorMode.POSITION)
    finally:
        client.disconnect()


def test_velocity_target_streams_output_velocity_then_holds_zero():
    store = TelemetryStore()
    client = ActuatorClient(SimulatedTransport(sample_hz=150.0), store)
    client.connect()
    try:
        client.set_mode(ActuatorMode.VELOCITY)
        client.start_stream()
        client.set_velocity_target(1.0, 25.0)
        moving = wait_for(
            lambda: store.latest() if store.latest() and store.latest().output_vel_rad_s > 0.3 else None,
            timeout_s=1.5,
        )
        assert moving.mode == int(ActuatorMode.VELOCITY)

        client.set_velocity_target(0.0, 25.0)
        stopped = wait_for(
            lambda: store.latest()
            if store.latest()
            and abs(store.latest().motor_vel_rad_s) < 0.05
            and abs(store.latest().output_vel_rad_s) < 0.25
            else None,
            timeout_s=1.5,
        )
        assert stopped.output_vel_rad_s == pytest.approx(0.0, abs=0.25)
    finally:
        client.disconnect()


def test_torque_proxy_target_uses_deflection_sign_and_reports_status():
    store = TelemetryStore()
    client = ActuatorClient(SimulatedTransport(sample_hz=200.0), store)
    client.connect()
    try:
        client.set_mode(ActuatorMode.TORQUE_PROXY)
        client.start_stream()
        client.set_torque_proxy_target(0.04, 2.0, 2.0, 1.0)
        sample = wait_for(
            lambda: store.latest() if store.latest() and store.latest().motor_vel_rad_s < -0.02 else None,
            timeout_s=1.0,
        )
        status = client.get_control_status()

        assert sample.mode == int(ActuatorMode.TORQUE_PROXY)
        assert status["torque_proxy_target_rad"] == pytest.approx(0.04)
        assert status["last_control_fault"] == ""
    finally:
        client.disconnect()


def test_missed_step_correction_and_fault_thresholds():
    transport = SimulatedTransport(sample_hz=200.0)
    store = TelemetryStore()
    client = ActuatorClient(transport, store)
    client.connect()
    try:
        client.set_config("missed_step_warn_motor_rad", 0.02)
        client.set_config("missed_step_fault_motor_rad", 0.20)
        client.set_mode(ActuatorMode.POSITION)
        client.start_stream()
        client.set_position_target(0.20, 2.0, 20.0)
        wait_for_samples(store, 5)

        transport.inject_motor_slip(0.08)
        status = wait_for(
            lambda: client.get_control_status()
            if abs(client.get_control_status()["motor_slip_rad"]) < 0.08
            else None,
            timeout_s=1.0,
        )
        assert abs(status["motor_slip_rad"]) < 0.08
        assert status["fault_flags"] == 0

        transport.inject_motor_slip(0.5)
        wait_for(lambda: client.faults() & int(FaultFlags.MISSED_STEP), timeout_s=1.0)
        assert client.faults() & int(FaultFlags.MISSED_STEP)
    finally:
        client.disconnect()


def test_current_scheduler_downshifts_after_motion():
    store = TelemetryStore()
    client = ActuatorClient(SimulatedTransport(sample_hz=200.0), store)
    client.connect()
    try:
        client.set_config("current_downshift_delay_s", 0.05)
        client.set_config("hold_current_ma", 250)
        client.set_mode(ActuatorMode.POSITION)
        client.start_stream()
        client.set_position_target(0.05, 1.0, 20.0)
        run_current = wait_for(
            lambda: store.latest() if store.latest() and store.latest().commanded_current >= 0.9 else None,
            timeout_s=1.0,
        )
        assert run_current.commanded_current == pytest.approx(1.0, abs=0.01)

        held = wait_for(
            lambda: store.latest() if store.latest() and store.latest().commanded_current <= 0.3 else None,
            timeout_s=2.0,
        )
        assert held.commanded_current == pytest.approx(0.25, abs=0.05)
    finally:
        client.disconnect()


def test_autotune_updates_gains_in_ram_and_reports_success():
    client = ActuatorClient(SimulatedTransport(sample_hz=200.0))
    client.connect()
    try:
        client.set_mode(ActuatorMode.POSITION)
        client.autotune_control(3, 0.05, 1.2, 0.05, 1.0)
        status = wait_for(
            lambda: client.get_control_status()
            if client.get_control_status()["autotune_state"] == 2
            else None,
            timeout_s=1.0,
        )
        config = client.get_config()

        assert status["autotune_state"] == 2
        assert config["pid_enabled"] is True
        assert config["pid_kp"] > 0
        assert config["velocity_pid_kp"] > 0
    finally:
        client.disconnect()
