import time

import pytest

from actuator_tool.actuator_data import TelemetryStore
from actuator_tool.actuator_protocol import ActuatorMode, CommandID, ResponseStatus
from actuator_tool.actuator_serial import ActuatorClient, ActuatorCommandError, CommandResponse, SimulatedTransport
from actuator_tool.actuator_tests import run_ratio_calibration, run_resonance_test
from actuator_tool.config_schema import SafetyLimits


def wait_for_samples(store: TelemetryStore, count: int, timeout_s: float = 2.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if store.total_samples >= count:
            return
        time.sleep(0.02)
    raise AssertionError(f"timed out waiting for {count} samples")


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
        result = run_resonance_test(
            client,
            store,
            calibration,
            limits,
            amplitude_rad=0.06,
            start_frequency_hz=2.0,
            end_frequency_hz=15.0,
            duration_s=5.0,
            max_deflection_rad=0.5,
        )

        assert result.analysis.peak_frequency_hz is not None
        assert abs(result.analysis.peak_frequency_hz - 8.0) < 1.0
        assert result.calibration.resonance_frequency_hz == result.analysis.peak_frequency_hz
        assert result.calibration.resonance_derating_enabled is True
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
