from pathlib import Path
import re

import pytest

from actuator_tool.actuator_protocol import (
    CommandID,
    MAX_PAYLOAD_SIZE,
    PacketParser,
    PacketType,
    ProtocolError,
    TelemetryPayload,
    pack_autotune_control_payload,
    decode_command_payload,
    decode_frame,
    decode_telemetry_payload,
    encode_command_payload,
    encode_frame,
    encode_telemetry_payload,
    crc16_ccitt,
    pack_chirp_payload,
    pack_move_output_rel_payload,
    pack_position_target_payload,
    pack_torque_proxy_target_payload,
    pack_velocity_target_payload,
    telemetry_payload_v2_size,
    unpack_autotune_control_payload,
    unpack_chirp_payload,
    unpack_move_output_rel_payload,
    unpack_position_target_payload,
    unpack_torque_proxy_target_payload,
    unpack_velocity_target_payload,
)
from actuator_tool.actuator_serial import ActuatorClient, SimulatedTransport
from actuator_tool.config_schema import SafetyLimits


def test_firmware_payload_limit_matches_host_protocol():
    firmware = Path("firmware/ActuatorFirmware/ActuatorFirmware.ino").read_text(encoding="utf-8")
    match = re.search(r"static const uint16_t MAX_PAYLOAD_SIZE = (\d+);", firmware)

    assert match is not None
    assert int(match.group(1)) == MAX_PAYLOAD_SIZE == 4096


def test_aggressive_motion_defaults_match_firmware_contract():
    firmware = Path("firmware/ActuatorFirmware/ActuatorFirmware.ino").read_text(encoding="utf-8")
    limits = SafetyLimits()
    client = ActuatorClient(SimulatedTransport())

    current_match = re.search(r"#define MOTOR_RMS_CURRENT_MA\s+(\d+)", firmware)
    velocity_match = re.search(r"#define MAX_VELOCITY_RAD_S\s+([0-9.]+)f", firmware)
    accel_match = re.search(r"#define MAX_ACCEL_RAD_S2\s+([0-9.]+)f", firmware)
    move_match = re.search(r"#define MAX_MOVE_RAD\s+([0-9.]+)f", firmware)
    step_rate_match = re.search(r"#define MAX_STEP_RATE_SPS\s+([0-9.]+)f", firmware)
    min_interval_match = re.search(r"#define MIN_STEP_INTERVAL_US\s+(\d+)UL", firmware)
    telemetry_match = re.search(r"#define TELEMETRY_HZ\s+(\d+)UL", firmware)

    assert current_match is not None
    assert velocity_match is not None
    assert accel_match is not None
    assert move_match is not None
    assert step_rate_match is not None
    assert min_interval_match is not None
    assert telemetry_match is not None
    assert int(current_match.group(1)) == 1000
    assert float(velocity_match.group(1)) == limits.max_velocity_rad_s == client.max_velocity_rad_s == 40.0
    assert float(accel_match.group(1)) == limits.max_accel_rad_s2 == client.max_accel_rad_s2 == 1000.0
    assert float(move_match.group(1)) == limits.max_move_rad == 60.0
    assert float(step_rate_match.group(1)) >= 60_000.0
    assert int(min_interval_match.group(1)) <= 17
    assert int(telemetry_match.group(1)) >= 500


def test_crc16_ccitt_false_standard_check_value():
    assert crc16_ccitt(b"123456789") == 0x29B1


def test_encoded_ping_frame_has_stable_wire_bytes():
    payload = encode_command_payload(CommandID.PING)

    frame_bytes = encode_frame(PacketType.COMMAND, 7, payload)

    assert frame_bytes == bytes.fromhex("a5 5a 01 01 07 00 01 00 01 ca 4c")


def test_frame_round_trip_command_payload():
    payload = encode_command_payload(CommandID.PING)
    frame_bytes = encode_frame(PacketType.COMMAND, 7, payload)

    frames = PacketParser().feed(frame_bytes)

    assert len(frames) == 1
    assert frames[0].packet_type == PacketType.COMMAND
    assert frames[0].sequence == 7
    assert decode_command_payload(frames[0].payload)[0] == CommandID.PING


def test_decode_frame_rejects_crc_mismatch():
    bad = bytearray(encode_frame(PacketType.COMMAND, 1, encode_command_payload(CommandID.PING)))
    bad[-1] ^= 0xFF

    with pytest.raises(ProtocolError, match="CRC mismatch"):
        decode_frame(bytes(bad))


def test_decode_frame_rejects_trailing_bytes():
    frame = encode_frame(PacketType.COMMAND, 1, encode_command_payload(CommandID.PING))

    with pytest.raises(ProtocolError, match="frame length mismatch"):
        decode_frame(frame + b"\x00")


def test_parser_recovers_from_garbage_and_multiple_frames():
    first = encode_frame(PacketType.COMMAND, 1, encode_command_payload(CommandID.PING))
    second = encode_frame(PacketType.COMMAND, 2, encode_command_payload(CommandID.INFO))
    parser = PacketParser()

    frames = parser.feed(b"garbage" + first + second)

    assert parser.stats.bad_magic_bytes == len(b"garbage")
    assert [frame.sequence for frame in frames] == [1, 2]


def test_parser_waits_for_truncated_frame():
    frame = encode_frame(PacketType.COMMAND, 3, encode_command_payload(CommandID.INFO))
    parser = PacketParser()

    assert parser.feed(frame[:4]) == []
    frames = parser.feed(frame[4:])

    assert len(frames) == 1
    assert frames[0].sequence == 3


def test_parser_counts_crc_error_and_recovers():
    bad = bytearray(encode_frame(PacketType.COMMAND, 1, encode_command_payload(CommandID.PING)))
    bad[-1] ^= 0xFF
    good = encode_frame(PacketType.COMMAND, 2, encode_command_payload(CommandID.INFO))
    parser = PacketParser()

    frames = parser.feed(bytes(bad) + good)

    assert parser.stats.crc_errors == 1
    assert len(frames) == 1
    assert frames[0].sequence == 2


def test_telemetry_payload_round_trip():
    payload = TelemetryPayload(
        t_us=123456,
        seq=42,
        cmd_pos=1.0,
        cmd_vel=2.0,
        motor_enc_raw=100,
        output_enc_raw=25,
        motor_rad=1.5,
        output_rad=0.4,
        motor_vel_rad_s=3.0,
        output_vel_rad_s=0.75,
        driver_current=0.8,
        bus_voltage=24.0,
        temperature=31.5,
        fault_flags=0,
        mode=1,
    )

    decoded = decode_telemetry_payload(encode_telemetry_payload(payload))

    assert decoded.t_us == payload.t_us
    assert decoded.seq == payload.seq
    assert decoded.motor_enc_raw == payload.motor_enc_raw
    assert decoded.mode == payload.mode
    assert abs(decoded.output_rad - payload.output_rad) < 1e-6


def test_telemetry_v2_payload_round_trip_keeps_v1_compatible_decode():
    payload = TelemetryPayload(
        t_us=123456,
        seq=42,
        cmd_pos=1.0,
        cmd_vel=2.0,
        motor_enc_raw=100,
        output_enc_raw=25,
        motor_rad=1.5,
        output_rad=0.4,
        motor_vel_rad_s=3.0,
        output_vel_rad_s=0.75,
        driver_current=0.8,
        bus_voltage=24.0,
        temperature=31.5,
        fault_flags=0,
        mode=1,
        output_target_rad=0.5,
        torque_proxy_rad=-0.02,
        motor_slip_rad=0.01,
        commanded_current=0.35,
        control_state=2,
        telemetry_schema_version=2,
    )

    encoded = encode_telemetry_payload(payload)
    decoded = decode_telemetry_payload(encoded)

    assert len(encoded) == telemetry_payload_v2_size()
    assert decoded.telemetry_schema_version == 2
    assert decoded.output_target_rad == pytest.approx(0.5)
    assert decoded.torque_proxy_rad == pytest.approx(-0.02)
    assert decoded.motor_slip_rad == pytest.approx(0.01)
    assert decoded.commanded_current == pytest.approx(0.35)
    assert decoded.control_state == 2


def test_chirp_payload_round_trip():
    payload = pack_chirp_payload(0.08, 0.5, 20.0, 10.0, 0.12)

    decoded = unpack_chirp_payload(payload)

    assert decoded == pytest.approx((0.08, 0.5, 20.0, 10.0, 0.12))


def test_move_output_rel_payload_round_trip():
    payload = pack_move_output_rel_payload(0.25, 1.2, 10.0)

    decoded = unpack_move_output_rel_payload(payload)

    assert decoded == pytest.approx((0.25, 1.2, 10.0))


def test_production_control_payload_round_trips():
    assert CommandID.SET_POSITION_TARGET == 19
    assert CommandID.SET_VELOCITY_TARGET == 20
    assert CommandID.SET_TORQUE_PROXY_TARGET == 21
    assert CommandID.AUTOTUNE_CONTROL == 22
    assert CommandID.GET_CONTROL_STATUS == 23

    position = pack_position_target_payload(1.25, 2.5, 30.0, relative=True)
    velocity = pack_velocity_target_payload(-1.5, 20.0)
    torque = pack_torque_proxy_target_payload(0.03, 4.0, 0.5, 1.2)
    autotune = pack_autotune_control_payload(3, 0.05, 1.2, 0.3, 0.12)

    assert unpack_position_target_payload(position) == pytest.approx((1.25, 2.5, 30.0, 1))
    assert unpack_velocity_target_payload(velocity) == pytest.approx((-1.5, 20.0))
    assert unpack_torque_proxy_target_payload(torque) == pytest.approx((0.03, 4.0, 0.5, 1.2))
    assert unpack_autotune_control_payload(autotune) == pytest.approx((3, 0.05, 1.2, 0.3, 0.12))
