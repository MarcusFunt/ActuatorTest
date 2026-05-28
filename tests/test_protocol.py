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
    decode_command_payload,
    decode_frame,
    decode_telemetry_payload,
    encode_command_payload,
    encode_frame,
    encode_telemetry_payload,
    crc16_ccitt,
    pack_chirp_payload,
    pack_move_output_rel_payload,
    unpack_chirp_payload,
    unpack_move_output_rel_payload,
)


def test_firmware_payload_limit_matches_host_protocol():
    firmware = Path("firmware/ActuatorFirmware/ActuatorFirmware.ino").read_text(encoding="utf-8")
    match = re.search(r"static const uint16_t MAX_PAYLOAD_SIZE = (\d+);", firmware)

    assert match is not None
    assert int(match.group(1)) == MAX_PAYLOAD_SIZE == 4096


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


def test_chirp_payload_round_trip():
    payload = pack_chirp_payload(0.08, 0.5, 20.0, 10.0, 0.12)

    decoded = unpack_chirp_payload(payload)

    assert decoded == pytest.approx((0.08, 0.5, 20.0, 10.0, 0.12))


def test_move_output_rel_payload_round_trip():
    payload = pack_move_output_rel_payload(0.25, 1.2, 10.0)

    decoded = unpack_move_output_rel_payload(payload)

    assert decoded == pytest.approx((0.25, 1.2, 10.0))
