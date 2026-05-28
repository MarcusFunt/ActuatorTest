"""Binary actuator protocol definitions and codecs.

Frame layout:

    magic       2 bytes  A5 5A
    version     uint8
    type        uint8
    sequence    uint16 little-endian
    length      uint16 little-endian
    payload     length bytes
    crc         uint16 little-endian, CRC-16/CCITT-FALSE over version..payload
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, IntEnum, IntFlag
import struct
from typing import Iterable

MAGIC = b"\xA5\x5A"
PROTOCOL_VERSION = 1
MAX_PAYLOAD_SIZE = 4096
CRC16_CCITT_FALSE_POLY = 0x1021
CRC16_CCITT_FALSE_INITIAL = 0xFFFF
CRC16_CCITT_FALSE_CHECK = 0x29B1

_HEADER = struct.Struct("<BBHH")
_CRC = struct.Struct("<H")
_TELEMETRY = struct.Struct("<QIffiifffffffIB")
_MOVE_REL = struct.Struct("<fff")
_CHIRP = struct.Struct("<fffff")
FRAME_OVERHEAD_SIZE = len(MAGIC) + _HEADER.size + _CRC.size


class ProtocolError(ValueError):
    """Raised when a payload cannot be encoded or decoded."""


class PacketType(IntEnum):
    COMMAND = 1
    RESPONSE = 2
    TELEMETRY = 3
    EVENT = 4


class CommandID(IntEnum):
    PING = 1
    INFO = 2
    STREAM_ON = 3
    STREAM_OFF = 4
    SET_MODE = 5
    MOVE_REL = 6
    STOP = 7
    ESTOP = 8
    ZERO_MOTOR_ENCODER = 9
    ZERO_OUTPUT_ENCODER = 10
    GET_CONFIG = 11
    SET_CONFIG = 12
    SAVE_CONFIG = 13
    FAULTS = 14
    CLEAR_FAULTS = 15
    SELF_TEST = 16
    START_CHIRP = 17
    MOVE_OUTPUT_REL = 18


class ResponseStatus(IntEnum):
    OK = 0
    ERROR = 1
    UNSUPPORTED = 2
    FAULT = 3
    BAD_MODE = 4
    BAD_PARAM = 5
    TIMEOUT = 6


class ActuatorMode(IntEnum):
    DISABLED = 0
    CALIBRATION = 1
    OPEN_LOOP = 2
    POSITION = 3
    VELOCITY = 4
    FAULT = 255


class FaultFlags(IntFlag):
    NONE = 0
    COMMAND_TIMEOUT = 1 << 0
    OVERTRAVEL = 1 << 1
    OVERCURRENT = 1 << 2
    ENCODER_DISAGREEMENT = 1 << 3
    OVER_TEMPERATURE = 1 << 4
    ESTOP_ACTIVE = 1 << 5
    PROTOCOL_ERROR = 1 << 6


@dataclass(frozen=True)
class Frame:
    packet_type: PacketType
    sequence: int
    payload: bytes
    version: int = PROTOCOL_VERSION


@dataclass
class ParserStats:
    frames_decoded: int = 0
    bad_magic_bytes: int = 0
    crc_errors: int = 0
    length_errors: int = 0
    version_errors: int = 0
    packet_type_errors: int = 0

    def as_dict(self) -> dict[str, int]:
        return {
            "frames_decoded": self.frames_decoded,
            "bad_magic_bytes": self.bad_magic_bytes,
            "crc_errors": self.crc_errors,
            "length_errors": self.length_errors,
            "version_errors": self.version_errors,
            "packet_type_errors": self.packet_type_errors,
        }


@dataclass(frozen=True)
class TelemetryPayload:
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


@dataclass(frozen=True)
class ResponsePayload:
    command: CommandID
    status: ResponseStatus
    data: bytes = field(default_factory=bytes)


def crc16_ccitt(
    data: bytes | bytearray | memoryview,
    initial: int = CRC16_CCITT_FALSE_INITIAL,
) -> int:
    """Compute CRC-16/CCITT-FALSE.

    Parameters are poly=0x1021, init=0xFFFF, refin=false, refout=false,
    xorout=0x0000. The standard check value for b"123456789" is 0x29B1.
    """

    crc = initial & 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ CRC16_CCITT_FALSE_POLY) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def encode_frame(
    packet_type: PacketType | int,
    sequence: int,
    payload: bytes | bytearray | memoryview = b"",
) -> bytes:
    payload = bytes(payload)
    if len(payload) > MAX_PAYLOAD_SIZE:
        raise ProtocolError(f"payload too large: {len(payload)} bytes")
    if not 0 <= sequence <= 0xFFFF:
        raise ProtocolError("frame sequence must fit uint16")
    try:
        packet_type = PacketType(packet_type)
    except ValueError as exc:
        raise ProtocolError(f"unknown packet type: {packet_type}") from exc

    body = _HEADER.pack(PROTOCOL_VERSION, int(packet_type), sequence, len(payload)) + payload
    crc = crc16_ccitt(body)
    return MAGIC + body + _CRC.pack(crc)


def decode_frame(data: bytes | bytearray | memoryview) -> Frame:
    """Decode and validate one complete binary frame."""

    frame = bytes(data)
    if len(frame) < FRAME_OVERHEAD_SIZE:
        raise ProtocolError(f"frame too short: {len(frame)} bytes")
    if frame[: len(MAGIC)] != MAGIC:
        raise ProtocolError("bad frame magic")

    header_start = len(MAGIC)
    header_end = header_start + _HEADER.size
    version, raw_type, sequence, payload_len = _HEADER.unpack(frame[header_start:header_end])
    if payload_len > MAX_PAYLOAD_SIZE:
        raise ProtocolError(f"payload length too large: {payload_len} bytes")

    expected_len = FRAME_OVERHEAD_SIZE + payload_len
    if len(frame) != expected_len:
        raise ProtocolError(f"frame length mismatch: expected {expected_len} bytes, got {len(frame)}")

    payload_end = header_end + payload_len
    expected_crc = _CRC.unpack(frame[payload_end:expected_len])[0]
    actual_crc = crc16_ccitt(frame[header_start:payload_end])
    if expected_crc != actual_crc:
        raise ProtocolError(f"CRC mismatch: expected 0x{expected_crc:04X}, computed 0x{actual_crc:04X}")

    if version != PROTOCOL_VERSION:
        raise ProtocolError(f"unsupported protocol version: {version}")
    try:
        packet_type = PacketType(raw_type)
    except ValueError as exc:
        raise ProtocolError(f"unknown packet type: {raw_type}") from exc

    return Frame(
        packet_type=packet_type,
        sequence=sequence,
        payload=frame[header_end:payload_end],
        version=version,
    )


class PacketParser:
    """Incremental frame parser that recovers after corrupt bytes or CRC failures."""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self.stats = ParserStats()

    @property
    def buffered_bytes(self) -> int:
        return len(self._buffer)

    def reset(self) -> None:
        self._buffer.clear()

    def _discard_to_next_magic(self, start: int = 1) -> None:
        next_magic = self._buffer.find(MAGIC, start)
        if next_magic == -1:
            keep = 1 if self._buffer[-1:] == MAGIC[:1] else 0
            discarded = len(self._buffer) - keep
            if discarded:
                del self._buffer[:discarded]
                self.stats.bad_magic_bytes += discarded
            return
        if next_magic:
            del self._buffer[:next_magic]
            self.stats.bad_magic_bytes += next_magic

    def feed(self, data: bytes | bytearray | memoryview) -> list[Frame]:
        if data:
            self._buffer.extend(data)
        frames: list[Frame] = []

        while True:
            if len(self._buffer) < len(MAGIC):
                break

            if self._buffer[:2] != MAGIC:
                self._discard_to_next_magic()
                continue

            if len(self._buffer) < FRAME_OVERHEAD_SIZE:
                break

            version, raw_type, sequence, payload_len = _HEADER.unpack(
                self._buffer[len(MAGIC) : len(MAGIC) + _HEADER.size]
            )
            if payload_len > MAX_PAYLOAD_SIZE:
                self.stats.length_errors += 1
                self._discard_to_next_magic()
                continue

            total_len = FRAME_OVERHEAD_SIZE + payload_len
            if len(self._buffer) < total_len:
                break

            body_start = len(MAGIC)
            body_end = body_start + _HEADER.size + payload_len
            expected_crc = _CRC.unpack(self._buffer[body_end:total_len])[0]
            actual_crc = crc16_ccitt(bytes(self._buffer[body_start:body_end]))
            if expected_crc != actual_crc:
                self.stats.crc_errors += 1
                self._discard_to_next_magic()
                continue

            raw_payload = bytes(self._buffer[body_start + _HEADER.size : body_end])
            del self._buffer[:total_len]

            if version != PROTOCOL_VERSION:
                self.stats.version_errors += 1
                continue

            try:
                packet_type = PacketType(raw_type)
            except ValueError:
                self.stats.packet_type_errors += 1
                continue

            frames.append(
                Frame(
                    packet_type=packet_type,
                    sequence=sequence,
                    payload=raw_payload,
                    version=version,
                )
            )
            self.stats.frames_decoded += 1

        return frames


def encode_command_payload(command: CommandID, data: bytes = b"") -> bytes:
    return bytes([int(command)]) + data


def decode_command_payload(payload: bytes) -> tuple[CommandID, bytes]:
    if not payload:
        raise ProtocolError("empty command payload")
    try:
        command = CommandID(payload[0])
    except ValueError as exc:
        raise ProtocolError(f"unknown command id: {payload[0]}") from exc
    return command, payload[1:]


def encode_response_payload(
    command: CommandID, status: ResponseStatus = ResponseStatus.OK, data: bytes = b""
) -> bytes:
    return bytes([int(command), int(status)]) + data


def decode_response_payload(payload: bytes) -> ResponsePayload:
    if len(payload) < 2:
        raise ProtocolError("response payload too short")
    try:
        command = CommandID(payload[0])
        status = ResponseStatus(payload[1])
    except ValueError as exc:
        raise ProtocolError("unknown response command or status") from exc
    return ResponsePayload(command=command, status=status, data=payload[2:])


def encode_telemetry_payload(telemetry: TelemetryPayload) -> bytes:
    return _TELEMETRY.pack(
        telemetry.t_us,
        telemetry.seq,
        telemetry.cmd_pos,
        telemetry.cmd_vel,
        telemetry.motor_enc_raw,
        telemetry.output_enc_raw,
        telemetry.motor_rad,
        telemetry.output_rad,
        telemetry.motor_vel_rad_s,
        telemetry.output_vel_rad_s,
        telemetry.driver_current,
        telemetry.bus_voltage,
        telemetry.temperature,
        telemetry.fault_flags,
        telemetry.mode,
    )


def decode_telemetry_payload(payload: bytes) -> TelemetryPayload:
    if len(payload) != _TELEMETRY.size:
        raise ProtocolError(f"telemetry payload must be {_TELEMETRY.size} bytes, got {len(payload)}")
    values = _TELEMETRY.unpack(payload)
    return TelemetryPayload(*values)


def telemetry_payload_size() -> int:
    return _TELEMETRY.size


def pack_mode_payload(mode: ActuatorMode) -> bytes:
    return struct.pack("<B", int(mode))


def unpack_mode_payload(payload: bytes) -> ActuatorMode:
    if len(payload) != 1:
        raise ProtocolError("mode payload must be one byte")
    try:
        return ActuatorMode(payload[0])
    except ValueError as exc:
        raise ProtocolError(f"unknown actuator mode: {payload[0]}") from exc


def pack_move_rel_payload(delta_rad: float, velocity_rad_s: float, accel_rad_s2: float) -> bytes:
    return _MOVE_REL.pack(float(delta_rad), float(velocity_rad_s), float(accel_rad_s2))


def unpack_move_rel_payload(payload: bytes) -> tuple[float, float, float]:
    if len(payload) != _MOVE_REL.size:
        raise ProtocolError("MOVE_REL payload must contain delta, velocity, accel as float32")
    return _MOVE_REL.unpack(payload)


def pack_move_output_rel_payload(delta_rad: float, velocity_rad_s: float, accel_rad_s2: float) -> bytes:
    return _MOVE_REL.pack(float(delta_rad), float(velocity_rad_s), float(accel_rad_s2))


def unpack_move_output_rel_payload(payload: bytes) -> tuple[float, float, float]:
    if len(payload) != _MOVE_REL.size:
        raise ProtocolError("MOVE_OUTPUT_REL payload must contain delta, velocity, accel as float32")
    return _MOVE_REL.unpack(payload)


def pack_chirp_payload(
    amplitude_rad: float,
    start_frequency_hz: float,
    end_frequency_hz: float,
    duration_s: float,
    max_deflection_rad: float,
) -> bytes:
    return _CHIRP.pack(
        float(amplitude_rad),
        float(start_frequency_hz),
        float(end_frequency_hz),
        float(duration_s),
        float(max_deflection_rad),
    )


def unpack_chirp_payload(payload: bytes) -> tuple[float, float, float, float, float]:
    if len(payload) != _CHIRP.size:
        raise ProtocolError(
            "START_CHIRP payload must contain amplitude, start_hz, end_hz, duration, max_deflection as float32"
        )
    return _CHIRP.unpack(payload)


def pack_faults_payload(fault_flags: int | FaultFlags) -> bytes:
    return struct.pack("<I", int(fault_flags))


def unpack_faults_payload(payload: bytes) -> int:
    if len(payload) != 4:
        raise ProtocolError("fault payload must be uint32")
    return struct.unpack("<I", payload)[0]


def _pack_string(value: str) -> bytes:
    encoded = value.encode("utf-8")
    if len(encoded) > 255:
        raise ProtocolError("info string too long")
    return bytes([len(encoded)]) + encoded


def _unpack_string(payload: bytes, offset: int) -> tuple[str, int]:
    if offset >= len(payload):
        raise ProtocolError("missing string length")
    length = payload[offset]
    offset += 1
    end = offset + length
    if end > len(payload):
        raise ProtocolError("truncated string")
    return payload[offset:end].decode("utf-8"), end


def pack_info_payload(
    actuator_id: str,
    firmware_version: str,
    hardware_revision: str,
    telemetry_schema_version: int = 1,
) -> bytes:
    return (
        struct.pack("<H", int(telemetry_schema_version))
        + _pack_string(actuator_id)
        + _pack_string(firmware_version)
        + _pack_string(hardware_revision)
    )


def unpack_info_payload(payload: bytes) -> dict[str, str | int]:
    if len(payload) < 2:
        raise ProtocolError("info payload too short")
    telemetry_schema_version = struct.unpack("<H", payload[:2])[0]
    offset = 2
    actuator_id, offset = _unpack_string(payload, offset)
    firmware_version, offset = _unpack_string(payload, offset)
    hardware_revision, offset = _unpack_string(payload, offset)
    if offset != len(payload):
        raise ProtocolError("trailing bytes in info payload")
    return {
        "telemetry_schema_version": telemetry_schema_version,
        "actuator_id": actuator_id,
        "firmware_version": firmware_version,
        "hardware_revision": hardware_revision,
    }


def iter_frames(chunks: Iterable[bytes]) -> Iterable[Frame]:
    parser = PacketParser()
    for chunk in chunks:
        yield from parser.feed(chunk)
