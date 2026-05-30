"""Serial connection, simulated transport, packet parsing, and actuator commands."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import queue
import random
import struct
import threading
import time
from typing import Callable, Protocol

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover - dependency is declared, this keeps imports graceful.
    serial = None
    list_ports = None

from .actuator_data import ActuatorInfo, TelemetrySample, TelemetryStore
from .actuator_protocol import (
    ActuatorMode,
    CommandID,
    FaultFlags,
    Frame,
    PacketParser,
    PacketType,
    ProtocolError,
    ResponsePayload,
    ResponseStatus,
    TelemetryPayload,
    decode_command_payload,
    decode_response_payload,
    decode_telemetry_payload,
    encode_command_payload,
    encode_frame,
    encode_response_payload,
    encode_telemetry_payload,
    pack_chirp_payload,
    pack_faults_payload,
    pack_info_payload,
    pack_mode_payload,
    pack_move_output_rel_payload,
    pack_move_rel_payload,
    pack_autotune_control_payload,
    pack_position_target_payload,
    pack_torque_proxy_target_payload,
    pack_velocity_target_payload,
    unpack_chirp_payload,
    unpack_faults_payload,
    unpack_info_payload,
    unpack_mode_payload,
    unpack_move_output_rel_payload,
    unpack_move_rel_payload,
    unpack_autotune_control_payload,
    unpack_position_target_payload,
    unpack_torque_proxy_target_payload,
    unpack_velocity_target_payload,
)


class ActuatorError(RuntimeError):
    """Base error for actuator client failures."""


class ActuatorTimeoutError(ActuatorError):
    """Raised when the actuator does not respond before the command timeout."""


class ActuatorCommandError(ActuatorError):
    """Raised when the actuator returns a non-OK command status."""


class Transport(Protocol):
    def open(self) -> None: ...

    def close(self) -> None: ...

    def write(self, data: bytes) -> int: ...

    def read(self, size: int = 4096, timeout: float = 0.05) -> bytes: ...

    @property
    def is_open(self) -> bool: ...


@dataclass(frozen=True)
class CommandResponse:
    command: CommandID
    status: ResponseStatus
    data: bytes
    frame_sequence: int


class PySerialTransport:
    """pyserial-backed transport for real USB serial/UART-to-USB actuators."""

    def __init__(self, port: str, baudrate: int = 1_000_000) -> None:
        if serial is None:
            raise RuntimeError("pyserial is not installed")
        self.port = port
        self.baudrate = baudrate
        self._serial = None

    def open(self) -> None:
        self._serial = serial.Serial(self.port, self.baudrate, timeout=0.02, write_timeout=1.0)

    def close(self) -> None:
        if self._serial is not None:
            self._serial.close()
            self._serial = None

    def write(self, data: bytes) -> int:
        if self._serial is None:
            raise RuntimeError("serial port is not open")
        return int(self._serial.write(data))

    def read(self, size: int = 4096, timeout: float = 0.05) -> bytes:
        serial_obj = self._serial
        if serial_obj is None:
            return b""
        old_timeout = None
        try:
            old_timeout = serial_obj.timeout
            serial_obj.timeout = timeout
            return bytes(serial_obj.read(size))
        except (OSError, serial.SerialException):
            return b""
        finally:
            if old_timeout is not None and serial_obj is self._serial and serial_obj.is_open:
                try:
                    serial_obj.timeout = old_timeout
                except (OSError, serial.SerialException):
                    pass

    @property
    def is_open(self) -> bool:
        return bool(self._serial is not None and self._serial.is_open)


class SimulatedTransport:
    """In-process actuator simulator that speaks the MVP binary protocol."""

    def __init__(
        self,
        *,
        output_per_motor: float = 0.25,
        output_offset_rad: float = 0.0132,
        sample_hz: float = 100.0,
        resonance_frequency_hz: float | None = None,
        resonance_damping_ratio: float = 0.05,
    ) -> None:
        self.output_per_motor = float(output_per_motor)
        self.output_offset_rad = float(output_offset_rad)
        self.sample_hz = float(sample_hz)
        self.resonance_frequency_hz = None if resonance_frequency_hz is None else float(resonance_frequency_hz)
        self.resonance_damping_ratio = float(resonance_damping_ratio)
        self.actuator_id = "sim_actuator_001"
        self.firmware_version = "sim-0.1.0"
        self.hardware_revision = "sim_rev_a"
        self._rx_parser = PacketParser()
        self._tx_queue: queue.Queue[bytes] = queue.Queue()
        self._lock = threading.RLock()
        self._running = threading.Event()
        self._thread: threading.Thread | None = None
        self._start_perf = time.perf_counter()
        self._last_perf = self._start_perf
        self._streaming = False
        self._mode = ActuatorMode.DISABLED
        self._fault_flags = FaultFlags.NONE
        self._frame_sequence = 0
        self._telemetry_sequence = 0
        self._motor_rad = 0.0
        self._output_rad = self.output_offset_rad
        self._motor_vel = 0.0
        self._output_vel = 0.0
        self._target_motor_rad = 0.0
        self._base_target_motor_rad = 0.0
        self._output_target_rad = self._output_rad
        self._velocity_target_output_rad_s = 0.0
        self._move_velocity = 1.0
        self._move_accel = 20.0
        self._chirp_active = False
        self._chirp_started_at = 0.0
        self._chirp_center_motor_rad = 0.0
        self._chirp_amplitude_rad = 0.18
        self._chirp_start_hz = 0.8
        self._chirp_end_hz = 75.0
        self._chirp_duration_s = 12.0
        self._chirp_max_deflection_rad = 0.25
        self.pid_enabled = False
        self.pid_kp = 0.0
        self.pid_ki = 0.0
        self.pid_kd = 0.0
        self.pid_i_limit_motor_rad = 0.05
        self.pid_output_limit_motor_rad = 0.25
        self.velocity_pid_kp = 0.2
        self.velocity_pid_ki = 2.0
        self.velocity_pid_i_limit_motor_rad = 0.2
        self.torque_proxy_kp = 3.0
        self.torque_proxy_limit_rad = 0.12
        self.torque_proxy_max_motor_velocity_rad_s = 4.0
        self.torque_proxy_timeout_s = 3.0
        self.missed_step_correction_enabled = True
        self.missed_step_warn_motor_rad = 0.05
        self.missed_step_fault_motor_rad = 0.25
        self.missed_step_correction_rate = 0.25
        self.current_control_enabled = True
        self.idle_current_ma = 0
        self.hold_current_ma = 350
        self.run_current_ma = 1000
        self.current_downshift_delay_s = 0.5
        self.autotune_max_amplitude_rad = 0.4
        self.autotune_max_duration_s = 15.0
        self.autotune_max_deflection_rad = 0.25
        self.backlash_motor_rad = 0.0
        self.backlash_comp_enabled = False
        self.resonance_derating_enabled = False
        self._pid_integral = 0.0
        self._pid_last_error = 0.0
        self._velocity_integral = 0.0
        self._last_position_direction = 0
        self._torque_proxy_target_rad = 0.0
        self._torque_proxy_max_velocity = 0.0
        self._torque_proxy_max_excursion = 0.0
        self._torque_proxy_started_at = 0.0
        self._torque_proxy_start_motor_rad = 0.0
        self._torque_proxy_deadline = float("inf")
        self._issued_motor_rad = 0.0
        self._motor_encoder_bias_rad = 0.0
        self._motor_slip_rad = 0.0
        self._commanded_current_a = 0.0
        self._last_motion_time = time.perf_counter()
        self._autotune_state = 0
        self._autotune_loop_selector = 0
        self._autotune_started_at = 0.0
        self._autotune_deadline = 0.0
        self._autotune_amplitude_rad = 0.0
        self._last_control_fault = ""
        self._rng = random.Random(42)

    def open(self) -> None:
        if self._running.is_set():
            return
        self._running.set()
        self._start_perf = time.perf_counter()
        self._last_perf = self._start_perf
        self._thread = threading.Thread(target=self._run, name="simulated-actuator", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._running.clear()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None

    def write(self, data: bytes) -> int:
        frames = self._rx_parser.feed(data)
        for frame in frames:
            if frame.packet_type == PacketType.COMMAND:
                self._handle_command(frame)
        return len(data)

    def read(self, size: int = 4096, timeout: float = 0.05) -> bytes:
        if not self._running.is_set() and self._tx_queue.empty():
            return b""
        try:
            first = self._tx_queue.get(timeout=timeout)
        except queue.Empty:
            return b""
        chunks = [first]
        total = len(first)
        while total < size:
            try:
                chunk = self._tx_queue.get_nowait()
            except queue.Empty:
                break
            chunks.append(chunk)
            total += len(chunk)
        return b"".join(chunks)[:size]

    @property
    def is_open(self) -> bool:
        return self._running.is_set()

    def inject_motor_slip(self, slip_rad: float) -> None:
        """Test hook: offset measured motor encoder angle from issued steps."""
        with self._lock:
            self._motor_encoder_bias_rad -= float(slip_rad)

    def _measured_motor_rad(self) -> float:
        return self._motor_rad + self._motor_encoder_bias_rad

    def _torque_proxy_rad(self) -> float:
        return self._output_rad - (self.output_per_motor * self._measured_motor_rad() + self.output_offset_rad)

    def _update_motor_slip(self) -> None:
        self._motor_slip_rad = self._issued_motor_rad - self._measured_motor_rad()
        if abs(self._motor_slip_rad) >= self.missed_step_fault_motor_rad:
            self._fault_flags |= FaultFlags.MISSED_STEP
            self._last_control_fault = "MISSED_STEP"
            self._mode = ActuatorMode.FAULT
            self._motor_vel = 0.0
            return
        if self.missed_step_correction_enabled and abs(self._motor_slip_rad) >= self.missed_step_warn_motor_rad:
            correction = self._motor_slip_rad * self.missed_step_correction_rate
            self._issued_motor_rad -= correction
            self._target_motor_rad -= correction
            self._base_target_motor_rad -= correction
            self._motor_slip_rad = self._issued_motor_rad - self._measured_motor_rad()

    def _update_current_command(self, now: float) -> None:
        if not self.current_control_enabled:
            self._commanded_current_a = self.run_current_ma / 1000.0
            return
        if self._mode in (ActuatorMode.DISABLED, ActuatorMode.FAULT) or self._fault_flags:
            self._commanded_current_a = self.idle_current_ma / 1000.0
            return
        high_load = abs(self._torque_proxy_rad()) > self.torque_proxy_limit_rad * 0.5
        moving = abs(self._motor_vel) > 1e-3 or abs(self._target_motor_rad - self._motor_rad) > 0.002
        if moving or high_load:
            self._last_motion_time = now
            self._commanded_current_a = self.run_current_ma / 1000.0
        elif now - self._last_motion_time >= self.current_downshift_delay_s:
            self._commanded_current_a = self.hold_current_ma / 1000.0
        else:
            self._commanded_current_a = self.run_current_ma / 1000.0

    @staticmethod
    def _config_number(value) -> float:
        if value is None or isinstance(value, bool):
            raise ValueError("config value must be numeric")
        return float(value)

    def _next_frame_sequence(self) -> int:
        self._frame_sequence = (self._frame_sequence + 1) & 0xFFFF
        return self._frame_sequence

    def _enqueue_frame(self, packet_type: PacketType, payload: bytes, sequence: int | None = None) -> None:
        seq = self._next_frame_sequence() if sequence is None else sequence & 0xFFFF
        self._tx_queue.put(encode_frame(packet_type, seq, payload))

    def _respond(
        self,
        command: CommandID,
        status: ResponseStatus = ResponseStatus.OK,
        data: bytes = b"",
        sequence: int | None = None,
    ) -> None:
        self._enqueue_frame(
            PacketType.RESPONSE,
            encode_response_payload(command, status, data),
            sequence=sequence,
        )

    def _handle_command(self, frame: Frame) -> None:
        try:
            command, payload = decode_command_payload(frame.payload)
        except ProtocolError:
            with self._lock:
                self._fault_flags |= FaultFlags.PROTOCOL_ERROR
            return

        try:
            self._execute_command(command, payload, frame.sequence)
        except ProtocolError:
            self._respond(command, ResponseStatus.BAD_PARAM, sequence=frame.sequence)
        except Exception as exc:  # pragma: no cover - defensive simulator boundary.
            self._respond(command, ResponseStatus.ERROR, str(exc).encode("utf-8"), frame.sequence)

    def _execute_command(self, command: CommandID, payload: bytes, sequence: int) -> None:
        if command == CommandID.PING:
            self._respond(command, data=b"PONG", sequence=sequence)
            return

        if command == CommandID.INFO:
            data = pack_info_payload(
                self.actuator_id,
                "sim-0.2.0",
                self.hardware_revision,
                telemetry_schema_version=2,
            )
            self._respond(command, data=data, sequence=sequence)
            return

        if command == CommandID.STREAM_ON:
            with self._lock:
                self._streaming = True
            self._respond(command, sequence=sequence)
            return

        if command == CommandID.STREAM_OFF:
            with self._lock:
                self._streaming = False
            self._respond(command, sequence=sequence)
            return

        if command == CommandID.SET_MODE:
            mode = unpack_mode_payload(payload)
            with self._lock:
                if mode == ActuatorMode.DISABLED:
                    self._motor_vel = 0.0
                    self._target_motor_rad = self._motor_rad
                    self._base_target_motor_rad = self._motor_rad
                    self._issued_motor_rad = self._motor_rad
                    self._velocity_target_output_rad_s = 0.0
                    self._chirp_active = False
                    self._reset_pid()
                if mode == ActuatorMode.TORQUE_PROXY:
                    self._torque_proxy_target_rad = self._torque_proxy_rad()
                    self._torque_proxy_started_at = time.perf_counter()
                    self._torque_proxy_start_motor_rad = self._motor_rad
                    self._torque_proxy_deadline = float("inf")
                    self._torque_proxy_max_velocity = self.torque_proxy_max_motor_velocity_rad_s
                    self._torque_proxy_max_excursion = float("inf")
                if mode != ActuatorMode.FAULT:
                    self._mode = mode
            self._respond(command, sequence=sequence)
            return

        if command == CommandID.MOVE_REL:
            delta_rad, velocity_rad_s, _accel_rad_s2 = unpack_move_rel_payload(payload)
            with self._lock:
                if self._mode != ActuatorMode.CALIBRATION or self._fault_flags:
                    self._respond(command, ResponseStatus.BAD_MODE, sequence=sequence)
                    return
                self._chirp_active = False
                self._target_motor_rad = self._motor_rad + float(delta_rad)
                self._base_target_motor_rad = self._target_motor_rad
                self._issued_motor_rad = self._motor_rad
                self._move_velocity = max(0.01, abs(float(velocity_rad_s)))
                self._move_accel = max(0.01, abs(float(_accel_rad_s2)))
            self._respond(command, sequence=sequence)
            return

        if command == CommandID.MOVE_OUTPUT_REL:
            delta_rad, velocity_rad_s, accel_rad_s2 = unpack_move_output_rel_payload(payload)
            with self._lock:
                if self._mode != ActuatorMode.POSITION or self._fault_flags:
                    self._respond(command, ResponseStatus.BAD_MODE, sequence=sequence)
                    return
                if abs(self.output_per_motor) < 1e-9:
                    self._respond(command, ResponseStatus.BAD_PARAM, sequence=sequence)
                    return
                self._chirp_active = False
                output_delta = float(delta_rad)
                velocity_output = max(0.01, abs(float(velocity_rad_s)))
                motor_delta = output_delta / self.output_per_motor
                direction = 1 if motor_delta >= 0.0 else -1
                if self.backlash_comp_enabled and self._last_position_direction and direction != self._last_position_direction:
                    motor_delta += math.copysign(abs(self.backlash_motor_rad), motor_delta)
                self._last_position_direction = direction
                self._output_target_rad = self._output_rad + output_delta
                self._base_target_motor_rad = self._motor_rad + motor_delta
                self._target_motor_rad = self._base_target_motor_rad
                self._issued_motor_rad = self._motor_rad
                if self.resonance_derating_enabled and self.resonance_frequency_hz:
                    min_duration = 2.0 / max(self.resonance_frequency_hz, 0.1)
                    velocity_output = min(velocity_output, max(0.01, abs(output_delta) / min_duration))
                self._move_velocity = max(0.01, velocity_output / abs(self.output_per_motor))
                self._move_accel = max(0.01, abs(float(accel_rad_s2)) / abs(self.output_per_motor))
                self._reset_pid()
            self._respond(command, sequence=sequence)
            return

        if command == CommandID.SET_POSITION_TARGET:
            target_rad, velocity_rad_s, accel_rad_s2, flags = unpack_position_target_payload(payload)
            with self._lock:
                if self._mode != ActuatorMode.POSITION or self._fault_flags:
                    self._respond(command, ResponseStatus.BAD_MODE, sequence=sequence)
                    return
                if abs(self.output_per_motor) < 1e-9:
                    self._respond(command, ResponseStatus.BAD_PARAM, sequence=sequence)
                    return
                relative = bool(flags & 0x01)
                target_output = self._output_rad + float(target_rad) if relative else float(target_rad)
                output_delta = target_output - self._output_rad
                velocity_output = max(0.01, abs(float(velocity_rad_s)))
                if self.resonance_derating_enabled and self.resonance_frequency_hz and abs(output_delta) > 1e-9:
                    min_duration = 2.0 / max(self.resonance_frequency_hz, 0.1)
                    velocity_output = min(velocity_output, max(0.01, abs(output_delta) / min_duration))
                motor_delta = output_delta / self.output_per_motor
                direction = 1 if motor_delta >= 0.0 else -1
                if self.backlash_comp_enabled and self._last_position_direction and direction != self._last_position_direction:
                    motor_delta += math.copysign(abs(self.backlash_motor_rad), motor_delta)
                self._last_position_direction = direction
                self._chirp_active = False
                self._output_target_rad = target_output
                self._base_target_motor_rad = self._motor_rad + motor_delta
                self._target_motor_rad = self._base_target_motor_rad
                self._issued_motor_rad = self._motor_rad
                self._move_velocity = max(0.01, velocity_output / abs(self.output_per_motor))
                self._move_accel = max(0.01, abs(float(accel_rad_s2)) / abs(self.output_per_motor))
                self._reset_pid()
            self._respond(command, sequence=sequence)
            return

        if command == CommandID.SET_VELOCITY_TARGET:
            output_velocity_rad_s, accel_rad_s2 = unpack_velocity_target_payload(payload)
            with self._lock:
                if self._mode != ActuatorMode.VELOCITY or self._fault_flags:
                    self._respond(command, ResponseStatus.BAD_MODE, sequence=sequence)
                    return
                if abs(self.output_per_motor) < 1e-9:
                    self._respond(command, ResponseStatus.BAD_PARAM, sequence=sequence)
                    return
                self._chirp_active = False
                self._velocity_target_output_rad_s = float(output_velocity_rad_s)
                self._move_accel = max(0.01, abs(float(accel_rad_s2)) / abs(self.output_per_motor))
                self._target_motor_rad = self._motor_rad
                self._base_target_motor_rad = self._motor_rad
                self._issued_motor_rad = self._motor_rad
                self._reset_pid()
            self._respond(command, sequence=sequence)
            return

        if command == CommandID.SET_TORQUE_PROXY_TARGET:
            target, max_vel, max_excursion, timeout_s = unpack_torque_proxy_target_payload(payload)
            with self._lock:
                if self._mode != ActuatorMode.TORQUE_PROXY or self._fault_flags:
                    self._respond(command, ResponseStatus.BAD_MODE, sequence=sequence)
                    return
                if not all(math.isfinite(v) for v in (target, max_vel, max_excursion, timeout_s)):
                    self._respond(command, ResponseStatus.BAD_PARAM, sequence=sequence)
                    return
                now = time.perf_counter()
                self._chirp_active = False
                self._torque_proxy_target_rad = max(
                    -self.torque_proxy_limit_rad,
                    min(float(target), self.torque_proxy_limit_rad),
                )
                self._torque_proxy_max_velocity = max(0.01, abs(float(max_vel)))
                self._torque_proxy_max_excursion = max(0.001, abs(float(max_excursion)))
                self._torque_proxy_started_at = now
                self._torque_proxy_start_motor_rad = self._motor_rad
                self._torque_proxy_deadline = now + max(0.05, abs(float(timeout_s)))
                self._issued_motor_rad = self._motor_rad
                self._reset_pid()
            self._respond(command, sequence=sequence)
            return

        if command == CommandID.AUTOTUNE_CONTROL:
            loop_selector, amplitude, max_velocity, duration_s, max_deflection = unpack_autotune_control_payload(payload)
            with self._lock:
                if self._mode not in (ActuatorMode.POSITION, ActuatorMode.VELOCITY) or self._fault_flags:
                    self._respond(command, ResponseStatus.BAD_MODE, sequence=sequence)
                    return
                if loop_selector not in (1, 2, 3):
                    self._respond(command, ResponseStatus.BAD_PARAM, sequence=sequence)
                    return
                now = time.perf_counter()
                self._autotune_loop_selector = int(loop_selector)
                self._autotune_amplitude_rad = max(0.001, min(abs(float(amplitude)), self.autotune_max_amplitude_rad))
                self._move_velocity = max(0.01, abs(float(max_velocity)))
                duration = max(0.05, min(abs(float(duration_s)), self.autotune_max_duration_s))
                self._chirp_active = False
                self._autotune_started_at = now
                self._autotune_deadline = now + duration
                self._autotune_state = 1
                self._last_control_fault = ""
            self._respond(command, sequence=sequence)
            return

        if command == CommandID.GET_CONTROL_STATUS:
            with self._lock:
                status = {
                    "mode": int(self._mode),
                    "mode_name": self._mode.name,
                    "target_motor_rad": self._target_motor_rad,
                    "target_output_rad": self._output_target_rad,
                    "velocity_target_output_rad_s": self._velocity_target_output_rad_s,
                    "torque_proxy_target_rad": self._torque_proxy_target_rad,
                    "torque_proxy_rad": self._torque_proxy_rad(),
                    "motor_slip_rad": self._motor_slip_rad,
                    "commanded_current_a": self._commanded_current_a,
                    "autotune_state": self._autotune_state,
                    "autotune_loop_selector": self._autotune_loop_selector,
                    "last_control_fault": self._last_control_fault,
                    "fault_flags": int(self._fault_flags),
                }
            self._respond(command, data=json.dumps(status).encode("utf-8"), sequence=sequence)
            return

        if command == CommandID.START_CHIRP:
            amplitude, start_hz, end_hz, duration_s, max_deflection = unpack_chirp_payload(payload)
            with self._lock:
                if self._mode != ActuatorMode.CALIBRATION or self._fault_flags:
                    self._respond(command, ResponseStatus.BAD_MODE, sequence=sequence)
                    return
                amplitude = max(0.001, min(abs(float(amplitude)), 0.5))
                start_hz = max(0.05, min(abs(float(start_hz)), 45.0))
                end_hz = max(start_hz, min(abs(float(end_hz)), 45.0))
                duration_s = max(1.0, min(abs(float(duration_s)), 120.0))
                self._chirp_center_motor_rad = self._motor_rad
                self._chirp_amplitude_rad = amplitude
                self._chirp_start_hz = start_hz
                self._chirp_end_hz = end_hz
                self._chirp_duration_s = duration_s
                self._chirp_max_deflection_rad = max(0.001, min(abs(float(max_deflection)), 1.0))
                self._chirp_started_at = time.perf_counter()
                self._chirp_active = True
                self._base_target_motor_rad = self._motor_rad
                self._target_motor_rad = self._motor_rad
                self._move_velocity = max(0.1, amplitude * 2.0 * math.pi * end_hz * 1.2)
                self._reset_pid()
            self._respond(command, sequence=sequence)
            return

        if command == CommandID.STOP:
            with self._lock:
                self._target_motor_rad = self._motor_rad
                self._base_target_motor_rad = self._motor_rad
                self._issued_motor_rad = self._motor_rad
                self._motor_vel = 0.0
                self._velocity_target_output_rad_s = 0.0
                self._chirp_active = False
                self._reset_pid()
            self._respond(command, sequence=sequence)
            return

        if command == CommandID.ESTOP:
            with self._lock:
                self._target_motor_rad = self._motor_rad
                self._base_target_motor_rad = self._motor_rad
                self._issued_motor_rad = self._motor_rad
                self._motor_vel = 0.0
                self._velocity_target_output_rad_s = 0.0
                self._chirp_active = False
                self._reset_pid()
                self._fault_flags |= FaultFlags.ESTOP_ACTIVE
                self._mode = ActuatorMode.FAULT
            self._respond(command, sequence=sequence)
            return

        if command == CommandID.ZERO_MOTOR_ENCODER:
            with self._lock:
                self._target_motor_rad -= self._motor_rad
                self._base_target_motor_rad -= self._motor_rad
                self._issued_motor_rad -= self._motor_rad
                self._motor_rad = 0.0
                self._motor_encoder_bias_rad = 0.0
            self._respond(command, sequence=sequence)
            return

        if command == CommandID.ZERO_OUTPUT_ENCODER:
            with self._lock:
                self._output_rad = 0.0
                self.output_offset_rad = -self.output_per_motor * self._motor_rad
            self._respond(command, sequence=sequence)
            return

        if command == CommandID.GET_CONFIG:
            config = {
                "output_per_motor": self.output_per_motor,
                "output_offset_rad": self.output_offset_rad,
                "pid_enabled": self.pid_enabled,
                "pid_kp": self.pid_kp,
                "pid_ki": self.pid_ki,
                "pid_kd": self.pid_kd,
                "pid_i_limit_motor_rad": self.pid_i_limit_motor_rad,
                "pid_output_limit_motor_rad": self.pid_output_limit_motor_rad,
                "velocity_pid_kp": self.velocity_pid_kp,
                "velocity_pid_ki": self.velocity_pid_ki,
                "velocity_pid_i_limit_motor_rad": self.velocity_pid_i_limit_motor_rad,
                "torque_proxy_kp": self.torque_proxy_kp,
                "torque_proxy_limit_rad": self.torque_proxy_limit_rad,
                "torque_proxy_max_motor_velocity_rad_s": self.torque_proxy_max_motor_velocity_rad_s,
                "torque_proxy_timeout_s": self.torque_proxy_timeout_s,
                "missed_step_correction_enabled": self.missed_step_correction_enabled,
                "missed_step_warn_motor_rad": self.missed_step_warn_motor_rad,
                "missed_step_fault_motor_rad": self.missed_step_fault_motor_rad,
                "missed_step_correction_rate": self.missed_step_correction_rate,
                "current_control_enabled": self.current_control_enabled,
                "idle_current_ma": self.idle_current_ma,
                "hold_current_ma": self.hold_current_ma,
                "run_current_ma": self.run_current_ma,
                "current_downshift_delay_s": self.current_downshift_delay_s,
                "autotune_max_amplitude_rad": self.autotune_max_amplitude_rad,
                "autotune_max_duration_s": self.autotune_max_duration_s,
                "autotune_max_deflection_rad": self.autotune_max_deflection_rad,
                "backlash_motor_rad": self.backlash_motor_rad,
                "backlash_comp_enabled": self.backlash_comp_enabled,
                "resonance_frequency_hz": self.resonance_frequency_hz,
                "resonance_derating_enabled": self.resonance_derating_enabled,
            }
            self._respond(command, data=json.dumps(config).encode("utf-8"), sequence=sequence)
            return

        if command == CommandID.SET_CONFIG:
            data = json.loads(payload.decode("utf-8"))
            key = str(data["key"])
            value = data["value"]
            with self._lock:
                try:
                    if key == "output_per_motor":
                        number = self._config_number(value)
                        if not math.isfinite(number) or abs(number) < 1e-9:
                            self._respond(command, ResponseStatus.BAD_PARAM, sequence=sequence)
                            return
                        self.output_per_motor = number
                    elif key == "output_offset_rad":
                        number = self._config_number(value)
                        if not math.isfinite(number):
                            self._respond(command, ResponseStatus.BAD_PARAM, sequence=sequence)
                            return
                        self.output_offset_rad = number
                    elif key == "pid_enabled":
                        if not isinstance(value, bool):
                            self._respond(command, ResponseStatus.BAD_PARAM, sequence=sequence)
                            return
                        self.pid_enabled = value
                        self._reset_pid()
                    elif key == "pid_kp":
                        number = self._config_number(value)
                        self.pid_kp = number if math.isfinite(number) and number >= 0.0 else 0.0
                        self._reset_pid()
                    elif key == "pid_ki":
                        number = self._config_number(value)
                        self.pid_ki = number if math.isfinite(number) and number >= 0.0 else 0.0
                        self._reset_pid()
                    elif key == "pid_kd":
                        number = self._config_number(value)
                        self.pid_kd = number if math.isfinite(number) and number >= 0.0 else 0.0
                        self._reset_pid()
                    elif key == "pid_i_limit_motor_rad":
                        number = self._config_number(value)
                        self.pid_i_limit_motor_rad = min(abs(number), 10.0) if math.isfinite(number) else 0.05
                        self._reset_pid()
                    elif key == "pid_output_limit_motor_rad":
                        number = self._config_number(value)
                        self.pid_output_limit_motor_rad = min(abs(number), 10.0) if math.isfinite(number) else 0.25
                    elif key == "velocity_pid_kp":
                        number = self._config_number(value)
                        self.velocity_pid_kp = number if math.isfinite(number) and number >= 0.0 else 0.0
                    elif key == "velocity_pid_ki":
                        number = self._config_number(value)
                        self.velocity_pid_ki = number if math.isfinite(number) and number >= 0.0 else 0.0
                    elif key == "velocity_pid_i_limit_motor_rad":
                        number = self._config_number(value)
                        self.velocity_pid_i_limit_motor_rad = min(abs(number), 10.0) if math.isfinite(number) else 0.2
                    elif key == "torque_proxy_kp":
                        number = self._config_number(value)
                        self.torque_proxy_kp = number if math.isfinite(number) and number >= 0.0 else 0.0
                    elif key == "torque_proxy_limit_rad":
                        number = self._config_number(value)
                        self.torque_proxy_limit_rad = min(max(abs(number), 0.001), 10.0) if math.isfinite(number) else 0.12
                    elif key == "torque_proxy_max_motor_velocity_rad_s":
                        number = self._config_number(value)
                        self.torque_proxy_max_motor_velocity_rad_s = (
                            min(max(abs(number), 0.01), 100.0) if math.isfinite(number) else 4.0
                        )
                    elif key == "torque_proxy_timeout_s":
                        number = self._config_number(value)
                        self.torque_proxy_timeout_s = min(max(abs(number), 0.05), 120.0) if math.isfinite(number) else 3.0
                    elif key == "missed_step_correction_enabled":
                        if not isinstance(value, bool):
                            self._respond(command, ResponseStatus.BAD_PARAM, sequence=sequence)
                            return
                        self.missed_step_correction_enabled = value
                    elif key == "missed_step_warn_motor_rad":
                        number = self._config_number(value)
                        self.missed_step_warn_motor_rad = min(abs(number), 10.0) if math.isfinite(number) else 0.05
                    elif key == "missed_step_fault_motor_rad":
                        number = self._config_number(value)
                        self.missed_step_fault_motor_rad = min(max(abs(number), 0.001), 10.0) if math.isfinite(number) else 0.25
                    elif key == "missed_step_correction_rate":
                        number = self._config_number(value)
                        self.missed_step_correction_rate = min(max(number, 0.0), 1.0) if math.isfinite(number) else 0.25
                    elif key == "current_control_enabled":
                        if not isinstance(value, bool):
                            self._respond(command, ResponseStatus.BAD_PARAM, sequence=sequence)
                            return
                        self.current_control_enabled = value
                    elif key == "idle_current_ma":
                        number = self._config_number(value)
                        self.idle_current_ma = int(min(max(abs(number), 0.0), 2000.0)) if math.isfinite(number) else 0
                    elif key == "hold_current_ma":
                        number = self._config_number(value)
                        self.hold_current_ma = int(min(max(abs(number), 0.0), 2000.0)) if math.isfinite(number) else 350
                    elif key == "run_current_ma":
                        number = self._config_number(value)
                        self.run_current_ma = int(min(max(abs(number), 1.0), 2000.0)) if math.isfinite(number) else 1000
                    elif key == "current_downshift_delay_s":
                        number = self._config_number(value)
                        self.current_downshift_delay_s = min(max(abs(number), 0.0), 30.0) if math.isfinite(number) else 0.5
                    elif key == "autotune_max_amplitude_rad":
                        number = self._config_number(value)
                        self.autotune_max_amplitude_rad = min(max(abs(number), 0.001), 10.0) if math.isfinite(number) else 0.4
                    elif key == "autotune_max_duration_s":
                        number = self._config_number(value)
                        self.autotune_max_duration_s = min(max(abs(number), 0.05), 120.0) if math.isfinite(number) else 15.0
                    elif key == "autotune_max_deflection_rad":
                        number = self._config_number(value)
                        self.autotune_max_deflection_rad = min(max(abs(number), 0.001), 10.0) if math.isfinite(number) else 0.25
                    elif key == "backlash_motor_rad":
                        number = self._config_number(value)
                        self.backlash_motor_rad = min(abs(number), 10.0) if math.isfinite(number) else 0.0
                    elif key == "backlash_comp_enabled":
                        if not isinstance(value, bool):
                            self._respond(command, ResponseStatus.BAD_PARAM, sequence=sequence)
                            return
                        self.backlash_comp_enabled = value
                    elif key == "resonance_frequency_hz":
                        if value is None:
                            self.resonance_frequency_hz = 0.0
                        else:
                            number = self._config_number(value)
                            if not math.isfinite(number):
                                self._respond(command, ResponseStatus.BAD_PARAM, sequence=sequence)
                                return
                            self.resonance_frequency_hz = min(abs(number), 100.0)
                    elif key == "resonance_derating_enabled":
                        if not isinstance(value, bool):
                            self._respond(command, ResponseStatus.BAD_PARAM, sequence=sequence)
                            return
                        self.resonance_derating_enabled = value
                    else:
                        self._respond(command, ResponseStatus.BAD_PARAM, sequence=sequence)
                        return
                except (TypeError, ValueError, OverflowError):
                    self._respond(command, ResponseStatus.BAD_PARAM, sequence=sequence)
                    return
            self._respond(command, sequence=sequence)
            return

        if command == CommandID.SAVE_CONFIG:
            self._respond(command, sequence=sequence)
            return

        if command == CommandID.FAULTS:
            with self._lock:
                faults = self._fault_flags
            self._respond(command, data=pack_faults_payload(faults), sequence=sequence)
            return

        if command == CommandID.CLEAR_FAULTS:
            with self._lock:
                self._fault_flags = FaultFlags.NONE
                if self._mode == ActuatorMode.FAULT:
                    self._mode = ActuatorMode.DISABLED
            self._respond(command, sequence=sequence)
            return

        if command == CommandID.SELF_TEST:
            self._respond(command, data=b"OK", sequence=sequence)
            return

        self._respond(command, ResponseStatus.UNSUPPORTED, sequence=sequence)

    def _reset_pid(self) -> None:
        self._pid_integral = 0.0
        self._pid_last_error = 0.0
        self._velocity_integral = 0.0

    def _run(self) -> None:
        period = 1.0 / self.sample_hz
        next_sample = time.perf_counter()
        while self._running.is_set():
            now = time.perf_counter()
            dt = max(0.0, now - self._last_perf)
            self._last_perf = now
            self._update_motion(dt)
            if now >= next_sample:
                next_sample = now + period
                with self._lock:
                    streaming = self._streaming
                if streaming:
                    self._emit_telemetry(now)
            time.sleep(min(period / 4.0, 0.005))

    def _update_motion(self, dt: float) -> None:
        with self._lock:
            now = time.perf_counter()
            if self._mode in (ActuatorMode.DISABLED, ActuatorMode.FAULT) or self._fault_flags:
                self._motor_vel = 0.0
                self._target_motor_rad = self._motor_rad
                self._base_target_motor_rad = self._motor_rad
                self._issued_motor_rad = self._motor_rad
                self._chirp_active = False
            else:
                if self._chirp_active:
                    elapsed = now - self._chirp_started_at
                    if elapsed >= self._chirp_duration_s:
                        self._chirp_active = False
                        self._base_target_motor_rad = self._motor_rad
                        self._target_motor_rad = self._motor_rad
                    else:
                        sweep_rate = (self._chirp_end_hz - self._chirp_start_hz) / self._chirp_duration_s
                        phase = 2.0 * math.pi * (
                            self._chirp_start_hz * elapsed + 0.5 * sweep_rate * elapsed * elapsed
                        )
                        frequency = self._chirp_start_hz + sweep_rate * elapsed
                        target = self._chirp_center_motor_rad + self._chirp_amplitude_rad * math.sin(phase)
                        deflection = self._output_rad - (
                            self.output_per_motor * self._motor_rad + self.output_offset_rad
                        )
                        if abs(deflection) > self._chirp_max_deflection_rad:
                            self._chirp_active = False
                            self._base_target_motor_rad = self._motor_rad
                            self._target_motor_rad = self._motor_rad
                        else:
                            self._base_target_motor_rad = target
                            self._target_motor_rad = target
                            self._move_velocity = max(
                                0.1,
                                self._chirp_amplitude_rad * 2.0 * math.pi * max(frequency, 0.1) * 1.25,
                            )

                if self._mode == ActuatorMode.POSITION and self.pid_enabled:
                    error = self._output_target_rad - self._output_rad
                    self._pid_integral += error * dt
                    if self.pid_ki != 0.0:
                        integral_limit = self.pid_i_limit_motor_rad / abs(self.pid_ki)
                        self._pid_integral = max(-integral_limit, min(self._pid_integral, integral_limit))
                    derivative = (error - self._pid_last_error) / dt if dt > 0 else 0.0
                    self._pid_last_error = error
                    correction = self.pid_kp * error + self.pid_ki * self._pid_integral + self.pid_kd * derivative
                    correction = max(
                        -self.pid_output_limit_motor_rad,
                        min(correction, self.pid_output_limit_motor_rad),
                    )
                    self._target_motor_rad = self._base_target_motor_rad + correction
                elif self._mode == ActuatorMode.VELOCITY:
                    target_motor_vel = self._velocity_target_output_rad_s / self.output_per_motor
                    max_delta = self._move_accel * dt
                    if self._motor_vel < target_motor_vel:
                        self._motor_vel = min(target_motor_vel, self._motor_vel + max_delta)
                    else:
                        self._motor_vel = max(target_motor_vel, self._motor_vel - max_delta)
                    self._motor_rad += self._motor_vel * dt
                    self._issued_motor_rad = self._motor_rad
                    self._target_motor_rad = self._motor_rad
                    self._base_target_motor_rad = self._motor_rad
                elif self._mode == ActuatorMode.TORQUE_PROXY:
                    if now > self._torque_proxy_deadline:
                        self._fault_flags |= FaultFlags.CONTROL_ERROR
                        self._last_control_fault = "TORQUE_PROXY_TIMEOUT"
                        self._mode = ActuatorMode.FAULT
                        self._motor_vel = 0.0
                    elif abs(self._motor_rad - self._torque_proxy_start_motor_rad) > self._torque_proxy_max_excursion:
                        self._fault_flags |= FaultFlags.CONTROL_ERROR
                        self._last_control_fault = "TORQUE_PROXY_EXCURSION"
                        self._mode = ActuatorMode.FAULT
                        self._motor_vel = 0.0
                    else:
                        error = self._torque_proxy_target_rad - self._torque_proxy_rad()
                        target_motor_vel = -self.torque_proxy_kp * error
                        target_motor_vel = max(
                            -self._torque_proxy_max_velocity,
                            min(target_motor_vel, self._torque_proxy_max_velocity),
                        )
                        max_delta = self._move_accel * dt
                        if self._motor_vel < target_motor_vel:
                            self._motor_vel = min(target_motor_vel, self._motor_vel + max_delta)
                        else:
                            self._motor_vel = max(target_motor_vel, self._motor_vel - max_delta)
                        self._motor_rad += self._motor_vel * dt
                        self._issued_motor_rad = self._motor_rad
                        self._target_motor_rad = self._motor_rad
                        self._base_target_motor_rad = self._motor_rad
                elif not self._chirp_active:
                    self._target_motor_rad = self._base_target_motor_rad

                if self._mode in (ActuatorMode.CALIBRATION, ActuatorMode.POSITION):
                    error = self._target_motor_rad - self._motor_rad
                    desired_vel = 0.0 if abs(error) < 1e-9 else math.copysign(self._move_velocity, error)
                    max_delta_vel = max(0.01, self._move_accel) * dt
                    if self._motor_vel < desired_vel:
                        self._motor_vel = min(desired_vel, self._motor_vel + max_delta_vel)
                    else:
                        self._motor_vel = max(desired_vel, self._motor_vel - max_delta_vel)
                    max_step = abs(self._motor_vel) * dt
                    if abs(error) <= max(max_step, 1e-9):
                        self._motor_rad = self._target_motor_rad
                        self._motor_vel = 0.0
                    else:
                        self._motor_rad += math.copysign(max_step, error)
                    self._issued_motor_rad = self._motor_rad

            if self._autotune_state == 1:
                if abs(self._torque_proxy_rad()) > self.autotune_max_deflection_rad:
                    self._autotune_state = 3
                    self._fault_flags |= FaultFlags.AUTOTUNE_FAILED
                    self._last_control_fault = "AUTOTUNE_DEFLECTION_LIMIT"
                    self._mode = ActuatorMode.FAULT
                elif now >= self._autotune_deadline:
                    if self._autotune_loop_selector in (1, 3):
                        self.velocity_pid_kp = max(0.05, self._move_velocity * 0.04)
                        self.velocity_pid_ki = max(0.1, self.velocity_pid_kp * 8.0)
                    if self._autotune_loop_selector in (2, 3):
                        self.pid_enabled = True
                        self.pid_kp = max(0.1, self._autotune_amplitude_rad * 3.0)
                        self.pid_ki = max(0.01, self.pid_kp * 0.15)
                        self.pid_kd = max(0.0, self.pid_kp * 0.01)
                    self._autotune_state = 2
                    self._last_control_fault = ""

            self._update_motor_slip()

            target_output = self.output_per_motor * self._motor_rad + self.output_offset_rad
            if dt > 0 and self.resonance_frequency_hz is not None and self.resonance_frequency_hz > 0.0:
                omega = 2.0 * math.pi * self.resonance_frequency_hz
                accel = omega * omega * (target_output - self._output_rad) - (
                    2.0 * self.resonance_damping_ratio * omega * self._output_vel
                )
                new_output_vel = self._output_vel + accel * dt
                new_output = self._output_rad + new_output_vel * dt
                self._output_vel = new_output_vel
                self._output_rad = new_output
            elif dt > 0:
                alpha = min(1.0, dt * 45.0)
                new_output = self._output_rad + (target_output - self._output_rad) * alpha
                self._output_vel = (new_output - self._output_rad) / dt
                self._output_rad = new_output
            else:
                self._output_vel = self.output_per_motor * self._motor_vel
            self._update_current_command(now)

    def _emit_telemetry(self, now: float) -> None:
        with self._lock:
            self._telemetry_sequence = (self._telemetry_sequence + 1) & 0xFFFFFFFF
            motor_noise = self._rng.gauss(0.0, 0.00015)
            output_noise = self._rng.gauss(0.0, 0.00010)
            motor_rad = self._measured_motor_rad() + motor_noise
            output_rad = self._output_rad + output_noise
            motor_enc_raw = int(round(motor_rad / (2 * math.pi) * 4096.0))
            output_enc_raw = int(round(output_rad / (2 * math.pi) * 4096.0))
            payload = TelemetryPayload(
                t_us=int((now - self._start_perf) * 1_000_000),
                seq=self._telemetry_sequence,
                cmd_pos=self._target_motor_rad,
                cmd_vel=self._motor_vel,
                motor_enc_raw=motor_enc_raw,
                output_enc_raw=output_enc_raw,
                motor_rad=motor_rad,
                output_rad=output_rad,
                motor_vel_rad_s=self._motor_vel,
                output_vel_rad_s=self._output_vel,
                driver_current=self._commanded_current_a,
                bus_voltage=24.0,
                temperature=32.0 + 0.3 * abs(self._motor_vel),
                fault_flags=int(self._fault_flags),
                mode=int(self._mode),
                output_target_rad=self._output_target_rad,
                torque_proxy_rad=self._torque_proxy_rad(),
                motor_slip_rad=self._motor_slip_rad,
                commanded_current=self._commanded_current_a,
                control_state=self._autotune_state,
                telemetry_schema_version=2,
            )
            frame_seq = self._telemetry_sequence & 0xFFFF
        self._enqueue_frame(PacketType.TELEMETRY, encode_telemetry_payload(payload), sequence=frame_seq)


def scan_serial_ports() -> list[str]:
    if list_ports is None:
        return []
    return [port.device for port in list_ports.comports()]


TelemetryCallback = Callable[[TelemetrySample], None]
EventCallback = Callable[[Frame], None]


class ActuatorClient:
    """Threaded actuator client that keeps transport/protocol logic out of UI code."""

    def __init__(self, transport: Transport, telemetry_store: TelemetryStore | None = None) -> None:
        self.transport = transport
        self.telemetry_store = telemetry_store or TelemetryStore()
        self.max_velocity_rad_s = 40.0
        self.max_accel_rad_s2 = 1000.0
        self._parser = PacketParser()
        self._responses: queue.Queue[CommandResponse] = queue.Queue()
        self._telemetry_callbacks: list[TelemetryCallback] = []
        self._event_callbacks: list[EventCallback] = []
        self._reader_stop = threading.Event()
        self._reader_thread: threading.Thread | None = None
        self._sequence = 0
        self._sequence_lock = threading.Lock()

    @property
    def parser_stats(self):
        return self._parser.stats

    @property
    def is_connected(self) -> bool:
        return self.transport.is_open and self._reader_thread is not None

    def add_telemetry_callback(self, callback: TelemetryCallback) -> None:
        self._telemetry_callbacks.append(callback)

    def remove_telemetry_callback(self, callback: TelemetryCallback) -> None:
        if callback in self._telemetry_callbacks:
            self._telemetry_callbacks.remove(callback)

    def add_event_callback(self, callback: EventCallback) -> None:
        self._event_callbacks.append(callback)

    def connect(self) -> None:
        self.transport.open()
        self._reader_stop.clear()
        self._reader_thread = threading.Thread(target=self._reader_loop, name="actuator-reader", daemon=True)
        self._reader_thread.start()

    def disconnect(self) -> None:
        if self.transport.is_open:
            try:
                self.stop(timeout=0.3)
                self.set_mode(ActuatorMode.DISABLED, timeout=0.3)
                self.stop_stream(timeout=0.3)
            except ActuatorError:
                pass
        self._reader_stop.set()
        self.transport.close()
        if self._reader_thread is not None:
            self._reader_thread.join(timeout=1.0)
            self._reader_thread = None

    def _next_sequence(self) -> int:
        with self._sequence_lock:
            self._sequence = (self._sequence + 1) & 0xFFFF
            return self._sequence

    def _reader_loop(self) -> None:
        while not self._reader_stop.is_set() or self.transport.is_open:
            data = self.transport.read(4096, timeout=0.05)
            if not data:
                if not self.transport.is_open:
                    break
                continue
            for frame in self._parser.feed(data):
                self._handle_frame(frame)

    def _handle_frame(self, frame: Frame) -> None:
        if frame.packet_type == PacketType.RESPONSE:
            try:
                payload = decode_response_payload(frame.payload)
            except ProtocolError:
                return
            self._responses.put(
                CommandResponse(
                    command=payload.command,
                    status=payload.status,
                    data=payload.data,
                    frame_sequence=frame.sequence,
                )
            )
            return

        if frame.packet_type == PacketType.TELEMETRY:
            try:
                payload = decode_telemetry_payload(frame.payload)
            except ProtocolError:
                return
            sample = TelemetrySample.from_payload(payload)
            self.telemetry_store.add(sample)
            for callback in list(self._telemetry_callbacks):
                callback(sample)
            return

        if frame.packet_type == PacketType.EVENT:
            for callback in list(self._event_callbacks):
                callback(frame)

    def command(
        self,
        command: CommandID,
        data: bytes = b"",
        *,
        timeout: float = 1.0,
        require_ok: bool = True,
    ) -> CommandResponse:
        if not self.transport.is_open:
            raise ActuatorError("transport is not open")
        sequence = self._next_sequence()
        payload = encode_command_payload(command, data)
        self.transport.write(encode_frame(PacketType.COMMAND, sequence, payload))
        response = self._wait_response(command, sequence, timeout)
        if require_ok and response.status != ResponseStatus.OK:
            raise ActuatorCommandError(f"{command.name} failed with {response.status.name}")
        return response

    def _wait_response(self, command: CommandID, sequence: int, timeout: float) -> CommandResponse:
        deadline = time.monotonic() + timeout
        skipped: list[CommandResponse] = []
        while time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            try:
                response = self._responses.get(timeout=min(0.05, remaining))
            except queue.Empty:
                continue
            if response.command == command and response.frame_sequence == sequence:
                for skipped_response in skipped:
                    self._responses.put(skipped_response)
                return response
            skipped.append(response)
        for skipped_response in skipped:
            self._responses.put(skipped_response)
        raise ActuatorTimeoutError(f"timed out waiting for {command.name} sequence {sequence}")

    def ping(self, timeout: float = 1.0) -> bool:
        response = self.command(CommandID.PING, timeout=timeout)
        return response.data == b"PONG"

    def info(self, timeout: float = 1.0) -> ActuatorInfo:
        response = self.command(CommandID.INFO, timeout=timeout)
        return ActuatorInfo.from_dict(unpack_info_payload(response.data))

    def start_stream(self, timeout: float = 1.0) -> None:
        self.command(CommandID.STREAM_ON, timeout=timeout)

    def stop_stream(self, timeout: float = 1.0) -> None:
        self.command(CommandID.STREAM_OFF, timeout=timeout)

    def set_mode(self, mode: ActuatorMode, timeout: float = 1.0) -> None:
        self.command(CommandID.SET_MODE, pack_mode_payload(mode), timeout=timeout)

    def move_rel(
        self,
        delta_rad: float,
        velocity_rad_s: float,
        accel_rad_s2: float,
        timeout: float = 1.0,
    ) -> None:
        velocity = max(0.01, min(abs(float(velocity_rad_s)), self.max_velocity_rad_s))
        accel = max(0.01, min(abs(float(accel_rad_s2)), self.max_accel_rad_s2))
        self.command(
            CommandID.MOVE_REL,
            pack_move_rel_payload(float(delta_rad), velocity, accel),
            timeout=timeout,
        )

    def move_output_rel(
        self,
        delta_rad: float,
        velocity_rad_s: float,
        accel_rad_s2: float,
        timeout: float = 1.0,
    ) -> None:
        velocity = max(0.01, min(abs(float(velocity_rad_s)), self.max_velocity_rad_s))
        accel = max(0.01, min(abs(float(accel_rad_s2)), self.max_accel_rad_s2))
        self.command(
            CommandID.MOVE_OUTPUT_REL,
            pack_move_output_rel_payload(float(delta_rad), velocity, accel),
            timeout=timeout,
        )

    def set_position_target(
        self,
        target_output_rad: float,
        velocity_rad_s: float,
        accel_rad_s2: float,
        *,
        relative: bool = False,
        timeout: float = 1.0,
    ) -> None:
        velocity = max(0.01, min(abs(float(velocity_rad_s)), self.max_velocity_rad_s))
        accel = max(0.01, min(abs(float(accel_rad_s2)), self.max_accel_rad_s2))
        self.command(
            CommandID.SET_POSITION_TARGET,
            pack_position_target_payload(float(target_output_rad), velocity, accel, relative=relative),
            timeout=timeout,
        )

    def set_velocity_target(
        self,
        output_velocity_rad_s: float,
        accel_rad_s2: float,
        timeout: float = 1.0,
    ) -> None:
        velocity = max(-self.max_velocity_rad_s, min(float(output_velocity_rad_s), self.max_velocity_rad_s))
        accel = max(0.01, min(abs(float(accel_rad_s2)), self.max_accel_rad_s2))
        self.command(
            CommandID.SET_VELOCITY_TARGET,
            pack_velocity_target_payload(velocity, accel),
            timeout=timeout,
        )

    def set_torque_proxy_target(
        self,
        target_deflection_rad: float,
        max_motor_velocity_rad_s: float,
        max_motor_excursion_rad: float,
        timeout_s: float,
        timeout: float = 1.0,
    ) -> None:
        self.command(
            CommandID.SET_TORQUE_PROXY_TARGET,
            pack_torque_proxy_target_payload(
                target_deflection_rad,
                max_motor_velocity_rad_s,
                max_motor_excursion_rad,
                timeout_s,
            ),
            timeout=timeout,
        )

    def autotune_control(
        self,
        loop_selector: int,
        amplitude_rad: float,
        max_velocity_rad_s: float,
        duration_s: float,
        max_deflection_rad: float,
        timeout: float = 1.0,
    ) -> None:
        self.command(
            CommandID.AUTOTUNE_CONTROL,
            pack_autotune_control_payload(
                loop_selector,
                amplitude_rad,
                max_velocity_rad_s,
                duration_s,
                max_deflection_rad,
            ),
            timeout=timeout,
        )

    def get_control_status(self, timeout: float = 1.0) -> dict:
        response = self.command(CommandID.GET_CONTROL_STATUS, timeout=timeout)
        return json.loads(response.data.decode("utf-8"))

    def start_chirp(
        self,
        amplitude_rad: float = 0.18,
        start_frequency_hz: float = 0.8,
        end_frequency_hz: float = 75.0,
        duration_s: float = 12.0,
        max_deflection_rad: float = 0.25,
        timeout: float = 1.0,
    ) -> None:
        self.command(
            CommandID.START_CHIRP,
            pack_chirp_payload(
                amplitude_rad,
                start_frequency_hz,
                end_frequency_hz,
                duration_s,
                max_deflection_rad,
            ),
            timeout=timeout,
        )

    def stop(self, timeout: float = 1.0) -> None:
        self.command(CommandID.STOP, timeout=timeout)

    def estop(self, timeout: float = 1.0) -> None:
        self.command(CommandID.ESTOP, timeout=timeout)

    def zero_motor_encoder(self, timeout: float = 1.0) -> None:
        self.command(CommandID.ZERO_MOTOR_ENCODER, timeout=timeout)

    def zero_output_encoder(self, timeout: float = 1.0) -> None:
        self.command(CommandID.ZERO_OUTPUT_ENCODER, timeout=timeout)

    def faults(self, timeout: float = 1.0) -> int:
        response = self.command(CommandID.FAULTS, timeout=timeout)
        return unpack_faults_payload(response.data)

    def clear_faults(self, timeout: float = 1.0) -> None:
        self.command(CommandID.CLEAR_FAULTS, timeout=timeout)

    def self_test(self, timeout: float = 2.0) -> bytes:
        return self.command(CommandID.SELF_TEST, timeout=timeout).data

    def get_config(self, timeout: float = 1.0) -> dict:
        response = self.command(CommandID.GET_CONFIG, timeout=timeout)
        return json.loads(response.data.decode("utf-8"))

    def set_config(self, key: str, value, timeout: float = 1.0) -> None:
        payload = json.dumps({"key": key, "value": value}).encode("utf-8")
        self.command(CommandID.SET_CONFIG, payload, timeout=timeout)

    def save_config(self, timeout: float = 1.0) -> None:
        self.command(CommandID.SAVE_CONFIG, timeout=timeout)
