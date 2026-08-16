"""Two-inertia reduced-order simulator for a compliant belted actuator."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import heapq
import math
import random
from typing import Sequence

import numpy as np

from .plant_schema import ActuatorPlantParameters, EncoderModelParameters


@dataclass
class PlantState:
    time_s: float = 0.0
    motor_angle_rad: float = 0.0
    motor_velocity_rad_s: float = 0.0
    output_angle_rad: float = 0.0
    output_velocity_rad_s: float = 0.0
    requested_motor_torque_nm: float = 0.0
    delayed_motor_torque_nm: float = 0.0
    applied_motor_torque_nm: float = 0.0
    belt_torque_nm: float = 0.0
    output_friction_torque_nm: float = 0.0
    load_torque_nm: float = 0.0

    @property
    def relative_deflection_rad(self) -> float:
        # This convenience property is only exact when the caller uses a 1:1
        # ratio.  Use TwoInertiaActuatorSimulator.relative_deflection_rad for
        # the configured transmission ratio.
        return self.motor_angle_rad - self.output_angle_rad


@dataclass
class EncoderObservation:
    sample_time_s: float
    raw_count: int
    angle_rad: float
    velocity_rad_s: float


@dataclass
class PlantObservation:
    time_s: float
    motor: EncoderObservation
    output: EncoderObservation


@dataclass
class PlantSimulationTrace:
    time_s: np.ndarray
    requested_motor_torque_nm: np.ndarray
    delayed_motor_torque_nm: np.ndarray
    applied_motor_torque_nm: np.ndarray
    motor_angle_rad: np.ndarray
    motor_velocity_rad_s: np.ndarray
    output_angle_rad: np.ndarray
    output_velocity_rad_s: np.ndarray
    relative_deflection_rad: np.ndarray
    belt_torque_nm: np.ndarray
    output_friction_torque_nm: np.ndarray
    load_torque_nm: np.ndarray

    def as_dict(self) -> dict[str, np.ndarray]:
        return {
            "time_s": self.time_s,
            "requested_motor_torque_nm": self.requested_motor_torque_nm,
            "delayed_motor_torque_nm": self.delayed_motor_torque_nm,
            "applied_motor_torque_nm": self.applied_motor_torque_nm,
            "motor_angle_rad": self.motor_angle_rad,
            "motor_velocity_rad_s": self.motor_velocity_rad_s,
            "output_angle_rad": self.output_angle_rad,
            "output_velocity_rad_s": self.output_velocity_rad_s,
            "relative_deflection_rad": self.relative_deflection_rad,
            "belt_torque_nm": self.belt_torque_nm,
            "output_friction_torque_nm": self.output_friction_torque_nm,
            "load_torque_nm": self.load_torque_nm,
        }


class TwoInertiaActuatorSimulator:
    """Motor inertia and output inertia coupled by a compliant 3:1-style drive.

    The model uses output-side belt deflection

    ``delta = theta_motor / N - theta_output``

    and ``tau_belt = K*delta + C*delta_dot``.  The belt reaction seen at the
    motor is ``tau_belt/N``.  A measured stepper torque-speed envelope limits
    motor torque, while fitted Coulomb/viscous/Stribeck friction acts on the
    output side.

    Command latency/jitter and encoder quantization/noise/latency are modeled
    separately from the rigid-body integration so the same plant parameters can
    be reused by external physics engines.
    """

    def __init__(self, parameters: ActuatorPlantParameters) -> None:
        parameters.validate()
        self.parameters = parameters
        self._rng = random.Random(parameters.random_seed)
        self._command_counter = 0
        self._pending_commands: list[tuple[float, int, float]] = []
        self._delayed_command_nm = 0.0
        self._state = PlantState()
        self._history: deque[PlantState] = deque(maxlen=200_000)
        self._encoder_cache: dict[str, EncoderObservation | None] = {"motor": None, "output": None}
        self._next_encoder_sample_s: dict[str, float] = {"motor": 0.0, "output": 0.0}
        self.reset()

    def reset(
        self,
        *,
        motor_angle_rad: float = 0.0,
        motor_velocity_rad_s: float = 0.0,
        output_angle_rad: float = 0.0,
        output_velocity_rad_s: float = 0.0,
    ) -> PlantState:
        self._rng = random.Random(self.parameters.random_seed)
        self._command_counter = 0
        self._pending_commands.clear()
        self._delayed_command_nm = 0.0
        self._state = PlantState(
            time_s=0.0,
            motor_angle_rad=float(motor_angle_rad),
            motor_velocity_rad_s=float(motor_velocity_rad_s),
            output_angle_rad=float(output_angle_rad),
            output_velocity_rad_s=float(output_velocity_rad_s),
        )
        self._history.clear()
        self._history.append(PlantState(**self._state.__dict__))
        self._encoder_cache = {"motor": None, "output": None}
        self._next_encoder_sample_s = {"motor": 0.0, "output": 0.0}
        return self.state

    @property
    def state(self) -> PlantState:
        return PlantState(**self._state.__dict__)

    @property
    def relative_deflection_rad(self) -> float:
        ratio = self.parameters.gear_ratio_motor_per_output
        return self._state.motor_angle_rad / ratio - self._state.output_angle_rad

    @property
    def relative_deflection_rate_rad_s(self) -> float:
        ratio = self.parameters.gear_ratio_motor_per_output
        return self._state.motor_velocity_rad_s / ratio - self._state.output_velocity_rad_s

    def _schedule_command(self, requested_motor_torque_nm: float) -> None:
        jitter = self._rng.gauss(0.0, self.parameters.command_jitter_std_s)
        apply_time = max(
            self._state.time_s,
            self._state.time_s + self.parameters.command_latency_s + jitter,
        )
        self._command_counter += 1
        heapq.heappush(
            self._pending_commands,
            (apply_time, self._command_counter, float(requested_motor_torque_nm)),
        )

    def _apply_due_commands(self) -> None:
        while self._pending_commands and self._pending_commands[0][0] <= self._state.time_s + 1e-15:
            _apply_time, _sequence, torque = heapq.heappop(self._pending_commands)
            self._delayed_command_nm = torque

    def _belt_torque(self, state_vector: np.ndarray) -> float:
        theta_m, omega_m, theta_o, omega_o = state_vector
        ratio = self.parameters.gear_ratio_motor_per_output
        deflection = theta_m / ratio - theta_o
        deflection_rate = omega_m / ratio - omega_o
        return (
            self.parameters.belt_stiffness_nm_per_rad * deflection
            + self.parameters.belt_damping_nm_s_per_rad * deflection_rate
        )

    def _motor_torque(self, requested_nm: float, motor_speed_rad_s: float) -> float:
        limit = self.parameters.torque_limit_nm(motor_speed_rad_s)
        return max(-limit, min(float(requested_nm), limit))

    def _derivative(self, state_vector: np.ndarray, delayed_command_nm: float, load_torque_nm: float) -> np.ndarray:
        theta_m, omega_m, theta_o, omega_o = state_vector
        belt_torque = self._belt_torque(state_vector)
        motor_torque = self._motor_torque(delayed_command_nm, omega_m)
        friction = self.parameters.output_friction_torque_nm(omega_o)
        ratio = self.parameters.gear_ratio_motor_per_output
        alpha_m = (motor_torque - belt_torque / ratio) / self.parameters.motor_inertia_kg_m2
        alpha_o = (
            belt_torque - friction - float(load_torque_nm)
        ) / self.parameters.output_inertia_kg_m2
        return np.array([omega_m, alpha_m, omega_o, alpha_o], dtype=float)

    def _integrate_substep(self, dt: float, load_torque_nm: float) -> None:
        y0 = np.array(
            [
                self._state.motor_angle_rad,
                self._state.motor_velocity_rad_s,
                self._state.output_angle_rad,
                self._state.output_velocity_rad_s,
            ],
            dtype=float,
        )
        u = self._delayed_command_nm
        k1 = self._derivative(y0, u, load_torque_nm)
        k2 = self._derivative(y0 + 0.5 * dt * k1, u, load_torque_nm)
        k3 = self._derivative(y0 + 0.5 * dt * k2, u, load_torque_nm)
        k4 = self._derivative(y0 + dt * k3, u, load_torque_nm)
        y = y0 + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
        if not np.all(np.isfinite(y)):
            raise FloatingPointError("plant integration diverged to a non-finite state")

        self._state.time_s += dt
        self._state.motor_angle_rad = float(y[0])
        self._state.motor_velocity_rad_s = float(y[1])
        self._state.output_angle_rad = float(y[2])
        self._state.output_velocity_rad_s = float(y[3])
        self._state.delayed_motor_torque_nm = self._delayed_command_nm
        self._state.applied_motor_torque_nm = self._motor_torque(
            self._delayed_command_nm, self._state.motor_velocity_rad_s
        )
        state_vector = np.array(y, dtype=float)
        self._state.belt_torque_nm = self._belt_torque(state_vector)
        self._state.output_friction_torque_nm = self.parameters.output_friction_torque_nm(
            self._state.output_velocity_rad_s
        )
        self._state.load_torque_nm = float(load_torque_nm)
        self._history.append(PlantState(**self._state.__dict__))

    def step(
        self,
        requested_motor_torque_nm: float,
        dt: float,
        *,
        load_torque_nm: float = 0.0,
    ) -> PlantState:
        """Advance the model by ``dt`` and return a copy of the resulting state."""
        if not math.isfinite(float(dt)) or dt <= 0.0:
            raise ValueError("dt must be finite and positive")
        if not math.isfinite(float(requested_motor_torque_nm)):
            raise ValueError("requested_motor_torque_nm must be finite")
        if not math.isfinite(float(load_torque_nm)):
            raise ValueError("load_torque_nm must be finite")

        self._state.requested_motor_torque_nm = float(requested_motor_torque_nm)
        self._schedule_command(requested_motor_torque_nm)
        remaining = float(dt)
        maximum_step = self.parameters.integration_step_s
        while remaining > 1e-15:
            self._apply_due_commands()
            substep = min(remaining, maximum_step)
            # If a delayed command lands inside the next integration interval,
            # split exactly at that boundary rather than smearing the latency.
            if self._pending_commands:
                until_command = self._pending_commands[0][0] - self._state.time_s
                if 1e-15 < until_command < substep:
                    substep = until_command
            self._integrate_substep(substep, load_torque_nm)
            remaining -= substep
        self._apply_due_commands()
        return self.state

    def _state_at_or_before(self, target_time_s: float) -> PlantState:
        target = max(0.0, min(float(target_time_s), self._state.time_s))
        for state in reversed(self._history):
            if state.time_s <= target + 1e-15:
                return state
        return self._history[0]

    def _sample_encoder(self, name: str, parameters: EncoderModelParameters) -> EncoderObservation:
        cached = self._encoder_cache[name]
        if cached is not None and self._state.time_s + 1e-15 < self._next_encoder_sample_s[name]:
            return cached

        latency_jitter = self._rng.gauss(0.0, parameters.jitter_std_s)
        source_time = self._state.time_s - parameters.latency_s - latency_jitter
        source_state = self._state_at_or_before(source_time)
        if name == "motor":
            true_angle = source_state.motor_angle_rad
            true_velocity = source_state.motor_velocity_rad_s
        else:
            true_angle = source_state.output_angle_rad
            true_velocity = source_state.output_velocity_rad_s

        noisy_angle = true_angle + parameters.bias_rad + self._rng.gauss(0.0, parameters.noise_std_rad)
        raw_count = int(round(noisy_angle / (2.0 * math.pi) * parameters.counts_per_rev))
        quantized_angle = raw_count * (2.0 * math.pi / parameters.counts_per_rev)
        observation = EncoderObservation(
            sample_time_s=source_state.time_s,
            raw_count=raw_count,
            angle_rad=float(quantized_angle),
            velocity_rad_s=float(true_velocity),
        )
        self._encoder_cache[name] = observation
        period_jitter = self._rng.gauss(0.0, parameters.jitter_std_s)
        period = max(0.0, parameters.sample_period_s + period_jitter)
        self._next_encoder_sample_s[name] = self._state.time_s + period
        return observation

    def observe(self) -> PlantObservation:
        """Return quantized/noisy/delayed encoder observations at the current time."""
        motor = self._sample_encoder("motor", self.parameters.motor_encoder)
        output = self._sample_encoder("output", self.parameters.output_encoder)
        return PlantObservation(time_s=self._state.time_s, motor=motor, output=output)

    def simulate(
        self,
        time_s: Sequence[float] | np.ndarray,
        requested_motor_torque_nm: Sequence[float] | np.ndarray,
        *,
        load_torque_nm: Sequence[float] | np.ndarray | float = 0.0,
        reset: bool = True,
    ) -> PlantSimulationTrace:
        """Simulate a sampled torque trace.

        Torque sample ``i-1`` is held over interval ``[t[i-1], t[i]]``.  The
        returned arrays include the state at every requested timestamp.
        """
        time = np.asarray(time_s, dtype=float).reshape(-1)
        torque = np.asarray(requested_motor_torque_nm, dtype=float).reshape(-1)
        if len(time) < 2 or len(time) != len(torque):
            raise ValueError("time_s and requested_motor_torque_nm must have the same length >= 2")
        if np.any(~np.isfinite(time)) or np.any(~np.isfinite(torque)):
            raise ValueError("simulation inputs must be finite")
        if np.any(np.diff(time) <= 0.0):
            raise ValueError("time_s must be strictly increasing")
        if abs(float(time[0])) > 1e-12:
            time = time - time[0]

        if np.isscalar(load_torque_nm):
            load = np.full(len(time), float(load_torque_nm), dtype=float)
        else:
            load = np.asarray(load_torque_nm, dtype=float).reshape(-1)
            if len(load) != len(time):
                raise ValueError("load_torque_nm must be scalar or match time_s")
        if np.any(~np.isfinite(load)):
            raise ValueError("load torque must be finite")

        if reset:
            self.reset()

        states: list[PlantState] = [self.state]
        states[0].requested_motor_torque_nm = float(torque[0])
        for index in range(1, len(time)):
            dt = float(time[index] - time[index - 1])
            state = self.step(
                float(torque[index - 1]),
                dt,
                load_torque_nm=float(load[index - 1]),
            )
            states.append(state)

        ratio = self.parameters.gear_ratio_motor_per_output
        return PlantSimulationTrace(
            time_s=time.copy(),
            requested_motor_torque_nm=torque.copy(),
            delayed_motor_torque_nm=np.array([state.delayed_motor_torque_nm for state in states]),
            applied_motor_torque_nm=np.array([state.applied_motor_torque_nm for state in states]),
            motor_angle_rad=np.array([state.motor_angle_rad for state in states]),
            motor_velocity_rad_s=np.array([state.motor_velocity_rad_s for state in states]),
            output_angle_rad=np.array([state.output_angle_rad for state in states]),
            output_velocity_rad_s=np.array([state.output_velocity_rad_s for state in states]),
            relative_deflection_rad=np.array(
                [state.motor_angle_rad / ratio - state.output_angle_rad for state in states]
            ),
            belt_torque_nm=np.array([state.belt_torque_nm for state in states]),
            output_friction_torque_nm=np.array(
                [state.output_friction_torque_nm for state in states]
            ),
            load_torque_nm=load.copy(),
        )
