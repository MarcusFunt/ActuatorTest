import math

import numpy as np

from actuator_tool.actuator_data import TelemetrySample
from actuator_tool.plant_schema import ActuatorPlantParameters, TorqueSpeedPoint
from actuator_tool.plant_telemetry import derive_telemetry_dynamics


def _sample(index: int, t_s: float) -> TelemetrySample:
    output = t_s
    motor = 3.0 * (output + 0.01)
    return TelemetrySample(
        t_us=int(round(t_s * 1_000_000)),
        seq=index,
        cmd_pos=motor,
        cmd_vel=3.0,
        motor_enc_raw=0,
        output_enc_raw=0,
        motor_rad=motor,
        output_rad=output,
        motor_vel_rad_s=3.0,
        output_vel_rad_s=1.0,
        driver_current=0.5,
        bus_voltage=24.0,
        temperature=25.0,
        fault_flags=0,
        mode=0,
    )


def test_derive_telemetry_dynamics_uses_physical_belt_sign_and_ratio():
    params = ActuatorPlantParameters(
        gear_ratio_motor_per_output=3.0,
        motor_inertia_kg_m2=1e-4,
        output_inertia_kg_m2=1e-3,
        belt_stiffness_nm_per_rad=10.0,
        belt_damping_nm_s_per_rad=0.2,
        motor_torque_speed_map=[TorqueSpeedPoint(0.0, 1.0), TorqueSpeedPoint(100.0, 0.5)],
    )
    samples = [_sample(i, i * 0.01) for i in range(6)]
    dynamics = derive_telemetry_dynamics(samples, params)

    assert np.allclose(dynamics.belt_deflection_rad, 0.01, atol=1e-12)
    assert np.allclose(dynamics.belt_deflection_rate_rad_s, 0.0, atol=1e-12)
    assert np.allclose(dynamics.belt_torque_nm, 0.1, atol=1e-12)
    assert np.allclose(dynamics.motor_acceleration_rad_s2, 0.0, atol=1e-10)
    assert np.allclose(dynamics.estimated_motor_torque_to_drivetrain_nm, 0.1 / 3.0, atol=1e-10)
