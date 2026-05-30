import pytest

from actuator_tool.config_schema import CALIBRATION_SCHEMA_VERSION, CalibrationConfig


def test_calibration_config_migrates_v1_to_v2_defaults():
    config = CalibrationConfig.from_dict(
        {
            "calibration_version": 1,
            "output_per_motor": 0.25,
            "motor_per_output": 4.0,
            "max_safe_velocity_rad_s": 8.0,
            "max_safe_accel_rad_s2": 60.0,
        }
    )

    assert config.calibration_version == CALIBRATION_SCHEMA_VERSION == 2
    assert config.velocity_pid_kp > 0
    assert config.torque_proxy_limit_rad > 0
    assert config.missed_step_correction_enabled is True
    assert config.run_current_ma == 1000


def test_calibration_config_rejects_invalid_production_limits():
    with pytest.raises(ValueError, match="run_current_ma"):
        CalibrationConfig(run_current_ma=100, hold_current_ma=350).validate()

    with pytest.raises(ValueError, match="missed_step_warn"):
        CalibrationConfig(missed_step_warn_motor_rad=0.5, missed_step_fault_motor_rad=0.2).validate()
