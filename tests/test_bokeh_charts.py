import pytest

from actuator_gui.bokeh_charts import (
    StickyRange,
    StickyRangeSpec,
    build_telemetry_series,
    local_websocket_origins,
)
from actuator_tool.actuator_data import TelemetrySample


def sample(
    seq: int,
    *,
    motor: float = 0.0,
    output: float = 0.0,
    cmd_pos: float = 0.0,
    cmd_vel: float = 0.0,
    motor_vel: float = 0.0,
    output_vel: float = 0.0,
    output_target: float = 0.0,
    torque_proxy: float = 0.0,
    motor_slip: float = 0.0,
    driver_current: float = 0.0,
    commanded_current: float = 0.0,
    bus_voltage: float = 24.0,
    temperature: float = 32.0,
    fault_flags: int = 0,
    mode: int = 1,
    control_state: int = 0,
) -> TelemetrySample:
    return TelemetrySample(
        t_us=seq * 10_000,
        seq=seq,
        cmd_pos=cmd_pos,
        cmd_vel=cmd_vel,
        motor_enc_raw=int(motor * 1000),
        output_enc_raw=int(output * 1000),
        motor_rad=motor,
        output_rad=output,
        motor_vel_rad_s=motor_vel,
        output_vel_rad_s=output_vel,
        driver_current=driver_current,
        bus_voltage=bus_voltage,
        temperature=temperature,
        fault_flags=fault_flags,
        mode=mode,
        output_target_rad=output_target,
        torque_proxy_rad=torque_proxy,
        motor_slip_rad=motor_slip,
        commanded_current=commanded_current,
        control_state=control_state,
        telemetry_schema_version=2,
    )


def test_build_telemetry_series_includes_schema_v2_fields_and_output_space_values():
    samples = [
        sample(
            1,
            motor=2.0,
            output=0.53,
            cmd_pos=3.0,
            cmd_vel=4.0,
            motor_vel=5.0,
            output_vel=1.26,
            output_target=0.75,
            torque_proxy=0.02,
            motor_slip=-0.01,
            driver_current=0.4,
            commanded_current=0.7,
            fault_flags=3,
            mode=2,
            control_state=4,
        )
    ]

    series = build_telemetry_series(samples, 0.25, 0.03)

    assert series["predicted_output_rad"] == pytest.approx([0.53])
    assert series["commanded_output_rad"] == pytest.approx([0.78])
    assert series["predicted_output_vel_rad_s"] == pytest.approx([1.25])
    assert series["commanded_output_vel_rad_s"] == pytest.approx([1.0])
    assert series["target_output_rad"] == pytest.approx([0.75])
    assert series["deflection_rad"] == pytest.approx([0.0])
    assert series["torque_proxy_rad"] == pytest.approx([0.02])
    assert series["motor_slip_rad"] == pytest.approx([-0.01])
    assert series["driver_current_a"] == pytest.approx([0.4])
    assert series["commanded_current_a"] == pytest.approx([0.7])
    assert series["fault_flags"] == pytest.approx([3.0])
    assert series["mode"] == pytest.approx([2.0])
    assert series["control_state"] == pytest.approx([4.0])


def test_build_telemetry_series_downsamples_to_limit():
    samples = [sample(seq, motor=float(seq), output=float(seq)) for seq in range(21)]

    series = build_telemetry_series(samples, 1.0, 0.0, max_points=5)

    assert len(series["t"]) <= 5
    assert series["motor_rad"] == pytest.approx([0.0, 5.0, 10.0, 15.0, 20.0])


def test_sticky_range_expands_but_does_not_shrink():
    sticky = StickyRange(StickyRangeSpec((0.0, 1.0), min_span=1.0, padding=0.0))

    assert sticky.update([0.0, 1.0]) == pytest.approx((0.0, 1.0))
    assert sticky.update([0.4, 0.6]) == pytest.approx((0.0, 1.0))
    assert sticky.update([-2.0, 0.5]) == pytest.approx((-2.0, 1.0))


def test_sticky_range_minimum_span_keeps_idle_noise_readable():
    sticky = StickyRange(
        StickyRangeSpec((-0.06, 0.06), min_span=0.12, padding=0.0, include_zero=True)
    )

    start, end = sticky.update([-0.0002, 0.0003])

    assert end - start == pytest.approx(0.12)
    assert start < -0.01
    assert end > 0.01


def test_bokeh_origins_are_limited_to_loopback_frontend_and_server():
    origins = local_websocket_origins(5006, frontend_port=3007)

    assert origins == [
        "localhost:5006",
        "127.0.0.1:5006",
        "localhost:3007",
        "127.0.0.1:3007",
    ]
