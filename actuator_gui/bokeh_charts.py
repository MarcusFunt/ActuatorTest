"""Bokeh live-chart server - runs in a daemon thread, served at http://localhost:5006."""

from __future__ import annotations

import asyncio
import errno
import math
import threading
from dataclasses import dataclass
from typing import Any, Iterable

from bokeh.application import Application
from bokeh.application.handlers.function import FunctionHandler
from bokeh.layouts import column, row
from bokeh.models import Button, ColumnDataSource, HoverTool, Range1d, Span
from bokeh.plotting import figure
from bokeh.server.server import Server
from bokeh.themes import Theme
from jinja2 import Template

from actuator_tool.actuator_data import TelemetrySample

BOKEH_PORT = 5006
_MAX_POINTS = 800

_DARK_THEME = Theme(
    json={
        "attrs": {
            "Plot": {
                "background_fill_color": "#080b10",
                "border_fill_color": "#161b22",
                "outline_line_color": "#30363d",
            },
            "Grid": {"grid_line_color": "#21262d", "grid_line_alpha": 0.82},
            "Axis": {
                "axis_line_color": "#30363d",
                "major_tick_line_color": "#30363d",
                "minor_tick_line_color": "#21262d",
                "major_label_text_color": "#8b949e",
                "axis_label_text_color": "#8b949e",
                "major_label_text_font": "JetBrains Mono",
                "axis_label_text_font": "Space Grotesk",
                "major_label_text_font_size": "10px",
                "axis_label_text_font_size": "11px",
            },
            "Title": {
                "text_color": "#e6edf3",
                "text_font_size": "13px",
                "text_font": "Space Grotesk",
            },
            "Legend": {
                "background_fill_color": "#161b22",
                "background_fill_alpha": 0.74,
                "label_text_color": "#e6edf3",
                "label_text_font": "Space Grotesk",
                "label_text_font_size": "11px",
                "border_line_color": "#30363d",
                "spacing": 3,
                "padding": 6,
            },
            "Toolbar": {"autohide": True},
        }
    }
)

_C = {
    "blue": "#58a6ff",
    "green": "#3fb950",
    "red": "#ff7b72",
    "amber": "#d29922",
    "orange": "#f0883e",
    "purple": "#a371f7",
    "cyan": "#56d4dd",
    "gray": "#8b949e",
    "muted": "#484f58",
}


_IFRAME_CSS = Template(
    """
{% extends base %}
{% block preamble %}
<style>
html, body {
    margin: 0 !important;
    padding: 0 !important;
    min-height: 100%;
    overflow-y: auto;
    overflow-x: hidden;
    background: #161b22;
}
body::-webkit-scrollbar {
    width: 8px;
}
body::-webkit-scrollbar-thumb {
    background: #30363d;
    border-radius: 4px;
}
.bk-root {
    background: #161b22;
}
</style>
{% endblock %}
"""
)

_SERIES_KEYS = (
    "t",
    "measured_output_rad",
    "predicted_output_rad",
    "target_output_rad",
    "commanded_output_rad",
    "deflection_rad",
    "torque_proxy_rad",
    "motor_slip_rad",
    "measured_output_vel_rad_s",
    "predicted_output_vel_rad_s",
    "commanded_output_vel_rad_s",
    "driver_current_a",
    "commanded_current_a",
    "bus_voltage_v",
    "temperature_c",
    "motor_enc_raw",
    "output_enc_raw",
    "mode",
    "fault_flags",
    "control_state",
    "motor_rad",
    "cmd_pos",
)


@dataclass(frozen=True)
class StickyRangeSpec:
    initial: tuple[float, float]
    min_span: float
    padding: float = 0.12
    include_zero: bool = False


class StickyRange:
    """A y-range that expands to fit new data but does not shrink every refresh."""

    def __init__(self, spec: StickyRangeSpec) -> None:
        self.spec = spec
        self.start, self.end = spec.initial
        self._has_data = False

    def reset(self) -> tuple[float, float]:
        self.start, self.end = self.spec.initial
        self._has_data = False
        return self.bounds

    @property
    def bounds(self) -> tuple[float, float]:
        return self.start, self.end

    def update(self, values: Iterable[float]) -> tuple[float, float]:
        numeric = [float(value) for value in values if math.isfinite(float(value))]
        if not numeric:
            return self.bounds

        lo = min(numeric)
        hi = max(numeric)
        if self.spec.include_zero:
            lo = min(lo, 0.0)
            hi = max(hi, 0.0)

        data_span = max(hi - lo, 0.0)
        padded_span = max(data_span * (1.0 + 2.0 * self.spec.padding), self.spec.min_span)
        center = (lo + hi) / 2.0
        proposed_start = center - padded_span / 2.0
        proposed_end = center + padded_span / 2.0

        if not self._has_data:
            self.start = proposed_start
            self.end = proposed_end
            self._has_data = True
        else:
            self.start = min(self.start, proposed_start)
            self.end = max(self.end, proposed_end)
        return self.bounds


def empty_telemetry_series() -> dict[str, list[float]]:
    return {key: [] for key in _SERIES_KEYS}


def build_telemetry_series(
    samples: Iterable[TelemetrySample],
    output_per_motor: float,
    output_offset_rad: float,
    *,
    max_points: int = _MAX_POINTS,
) -> dict[str, list[float]]:
    """Return downsampled live telemetry in calibrated output-space where possible."""
    sample_list = list(samples)
    if not sample_list:
        return empty_telemetry_series()

    if max_points > 0 and len(sample_list) > max_points:
        stride = max(1, math.ceil(len(sample_list) / max_points))
        sample_list = sample_list[::stride]

    base = sample_list[0].t_us
    ratio = float(output_per_motor)
    offset = float(output_offset_rad)

    series = empty_telemetry_series()
    for sample in sample_list:
        predicted_output = ratio * sample.motor_rad + offset
        commanded_output = ratio * sample.cmd_pos + offset
        predicted_output_vel = ratio * sample.motor_vel_rad_s
        commanded_output_vel = ratio * sample.cmd_vel

        series["t"].append((sample.t_us - base) / 1_000_000.0)
        series["measured_output_rad"].append(sample.output_rad)
        series["predicted_output_rad"].append(predicted_output)
        series["target_output_rad"].append(sample.output_target_rad)
        series["commanded_output_rad"].append(commanded_output)
        series["deflection_rad"].append(sample.output_rad - predicted_output)
        series["torque_proxy_rad"].append(sample.torque_proxy_rad)
        series["motor_slip_rad"].append(sample.motor_slip_rad)
        series["measured_output_vel_rad_s"].append(sample.output_vel_rad_s)
        series["predicted_output_vel_rad_s"].append(predicted_output_vel)
        series["commanded_output_vel_rad_s"].append(commanded_output_vel)
        series["driver_current_a"].append(sample.driver_current)
        series["commanded_current_a"].append(sample.commanded_current)
        series["bus_voltage_v"].append(sample.bus_voltage)
        series["temperature_c"].append(sample.temperature)
        series["motor_enc_raw"].append(float(sample.motor_enc_raw))
        series["output_enc_raw"].append(float(sample.output_enc_raw))
        series["mode"].append(float(sample.mode))
        series["fault_flags"].append(float(sample.fault_flags))
        series["control_state"].append(float(sample.control_state))
        series["motor_rad"].append(sample.motor_rad)
        series["cmd_pos"].append(sample.cmd_pos)
    return series


def _make_range(spec: StickyRangeSpec) -> tuple[StickyRange, Range1d]:
    sticky = StickyRange(spec)
    start, end = sticky.bounds
    return sticky, Range1d(start=start, end=end)


def _plot(title: str, height: int, y_label: str, y_range: Range1d) -> Any:
    p = figure(
        title=title,
        height=height,
        sizing_mode="stretch_width",
        tools="pan,box_zoom,wheel_zoom,reset,save",
        toolbar_location="right",
        x_axis_label="Time [s]",
        y_axis_label=y_label,
        x_range=Range1d(0.0, 1.0),
        y_range=y_range,
        active_drag="pan",
    )
    p.toolbar.logo = None
    p.toolbar.autohide = True
    p.min_border_left = 54
    p.min_border_right = 18
    p.min_border_top = 28
    p.min_border_bottom = 34
    p.legend.click_policy = "hide"
    p.legend.location = "top_right"
    return p


def _add_zero_line(plot: Any) -> None:
    plot.add_layout(Span(location=0.0, dimension="width", line_color=_C["muted"], line_alpha=0.55, line_width=1))


def _apply_x_range(plots: Iterable[Any], t_values: list[float]) -> None:
    end = max(max(t_values, default=1.0), 1.0)
    for plot in plots:
        plot.x_range.start = 0.0
        plot.x_range.end = end


def _apply_y_range(sticky: StickyRange, model_range: Range1d, values: Iterable[float]) -> None:
    model_range.start, model_range.end = sticky.update(values)


def _make_document(doc: Any, ctx_obj: Any) -> None:
    """Build a Bokeh document with grouped live-updating telemetry charts."""
    src = ColumnDataSource(empty_telemetry_series())

    range_defs = {
        "position": StickyRangeSpec((-0.5, 0.5), min_span=1.0, include_zero=True),
        "deflection": StickyRangeSpec((-0.06, 0.06), min_span=0.12, include_zero=True),
        "velocity": StickyRangeSpec((-1.0, 1.0), min_span=2.0, include_zero=True),
        "current": StickyRangeSpec((0.0, 1.2), min_span=1.2, include_zero=True),
        "environment": StickyRangeSpec((15.0, 40.0), min_span=12.0),
        "encoder": StickyRangeSpec((-1000.0, 1000.0), min_span=2000.0, include_zero=True),
        "status": StickyRangeSpec((-0.5, 4.5), min_span=5.0, include_zero=True),
    }
    sticky_ranges: dict[str, StickyRange] = {}
    bokeh_ranges: dict[str, Range1d] = {}
    for name, spec in range_defs.items():
        sticky_ranges[name], bokeh_ranges[name] = _make_range(spec)

    p_position = _plot("Output Position Tracking", 240, "output rad", bokeh_ranges["position"])
    p_position.line("t", "measured_output_rad", source=src, color=_C["green"], legend_label="measured output", line_width=2.2)
    p_position.line("t", "predicted_output_rad", source=src, color=_C["blue"], legend_label="motor -> output", line_width=1.7)
    p_position.line("t", "target_output_rad", source=src, color=_C["amber"], legend_label="output target", line_width=1.7)
    p_position.line("t", "commanded_output_rad", source=src, color=_C["gray"], legend_label="cmd -> output", line_width=1.2, line_dash="dashed")
    p_position.add_tools(
        HoverTool(
            mode="vline",
            tooltips=[
                ("t", "@t{0.000}s"),
                ("measured", "@measured_output_rad{0.0000} rad"),
                ("motor -> output", "@predicted_output_rad{0.0000} rad"),
                ("target", "@target_output_rad{0.0000} rad"),
            ],
        )
    )

    p_deflection = _plot("Deflection, Torque Proxy, and Slip", 190, "rad", bokeh_ranges["deflection"])
    p_deflection.line("t", "deflection_rad", source=src, color=_C["red"], legend_label="deflection", line_width=2.0)
    p_deflection.line("t", "torque_proxy_rad", source=src, color=_C["orange"], legend_label="torque proxy", line_width=1.6)
    p_deflection.line("t", "motor_slip_rad", source=src, color=_C["purple"], legend_label="motor slip", line_width=1.4, line_dash="dotted")
    _add_zero_line(p_deflection)
    p_deflection.add_tools(
        HoverTool(
            mode="vline",
            tooltips=[
                ("t", "@t{0.000}s"),
                ("deflection", "@deflection_rad{0.00000} rad"),
                ("torque proxy", "@torque_proxy_rad{0.00000} rad"),
                ("motor slip", "@motor_slip_rad{0.00000} rad"),
            ],
        )
    )

    p_velocity = _plot("Output Velocity Tracking", 190, "output rad/s", bokeh_ranges["velocity"])
    p_velocity.line("t", "measured_output_vel_rad_s", source=src, color=_C["green"], legend_label="measured output vel", line_width=2.0)
    p_velocity.line("t", "predicted_output_vel_rad_s", source=src, color=_C["blue"], legend_label="motor -> output vel", line_width=1.7)
    p_velocity.line("t", "commanded_output_vel_rad_s", source=src, color=_C["gray"], legend_label="cmd -> output vel", line_width=1.2, line_dash="dashed")
    _add_zero_line(p_velocity)
    p_velocity.add_tools(
        HoverTool(
            mode="vline",
            tooltips=[
                ("t", "@t{0.000}s"),
                ("measured", "@measured_output_vel_rad_s{0.000} rad/s"),
                ("motor -> output", "@predicted_output_vel_rad_s{0.000} rad/s"),
                ("command", "@commanded_output_vel_rad_s{0.000} rad/s"),
            ],
        )
    )

    p_current = _plot("Current", 160, "A", bokeh_ranges["current"])
    p_current.line("t", "driver_current_a", source=src, color=_C["cyan"], legend_label="driver current", line_width=2.0)
    p_current.line("t", "commanded_current_a", source=src, color=_C["amber"], legend_label="commanded current", line_width=1.5, line_dash="dashed")
    _add_zero_line(p_current)
    p_current.add_tools(
        HoverTool(
            mode="vline",
            tooltips=[
                ("t", "@t{0.000}s"),
                ("driver", "@driver_current_a{0.000} A"),
                ("commanded", "@commanded_current_a{0.000} A"),
            ],
        )
    )

    p_environment = _plot("Voltage and Temperature", 160, "V / C", bokeh_ranges["environment"])
    p_environment.line("t", "bus_voltage_v", source=src, color=_C["blue"], legend_label="bus voltage", line_width=1.9)
    p_environment.line("t", "temperature_c", source=src, color=_C["orange"], legend_label="temperature", line_width=1.9)
    p_environment.add_tools(
        HoverTool(
            mode="vline",
            tooltips=[
                ("t", "@t{0.000}s"),
                ("bus", "@bus_voltage_v{0.00} V"),
                ("temp", "@temperature_c{0.00} C"),
            ],
        )
    )

    p_encoder = _plot("Raw Encoder Counts", 170, "ticks", bokeh_ranges["encoder"])
    p_encoder.line("t", "motor_enc_raw", source=src, color=_C["blue"], legend_label="motor raw", line_width=1.6)
    p_encoder.line("t", "output_enc_raw", source=src, color=_C["green"], legend_label="output raw", line_width=1.6)
    p_encoder.add_tools(
        HoverTool(
            mode="vline",
            tooltips=[
                ("t", "@t{0.000}s"),
                ("motor raw", "@motor_enc_raw{0}"),
                ("output raw", "@output_enc_raw{0}"),
            ],
        )
    )

    p_status = _plot("Status Diagnostics", 150, "state / flags", bokeh_ranges["status"])
    p_status.line("t", "mode", source=src, color=_C["green"], legend_label="mode", line_width=1.8, line_dash="solid")
    p_status.line("t", "control_state", source=src, color=_C["purple"], legend_label="control state", line_width=1.8, line_dash="dashed")
    p_status.line("t", "fault_flags", source=src, color=_C["red"], legend_label="fault flags", line_width=1.8, line_dash="dotted")
    _add_zero_line(p_status)
    p_status.add_tools(
        HoverTool(
            mode="vline",
            tooltips=[
                ("t", "@t{0.000}s"),
                ("mode", "@mode{0}"),
                ("control state", "@control_state{0}"),
                ("fault flags", "@fault_flags{0}"),
            ],
        )
    )

    plots = [p_position, p_deflection, p_velocity, p_current, p_environment, p_encoder, p_status]
    for plot in plots:
        plot.legend.click_policy = "hide"
        plot.legend.location = "top_right"

    def reset_ranges() -> None:
        for name, sticky in sticky_ranges.items():
            bokeh_ranges[name].start, bokeh_ranges[name].end = sticky.reset()

    reset_button = Button(label="Reset plot ranges", button_type="primary", width=150, height=30)
    reset_button.on_click(reset_ranges)

    doc.add_root(
        column(
            row(reset_button, sizing_mode="stretch_width", styles={"padding": "8px 10px 0 10px"}),
            *plots,
            sizing_mode="stretch_width",
            styles={"background-color": "#161b22", "padding": "0 10px 12px 10px"},
        )
    )
    doc.theme = _DARK_THEME
    doc.template = _IFRAME_CSS

    def update() -> None:
        samples = ctx_obj.store.snapshot(2000)
        if not samples:
            return

        cal = ctx_obj.calibration
        data = build_telemetry_series(samples, cal.output_per_motor, cal.output_offset_rad)
        src.data = data

        t_values = data["t"]
        _apply_x_range(plots, t_values)
        _apply_y_range(
            sticky_ranges["position"],
            bokeh_ranges["position"],
            data["measured_output_rad"] + data["predicted_output_rad"] + data["target_output_rad"] + data["commanded_output_rad"],
        )
        _apply_y_range(
            sticky_ranges["deflection"],
            bokeh_ranges["deflection"],
            data["deflection_rad"] + data["torque_proxy_rad"] + data["motor_slip_rad"],
        )
        _apply_y_range(
            sticky_ranges["velocity"],
            bokeh_ranges["velocity"],
            data["measured_output_vel_rad_s"] + data["predicted_output_vel_rad_s"] + data["commanded_output_vel_rad_s"],
        )
        _apply_y_range(
            sticky_ranges["current"],
            bokeh_ranges["current"],
            data["driver_current_a"] + data["commanded_current_a"],
        )
        _apply_y_range(
            sticky_ranges["environment"],
            bokeh_ranges["environment"],
            data["bus_voltage_v"] + data["temperature_c"],
        )
        _apply_y_range(
            sticky_ranges["encoder"],
            bokeh_ranges["encoder"],
            data["motor_enc_raw"] + data["output_enc_raw"],
        )
        _apply_y_range(
            sticky_ranges["status"],
            bokeh_ranges["status"],
            data["mode"] + data["fault_flags"] + data["control_state"],
        )

    doc.add_periodic_callback(update, 100)  # 10 Hz


def start_bokeh_server(ctx_obj: Any, port: int = BOKEH_PORT) -> None:
    """Start the Bokeh server in the calling thread. Blocks indefinitely."""
    # Give Tornado its own event loop so it does not conflict with Reflex/uvicorn.
    asyncio.set_event_loop(asyncio.new_event_loop())

    try:
        server = Server(
            {"/": Application(FunctionHandler(lambda doc: _make_document(doc, ctx_obj)))},
            port=port,
            session_token_expiration=3600,
            allow_websocket_origin=[
                f"localhost:{port}",
                "localhost:3000",
                "localhost:3001",
                "localhost:3002",
                "localhost:3003",
                "localhost:3004",
                "localhost:3005",
                "localhost:3006",
                "localhost:8000",
                "localhost:8001",
                "127.0.0.1:3000",
                "127.0.0.1:3001",
                "127.0.0.1:3002",
                "127.0.0.1:3003",
                "127.0.0.1:3004",
                "127.0.0.1:3005",
                "127.0.0.1:3006",
                "127.0.0.1:8000",
                "127.0.0.1:8001",
            ],
        )
    except OSError as exc:
        if exc.errno == errno.EADDRINUSE or getattr(exc, "winerror", None) == 10048:
            return
        raise
    server.start()
    server.io_loop.start()


def launch_bokeh_thread(ctx_obj: Any, port: int = BOKEH_PORT) -> threading.Thread:
    """Spawn the Bokeh server as a daemon thread and return it."""
    t = threading.Thread(
        target=start_bokeh_server,
        args=(ctx_obj, port),
        daemon=True,
        name="bokeh-server",
    )
    t.start()
    return t
