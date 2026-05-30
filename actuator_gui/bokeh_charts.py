"""Bokeh live-chart server — runs in a daemon thread, served at http://localhost:5006."""

from __future__ import annotations

import asyncio
import errno
import threading
from typing import TYPE_CHECKING, Any

from bokeh.application import Application
from bokeh.application.handlers.function import FunctionHandler
from bokeh.layouts import column
from bokeh.models import ColumnDataSource
from bokeh.plotting import figure
from bokeh.server.server import Server
from bokeh.themes import Theme
from jinja2 import Template

if TYPE_CHECKING:
    from actuator_tool.actuator_data import TelemetryStore

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
            "Grid": {"grid_line_color": "#21262d", "grid_line_alpha": 0.85},
            "Axis": {
                "axis_line_color": "#30363d",
                "major_tick_line_color": "#30363d",
                "minor_tick_line_color": "#21262d",
                "major_label_text_color": "#8b949e",
                "axis_label_text_color": "#8b949e",
                "major_label_text_font": "JetBrains Mono",
                "axis_label_text_font": "Space Grotesk",
            },
            "Title": {
                "text_color": "#e6edf3",
                "text_font_size": "12px",
                "text_font": "Space Grotesk",
            },
            "Legend": {
                "background_fill_color": "#161b22",
                "background_fill_alpha": 0.85,
                "label_text_color": "#e6edf3",
                "label_text_font": "Space Grotesk",
                "border_line_color": "#30363d",
            },
        }
    }
)

_C = ["#2f81f7", "#3fb950", "#f85149", "#d29922", "#f0883e", "#8b949e"]


_IFRAME_CSS = Template("""
{% extends base %}
{% block preamble %}
<style>
html, body {
    margin: 0 !important;
    padding: 0 !important;
    height: 100%;
    overflow: hidden;
    background: #161b22;
}
</style>
{% endblock %}
""")


def _make_document(doc: Any, ctx_obj: Any) -> None:
    """Build a Bokeh document with four live-updating charts."""
    from actuator_tool.actuator_analysis import compute_deflection, predicted_output

    src_ang = ColumnDataSource(dict(t=[], motor=[], output=[], predicted=[]))
    src_def = ColumnDataSource(dict(t=[], defl=[]))
    src_vel = ColumnDataSource(dict(t=[], mvel=[], ovel=[]))
    src_cmd = ColumnDataSource(dict(t=[], cmd=[], meas=[]))

    kw: dict[str, Any] = dict(
        sizing_mode="stretch_both",
        tools="pan,box_zoom,wheel_zoom,reset,save",
        toolbar_location="right",
        x_range=(0.0, 1.0),
        y_range=(-1.0, 1.0),
    )

    p_ang = figure(title="Angles", min_height=200, x_axis_label="Time [s]", y_axis_label="rad", **kw)
    p_ang.line("t", "motor",     source=src_ang, color=_C[0], legend_label="motor",     line_width=1.5)
    p_ang.line("t", "output",    source=src_ang, color=_C[1], legend_label="output",    line_width=1.5)
    p_ang.line("t", "predicted", source=src_ang, color=_C[5], legend_label="predicted", line_width=1, line_dash="dashed")
    p_ang.legend.click_policy = "hide"

    p_def = figure(title="Deflection", min_height=115, x_axis_label="Time [s]", y_axis_label="rad", **kw)
    p_def.line("t", "defl", source=src_def, color=_C[2], line_width=2)

    p_vel = figure(title="Velocity", min_height=115, x_axis_label="Time [s]", y_axis_label="rad/s", **kw)
    p_vel.line("t", "mvel", source=src_vel, color=_C[0], legend_label="motor vel", line_width=1.5)
    p_vel.line("t", "ovel", source=src_vel, color=_C[1], legend_label="output vel", line_width=1.5)
    p_vel.legend.click_policy = "hide"

    p_cmd = figure(title="Command vs Measured", min_height=115, x_axis_label="Time [s]", y_axis_label="rad", **kw)
    p_cmd.line("t", "cmd",  source=src_cmd, color=_C[4], legend_label="cmd pos",      line_width=1.5)
    p_cmd.line("t", "meas", source=src_cmd, color=_C[0], legend_label="measured pos", line_width=1, line_dash="dashed")
    p_cmd.legend.click_policy = "hide"

    for plot in (p_ang, p_def, p_vel, p_cmd):
        plot.toolbar.logo = None
        plot.toolbar.autohide = True
        plot.min_border = 8

    doc.add_root(
        column(
            p_ang,
            p_def,
            p_vel,
            p_cmd,
            sizing_mode="stretch_both",
            styles={"background-color": "#161b22"},
        )
    )
    doc.theme = _DARK_THEME
    doc.template = _IFRAME_CSS

    def padded_range(values: list[float], fallback: tuple[float, float]) -> tuple[float, float]:
        if not values:
            return fallback
        lo = min(values)
        hi = max(values)
        if lo == hi:
            spread = max(abs(lo), 1.0) * 0.5
        else:
            spread = (hi - lo) * 0.08
        return lo - spread, hi + spread

    def apply_ranges(plot: Any, x_values: list[float], y_values: list[float]) -> None:
        plot.x_range.start, plot.x_range.end = padded_range(x_values, (0.0, 1.0))
        plot.y_range.start, plot.y_range.end = padded_range(y_values, (-1.0, 1.0))

    def update() -> None:
        samples = ctx_obj.store.snapshot(2000)
        if not samples:
            return
        cal = ctx_obj.calibration
        if len(samples) > _MAX_POINTS:
            stride = max(1, len(samples) // _MAX_POINTS)
            samples = samples[::stride]
        base = samples[0].t_us
        t = [(s.t_us - base) / 1_000_000.0 for s in samples]
        motor = [s.motor_rad for s in samples]
        output = [s.output_rad for s in samples]
        predicted = predicted_output(samples, cal.output_per_motor, cal.output_offset_rad).tolist()
        deflection = compute_deflection(samples, cal.output_per_motor, cal.output_offset_rad).tolist()
        motor_vel = [s.motor_vel_rad_s for s in samples]
        output_vel = [s.output_vel_rad_s for s in samples]
        cmd = [s.cmd_pos for s in samples]
        src_ang.data = dict(
            t=t,
            motor=motor,
            output=output,
            predicted=predicted,
        )
        src_def.data = dict(t=t, defl=deflection)
        src_vel.data = dict(
            t=t,
            mvel=motor_vel,
            ovel=output_vel,
        )
        src_cmd.data = dict(
            t=t,
            cmd=cmd,
            meas=motor,
        )
        apply_ranges(p_ang, t, motor + output + predicted)
        apply_ranges(p_def, t, deflection)
        apply_ranges(p_vel, t, motor_vel + output_vel)
        apply_ranges(p_cmd, t, cmd + motor)

    doc.add_periodic_callback(update, 100)  # 10 Hz


def start_bokeh_server(ctx_obj: Any, port: int = BOKEH_PORT) -> None:
    """Start the Bokeh server in the calling thread. Blocks indefinitely."""
    # Give Tornado its own event loop so it doesn't conflict with Reflex/uvicorn.
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
