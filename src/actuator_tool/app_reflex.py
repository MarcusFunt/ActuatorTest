"""CLI entry-point: launches `reflex run` from the project root."""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path

# Project root is three levels up from this file:  src/actuator_tool/app_reflex.py
_PROJECT_ROOT = Path(__file__).resolve().parents[2]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Actuator bench tool — Reflex/Bokeh web UI")
    target = parser.add_mutually_exclusive_group()
    target.add_argument("--sim", action="store_true", help="preselect the simulator transport")
    target.add_argument("--port", help="preselect a hardware serial port, e.g. COM3")
    parser.add_argument("--prod", action="store_true", help="run in production mode")
    return parser


def main(argv: list[str] | None = None) -> None:
    parser = build_parser()
    args = parser.parse_args(argv)

    cmd = [sys.executable, "-m", "reflex", "run"]
    if args.prod:
        cmd.extend(["--env", "prod"])

    env = os.environ.copy()
    if args.sim:
        env["ACTUATOR_GUI_USE_SIM"] = "1"
        env.pop("ACTUATOR_GUI_PORT", None)
    elif args.port:
        env["ACTUATOR_GUI_USE_SIM"] = "0"
        env["ACTUATOR_GUI_PORT"] = args.port

    print(f"Starting Reflex app from {_PROJECT_ROOT}")
    if args.port:
        print(f"Hardware port preselected: {args.port}")
    elif args.sim:
        print("Simulator transport preselected.")
    print("Open http://localhost:3000 in your browser.")
    subprocess.run(cmd, cwd=_PROJECT_ROOT, check=True, env=env)


if __name__ == "__main__":
    main()
