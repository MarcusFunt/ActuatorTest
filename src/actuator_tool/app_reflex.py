"""CLI entry-point: launches `reflex run` from the project root."""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path


def _find_project_root() -> Path:
    """Find the checkout root that contains the Reflex config."""
    for parent in Path(__file__).resolve().parents:
        if (parent / "rxconfig.py").is_file():
            return parent
    return Path.cwd()


_PROJECT_ROOT = _find_project_root()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Actuator bench tool - Reflex/Bokeh web UI")
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
        cmd.extend(
            [
                "--env",
                "prod",
                "--frontend-port",
                "3000",
                "--backend-port",
                "3000",
                "--single-port",
            ]
        )

    env = os.environ.copy()
    if args.sim or not args.port:
        env["ACTUATOR_GUI_USE_SIM"] = "1"
        env.pop("ACTUATOR_GUI_PORT", None)
    elif args.port:
        env["ACTUATOR_GUI_USE_SIM"] = "0"
        env["ACTUATOR_GUI_PORT"] = args.port

    print(f"Starting Reflex app from {_PROJECT_ROOT}")
    if args.port:
        print(f"Hardware port preselected: {args.port}")
    else:
        print("Simulator transport preselected.")
    print("Bench:       http://localhost:3000/")
    print("Characterize: http://localhost:3000/characterize")
    subprocess.run(cmd, cwd=_PROJECT_ROOT, check=True, env=env)


if __name__ == "__main__":
    main()
