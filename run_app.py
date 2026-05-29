"""One-command launcher for a local checkout of the actuator bench tool."""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parent
VENV_DIR = ROOT / ".venv"
MIN_PYTHON = (3, 11)
REQUIRED_MODULES = (
    "actuator_tool",
    "reflex",
    "bokeh",
    "serial",
    "numpy",
    "scipy",
    "matplotlib",
    "pytest",
)


def _venv_python() -> Path:
    if os.name == "nt":
        return VENV_DIR / "Scripts" / "python.exe"
    return VENV_DIR / "bin" / "python"


def _same_python(candidate: Path) -> bool:
    try:
        return Path(sys.executable).resolve() == candidate.resolve()
    except OSError:
        return False


def _print_help() -> None:
    print(
        """usage: python run_app.py [--sim | --port PORT] [--prod]

Bootstraps .venv if needed, installs this project in editable mode, then
launches the Reflex/Bokeh actuator bench tool.

options:
  --sim        Preselect the simulator transport. This is the default.
  --port PORT  Preselect a hardware serial port, e.g. COM3.
  --prod       Run Reflex in production mode.
  -h, --help   Show this help message and exit.
"""
    )


def _check_python_version() -> None:
    if sys.version_info < MIN_PYTHON:
        required = ".".join(str(part) for part in MIN_PYTHON)
        current = ".".join(str(part) for part in sys.version_info[:3])
        raise SystemExit(
            f"Python {required} or newer is required; current Python is {current}."
        )


def _run(
    cmd: list[str | Path],
    *,
    check: bool = True,
    echo: bool = True,
) -> subprocess.CompletedProcess[str]:
    if echo:
        printable = " ".join(str(part) for part in cmd)
        print(f"> {printable}")
    return subprocess.run(
        [str(part) for part in cmd],
        cwd=ROOT,
        check=check,
        text=True,
    )


def _create_venv(python: Path) -> None:
    if python.exists():
        return
    print(f"Creating virtual environment at {VENV_DIR}")
    _run([sys.executable, "-m", "venv", VENV_DIR])


def _modules_available(python: Path) -> bool:
    module_list = ", ".join(repr(name) for name in REQUIRED_MODULES)
    code = (
        "import importlib.util, sys\n"
        f"missing = [name for name in ({module_list},) "
        "if importlib.util.find_spec(name) is None]\n"
        "if missing:\n"
        "    print('Missing modules: ' + ', '.join(missing))\n"
        "    sys.exit(1)\n"
    )
    return _run([python, "-c", code], check=False, echo=False).returncode == 0


def _install_project(python: Path) -> None:
    print("Installing actuator bench tool and dependencies")
    _run([python, "-m", "pip", "install", "-e", ".[dev]"])


def _bootstrap(argv: list[str]) -> int:
    python = _venv_python()
    _create_venv(python)
    if not _modules_available(python):
        _install_project(python)
    return _run([python, Path(__file__).resolve(), *argv], check=False).returncode


def main(argv: list[str] | None = None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    if any(arg in {"-h", "--help"} for arg in args):
        _print_help()
        return 0
    _check_python_version()

    python = _venv_python()
    if not _same_python(python):
        return _bootstrap(args)

    if not _modules_available(python):
        _install_project(python)

    from actuator_tool.app_reflex import main as run_app

    run_app(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
