import os

import reflex as rx


def _port(name: str, default: int) -> int:
    try:
        return int(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default


def _loopback_host(name: str) -> str:
    host = os.environ.get(name, "127.0.0.1").strip().lower()
    if host not in {"localhost", "127.0.0.1", "::1"}:
        raise RuntimeError(f"{name} must be a loopback address for the bench tool")
    return host


_backend_port = _port("REFLEX_BACKEND_PORT", 8001)
_frontend_port = _port("REFLEX_FRONTEND_PORT", 3000)
_backend_host = _loopback_host("REFLEX_BACKEND_HOST")
_frontend_host = _loopback_host("REFLEX_FRONTEND_HOST")

# RadixThemesPlugin was removed in newer Reflex releases.  Keeping it when
# available preserves the intended dark theme on older supported versions,
# while allowing the unconstrained ``reflex>=0.6.0`` dependency to start.
_plugins = []
if hasattr(rx.plugins, "RadixThemesPlugin"):
    _plugins.append(
        rx.plugins.RadixThemesPlugin(
            theme=rx.theme(appearance="dark", accent_color="blue", radius="small"),
        )
    )
_plugins.append(rx.plugins.SitemapPlugin())

config = rx.Config(
    app_name="actuator_gui",
    backend_port=_backend_port,
    frontend_port=_frontend_port,
    api_url=os.environ.get("REFLEX_API_URL", f"http://{_backend_host}:{_backend_port}"),
    deploy_url=os.environ.get("REFLEX_DEPLOY_URL", f"http://{_frontend_host}:{_frontend_port}"),
    cors_allowed_origins=[
        f"http://localhost:{_frontend_port}",
        f"http://127.0.0.1:{_frontend_port}",
    ],
    plugins=_plugins,
)
