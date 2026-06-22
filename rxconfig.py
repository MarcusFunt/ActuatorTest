import os

import reflex as rx


def _port(name: str, default: int) -> int:
    try:
        return int(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default


_backend_port = _port("REFLEX_BACKEND_PORT", 8001)
_frontend_port = _port("REFLEX_FRONTEND_PORT", 3000)
_backend_host = os.environ.get("REFLEX_BACKEND_HOST", "localhost")
_frontend_host = os.environ.get("REFLEX_FRONTEND_HOST", "localhost")

config = rx.Config(
    app_name="actuator_gui",
    backend_port=_backend_port,
    frontend_port=_frontend_port,
    api_url=os.environ.get("REFLEX_API_URL", f"http://{_backend_host}:{_backend_port}"),
    deploy_url=os.environ.get("REFLEX_DEPLOY_URL", f"http://{_frontend_host}:{_frontend_port}"),
    cors_allowed_origins=["*"],
    plugins=[
        rx.plugins.RadixThemesPlugin(
            theme=rx.theme(appearance="dark", accent_color="blue", radius="small"),
        ),
        rx.plugins.SitemapPlugin(),
    ],
)
