import reflex as rx

config = rx.Config(
    app_name="actuator_gui",
    backend_port=8001,
    frontend_port=3000,
    plugins=[
        rx.plugins.RadixThemesPlugin(
            theme=rx.theme(appearance="dark", accent_color="blue", radius="small"),
        ),
        rx.plugins.SitemapPlugin(),
    ],
)
