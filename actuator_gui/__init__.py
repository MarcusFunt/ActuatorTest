"""Actuator bench Reflex application package."""

# Reflex convention imports ``actuator_gui.actuator_gui`` as the app module.  By
# importing the main app first and the extra page second, the guided page can
# attach itself to the same ``rx.App`` instance without duplicating backend
# connection state.
from .actuator_gui import app as app
from . import characterize_page as characterize_page

__all__ = ["app"]
