"""Four decimal places in every mm / deg field (v12 #28).

viser shows (and rounds a typed value to) as many decimals as a field's arrow step has: a pose
field with a 1 mm step took 12.3456 as 12. ``precise(server)`` wraps the server's ``add_number``,
``add_vector2``, ``add_vector3`` and ``add_slider`` so every field whose label names mm or deg gets
``DIGITS`` decimals (viser's ``precision``, set apart from the step: the arrows keep their steps),
also fields made later (pose widgets, carrier sliders, TCP rows). Applied by app.build_app and
the builder window.
"""

from __future__ import annotations

import functools
import re

DIGITS = 4
UNITS = re.compile(r"\((?:[^)]*\b)?(mm|deg)\b")  # "xyz (mm)", "J1 (deg)", "Cylinder r / h (mm)"
METHODS = ("add_number", "add_vector2", "add_vector3", "add_slider")


def wants(label: str) -> bool:
    return bool(UNITS.search(label or ""))


def precise(server, digits: int = DIGITS) -> None:
    """Make every mm / deg field ``server`` creates from now on show ``digits`` decimals."""
    gui = server.gui
    if getattr(gui, "_rso_precise", False):
        return
    for name in METHODS:
        add = getattr(gui, name)

        @functools.wraps(add)
        def wrapped(label, *args, _add=add, **kwargs):
            h = _add(label, *args, **kwargs)
            if wants(label) and hasattr(h, "precision") and h.precision < digits:
                h.precision = digits
            return h

        setattr(gui, name, wrapped)
    gui._rso_precise = True
