"""Results -> Paths & simulation: the path option fields (split from ui.simulate in v10 task 9).
``panel`` is the SimPanel; the widgets become its attributes."""

from __future__ import annotations

BETWEEN_LABELS = {"Collision-free joint path": "direct", "Via home": "home"}
PLANNER_ORDER = "Planner order"


def build(panel, gui) -> None:
    from core.patterns import CORNERS, PATTERNS
    from ui.method_picker import MethodPicker

    panel.pattern = gui.add_dropdown("Pattern", (*PATTERNS, PLANNER_ORDER), initial_value="Zigzag")
    panel.angle = gui.add_number("Raster angle (deg)", 0.0, min=-180.0, max=180.0, step=5.0)
    panel.corner = gui.add_dropdown("Start corner", CORNERS, hint="<along rows>-<across rows>")
    panel.row_tol = gui.add_number("Row tolerance (mm)", 0.0, min=0.0, step=1.0, hint="0 = automatic")
    panel.inward = gui.add_checkbox("Outside → in", True)
    panel.no_cross = gui.add_checkbox("Prevent crossing paths", False, hint="lift off for moves that cross")
    panel.lift = gui.add_number("Lift height (mm)", 30.0, min=1.0, step=5.0)
    panel.home = gui.add_checkbox(
        "Home between stops", True, hint="arm to home before the carrier moves, back after"
    )
    panel.between = gui.add_dropdown(  # keys of core.paths.BETWEEN
        "Between groups", tuple(BETWEEN_LABELS), hint="to the next group / configuration at a stop"
    )
    panel.drive = gui.add_checkbox(
        "Drive carrier between stops",
        True,
        hint="plan the mobile base / external axis moves between stops (collision checked)",
    )
    panel.order = MethodPicker(gui, "point_order", "Point order")  # v8 T14
    panel.motion = gui.add_checkbox(  # v8 T7
        "Check moves between points",
        True,
        hint="collision check every 2° of joint motion between samples (flags 'motion')",
    )
    panel.spin_opt = gui.add_checkbox(  # v10 #9
        "Optimize tool spin",
        True,
        hint="where the spin is free (or a range): choose it along the path for least joint motion",
    )
    panel.smooth = gui.add_checkbox(  # v10 T13
        "Smooth joint moves",
        True,
        hint="optimise the planned moves between points: smoother, shorter, further from the part",
    )
    for w in (panel.pattern, panel.no_cross):
        w.on_update(lambda e: show(panel))
    show(panel)


def show(panel) -> None:
    from core.patterns import ROW_PATTERNS

    p = panel.pattern.value
    on = p != PLANNER_ORDER
    panel.angle.visible = panel.corner.visible = panel.no_cross.visible = on
    panel.row_tol.visible = p in (*ROW_PATTERNS, "Concentric", "Spiral")
    panel.inward.visible = p in ("Concentric", "Spiral")
    panel.lift.visible = on and panel.no_cross.value


def pattern_settings(panel):
    from core.patterns import PatternSettings

    if panel.pattern.value == PLANNER_ORDER:
        return None
    return PatternSettings(
        panel.pattern.value,
        panel.angle.value,
        panel.corner.value,
        panel.row_tol.value / 1000,
        panel.inward.value,
        panel.no_cross.value,
    )
