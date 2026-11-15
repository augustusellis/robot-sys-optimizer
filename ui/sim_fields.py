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
    panel.motion_mode = gui.add_dropdown(  # v10 T16
        "Carrier motion",
        MOTION,
        hint="rail / gantry: stop and go, or the carrier moving with the tool in continuous passes",
    )
    panel.turn_lift = gui.add_checkbox(  # v10 T15
        "Lift on wide turns",
        False,
        hint="row patterns: lift off for row changes longer than the tool's footprint diameter",
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


def group_patterns(panel, pattern):
    """(pattern, pattern_of) for core.paths.trace (v10 T15): *Lift on wide turns* sizes the row
    patterns' turn lifts to the tool's footprint; a target group with its own pattern gets it with
    the panel's other settings."""
    from dataclasses import replace

    from core.patterns import PatternSettings
    from core.surface_radius import footprint_of

    s = panel.s
    turn = 0.0
    if panel.turn_lift.value and s.tool_model is not None and s.tool is not None:
        turn = 2 * footprint_of(s.tool_model, s.tool.tcps[0].name, 0.025)
    if pattern is not None:
        pattern = replace(pattern, turn_lift_m=turn)
    own = {g.name: g.pattern for g in s.targets.groups if g.pattern}
    if not own:
        return pattern, None
    base = pattern or replace(PatternSettings(), turn_lift_m=turn)
    return pattern, lambda name: replace(base, pattern=own[name]) if name in own else None


def spin_of(panel, res):
    """v10 #9: the allowed spin per point for core.path_spin (None: the planned spins)."""
    if not panel.spin_opt.value:
        return None
    from core.path_spin import allowed_from

    s = panel.s
    groups = {g.name: g for g in s.targets.groups}
    return allowed_from(groups, res, {p.id: p.group for p in s.targets.points})


MOTION = ("Uncoordinated (stop and go)", "Coordinated (the carrier follows the tool)")  # v10 T16


def coordinated(panel) -> bool:
    """Coordinated carrier motion chosen, and the carrier can do it (a movable rail or gantry)."""
    from core.coordinated import KINDS

    b = panel.s.base
    return panel.motion_mode.value == MOTION[1] and b.mode == "movable" and b.carrier in KINDS


def show_motion(panel) -> None:
    b = panel.s.base
    panel.motion_mode.visible = b.mode == "movable" and b.carrier in ("rail", "gantry")


def keep_cycle(panel, cycle: dict) -> None:
    """The built paths' cycle time per (plan label, carrier motion), for the report."""
    mode = "coordinated" if coordinated(panel) else "uncoordinated"
    if not hasattr(panel, "cycles"):
        panel.cycles = {}
    panel.cycles[(panel.label, mode)] = dict(cycle)


def motion_line(panel, label: str) -> str:
    """'Coordinated carrier motion: 72 s, stop and go: 83 s (-12 %).' when both were built."""
    c = getattr(panel, "cycles", {})
    a, b = c.get((label, "coordinated")), c.get((label, "uncoordinated"))
    if a is None or b is None:
        return ""
    d = 100 * (a["total"] - b["total"]) / max(b["total"], 1e-9)
    return (
        f"Carrier motion: coordinated **{a['total']:.0f} s**, stop and go **{b['total']:.0f} s** "
        f"({d:+.0f} %; carrier moves {a.get('carrier', 0):.0f} / {b.get('carrier', 0):.0f} s, "
        f"settle {a.get('settle', 0):.0f} / {b.get('settle', 0):.0f} s)."
    )


def direction_of(s):
    """v11 #7: ``direction(tcp)`` for core.paths.trace (ui.travel_ui)."""
    from ui.travel_ui import direction_of as of

    return of(s)
