"""Optimize -> Cycle time (v8 T6): the program speeds behind the cycle-time estimate (planner
cost term, Results card) and the built paths' cycle time. Kept on the session (``s.cycle``)
and in the saved panel settings."""

from __future__ import annotations

import math

import viser

from core.cycle_time import CycleSettings

# v10 T14: label -> CycleSettings.timing
TIMING = {
    "Time-optimal (speed, acceleration, jerk limits)": "topp",
    "Trapezoid per move (pre-v10)": "trapezoid",
}

# (attribute, label, GUI unit factor (GUI = SI * f), step, hint)
FIELDS = (
    (
        "joint_speed",
        "Joint speed (°/s)",
        180 / math.pi,
        5.0,
        "program speed of joint moves; capped per joint",
    ),
    ("joint_acc", "Joint acceleration (°/s²)", 180 / math.pi, 5.0, ""),
    ("joint_jerk", "Joint jerk (°/s³)", 180 / math.pi, 50.0, "time-optimal timing only; 0 = no jerk limit"),
    ("process_speed", "Process speed (mm/s)", 1000.0, 5.0, "TCP speed along the surface"),
    ("approach_speed", "Approach speed (mm/s)", 1000.0, 10.0, "lifts, approach and retract"),
    ("linear_acc", "Linear acceleration (mm/s²)", 1000.0, 100.0, ""),
    ("carrier_speed", "Carrier speed (mm/s)", 1000.0, 10.0, "rail / gantry axes / mobile base"),
    ("carrier_acc", "Carrier acceleration (mm/s²)", 1000.0, 50.0, ""),
    ("lift_speed", "Lift speed (mm/s)", 1000.0, 10.0, ""),
    (
        "settle_s",
        "Settle time per stop (s)",
        1.0,
        0.5,
        "carrier settling / localization before the arm starts",
    ),
)


class CyclePanel:
    def __init__(self, server: viser.ViserServer, session):
        self.s = session
        d = CycleSettings()
        gui = server.gui
        self.folder = gui.add_folder("Cycle time", expand_by_default=False)
        with self.folder:
            gui.add_markdown(
                "_Speeds of the robot program, used for the **estimated cycle time** of each plan (Results "
                "card; cost term *Cost per minute of cycle time*) and the cycle time of built paths._"
            )
            self.timing = gui.add_dropdown(  # v10 T14
                "Path timing",
                tuple(TIMING),
                initial_value=next(k for k, v in TIMING.items() if v == d.timing),
                hint="how built paths are timed (the plans' estimate is always per move)",
            )
            self.timing.on_update(lambda _e: self.commit())
            for key, label, f, step, hint in FIELDS:
                h = gui.add_number(
                    label, round(getattr(d, key) * f, 3), min=0.0, step=step, hint=hint or None
                )
                h.on_update(lambda _e: self.commit())
                setattr(self, key, h)
        self.commit()

    def settings(self) -> CycleSettings:
        vals = {key: float(getattr(self, key).value) / f for key, _l, f, _s, _h in FIELDS}
        return CycleSettings(**vals, timing=TIMING.get(self.timing.value, "topp"))

    def commit(self) -> CycleSettings:
        self.s.cycle = self.settings()
        return self.s.cycle
