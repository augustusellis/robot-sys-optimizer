"""Tool tab -> TCPs: directional travel (v11 task 7; core.path_direction).

*Travel direction* (free / along TCP x / along TCP y / drum: x across travel), *Scan width (mm)*,
*Overlap (%)* and *Overrun past edges (mm)* per TCP; the row spacing they need and a button that
writes it into Targets -> Generate points (row spacing of the grids, step-over of the rasters).
"""

from __future__ import annotations

from core.path_direction import Direction, row_pitch

LABELS = {
    "free": "free (spins about the normal)",
    "along_x": "along TCP x (scanner)",
    "along_y": "along TCP y (scanner)",
    "perpendicular_x": "TCP x across travel (drum)",
}


def direction_of(session):
    """``direction(tcp)`` for core.paths.trace from the session's tool (None without a rule)."""
    tool = session.tool
    if tool is None:
        return None
    by = {t.name: t for t in tool.tcps}
    if all(t.travel == "free" for t in by.values()):
        return None
    return lambda name: (
        Direction(by[name].travel, by[name].overrun_mm / 1000)
        if name in by and by[name].travel != "free"
        else None
    )


class TravelFields:
    def __init__(self, gui, tab):
        self.tab = tab
        self.targets = None  # app.py: the Targets tab (row spacing fields)
        self._busy = False
        self.travel = gui.add_dropdown("Travel direction", tuple(LABELS.values()))
        self.width = gui.add_number(
            "Scan width (mm)",
            0.0,
            min=0.0,
            max=2000.0,
            step=1.0,
            hint="the stripe one pass covers; 0 = not set",
        )
        self.overlap = gui.add_number("Overlap (%)", 10.0, min=0.0, max=90.0, step=1.0)
        self.overrun = gui.add_number(
            "Overrun past edges (mm)",
            0.0,
            min=0.0,
            max=1000.0,
            step=5.0,
            hint="rows run this far off the part",
        )
        self.pitch = gui.add_markdown("")
        self.use = gui.add_button("Use for the target rows", hint="Targets: row spacing / raster step-over")
        for w in (self.travel, self.width, self.overlap, self.overrun):
            w.on_update(lambda e: self._commit(e))
        self.use.on_click(lambda e: self.use_pitch(e))

    def show(self, tcp) -> None:
        self._busy = True
        try:
            self.travel.value = LABELS[tcp.travel if tcp is not None else "free"]
            self.width.value = float(tcp.scan_width_mm or 0.0) if tcp is not None else 0.0
            self.overlap.value = float(tcp.overlap_pct) if tcp is not None else 10.0
            self.overrun.value = float(tcp.overrun_mm) if tcp is not None else 0.0
        finally:
            self._busy = False
        self._pitch_text()

    def _pitch_text(self) -> None:
        w = float(self.width.value)
        self.use.visible = w > 0
        self.pitch.content = (
            f"_Rows {row_pitch(w, float(self.overlap.value)):.1f} mm apart cover the surface with "
            f"{self.overlap.value:g} % overlap._"
            if w > 0
            else ""
        )

    def _commit(self, e) -> None:
        if self._busy or self.tab.s.tool is None:
            return
        cur = self.tab.tcp_choice.value
        kind = next(k for k, lab in LABELS.items() if lab == self.travel.value)
        w = float(self.width.value)
        upd = {
            "travel": kind,
            "scan_width_mm": w if w > 0 else None,
            "overlap_pct": float(self.overlap.value),
            "overrun_mm": float(self.overrun.value),
        }
        self.tab._commit(lambda d: self.tab._tcp(d, cur).update(**upd), e)
        self._pitch_text()

    def use_pitch(self, e=None) -> float:
        """Write the row spacing into Targets -> Generate points; returns it (mm)."""
        from ui.widgets import notify

        w = float(self.width.value)
        if w <= 0 or self.targets is None:
            notify(e, "No scan width", "Set the scan width of this TCP first")
            return 0.0
        p = row_pitch(w, float(self.overlap.value))
        self.targets.row.value = p
        self.targets.stepover.value = p
        notify(e, "Row spacing set", f"{p:.1f} mm (Targets → Generate points)", ok=True)
        return p
