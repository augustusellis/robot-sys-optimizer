"""Minimum surface radius per tool (v10 T12; core.surface_radius).

``fit_of(s)``: the targets checked against the tool's TCP minimum radii (cached per part mesh,
targets and tool); used by Targets (*Colour part by*, markers), the planners (*Leave out points the
tool cannot fit*: ``skip_ids``), the Results card, the log and the report. ``RadiusFields``: the
Tool tab's fields under TCPs (library tools; packages set them in the builder).
"""

from __future__ import annotations

import hashlib

from core import targets as tg
from core.surface_radius import APPLIES, Fit, check

LABELS = {"concave": "concave (inside curves)", "convex": "convex (outside curves)", "both": "both"}


def _key(s) -> str | None:
    if s.part is None or s.tool is None or s.tool_model is None or not s.targets.points:
        return None
    if not any(t.min_radius_mm for t in s.tool.tcps):
        return None
    h = hashlib.sha1()
    h.update(repr((s.part.mesh_path, s.part.units)).encode())
    h.update(s.targets.model_dump_json().encode())
    h.update(
        repr(
            [
                (t.name, t.min_radius_mm, t.radius_applies, t.contact_links, t.contact_faces, t.T_head_tcp)
                for t in s.tool.tcps
            ]
        ).encode()
    )
    return h.hexdigest()


def fit_of(s) -> Fit:
    """The tool-fit check of the current targets (empty Fit when no TCP has a minimum radius)."""
    key = _key(s)
    if key is None:
        return Fit()
    cache = getattr(s, "_radius_fit", None)
    if cache is not None and cache[0] == key:
        return cache[1]
    ids, P, N, G = tg.as_arrays(s.targets)
    groups = {g.name: g for g in s.targets.groups}
    from core.schema import TargetGroup

    per = [groups.get(g) or TargetGroup(name=g) for g in G]
    fit = check(s.part_mesh(), ids, P, N, per, s.tool_model)
    s._radius_fit = (key, fit)
    return fit


def skip_ids(s) -> list[int]:
    """Point ids the planners leave out (*Leave out points the tool cannot fit* on)."""
    return sorted(fit_of(s).bad) if getattr(s, "skip_unfit", False) else []


def report_md(s) -> str:
    fit = fit_of(s)
    if not fit.bad:
        return f"{fit.lines()}." if fit.unsure else ""
    how = "left out of planning" if getattr(s, "skip_unfit", False) else "planned anyway (only reported)"
    return f"{fit.lines()}. The incompatible points are {how}."


def log_fit(s) -> None:
    """Log the incompatible points (and v12 #4: unmeasured ones) once per targets / tool state."""
    fit = fit_of(s)
    key = getattr(s, "_radius_fit", (None,))[0]
    if (fit.bad or fit.unsure) and getattr(s, "_radius_logged", None) != key:
        from ui.log_panel import log

        s._radius_logged = key
        log(f"Tool fit: {report_md(s)}", "warn")


class RadiusFields:
    """Tool tab -> TCPs: *Minimum surface radius (mm)* (0 = any) and *Radius limit applies to*."""

    def __init__(self, gui, tab):
        self.tab = tab
        self._busy = False
        self.radius = gui.add_number(
            "Minimum surface radius (mm)",
            0.0,
            min=0.0,
            max=10000.0,
            step=1.0,
            hint="the tightest curve this TCP can follow; 0 = any",
        )
        self.applies = gui.add_dropdown("Radius limit applies to", tuple(LABELS[a] for a in APPLIES))
        self.radius.on_update(lambda e: self._commit(e))
        self.applies.on_update(lambda e: self._commit(e))

    def show(self, tcp) -> None:
        self._busy = True
        try:
            self.radius.value = float(tcp.min_radius_mm or 0.0) if tcp is not None else 0.0
            self.applies.value = LABELS[tcp.radius_applies if tcp is not None else "concave"]
        finally:
            self._busy = False

    def _commit(self, e) -> None:
        if self._busy or self.tab.s.tool is None:
            return
        cur = self.tab.tcp_choice.value
        r = float(self.radius.value)
        kind = next(a for a, lab in LABELS.items() if lab == self.applies.value)
        self.tab._commit(
            lambda d: self.tab._tcp(d, cur).update(min_radius_mm=r if r > 0 else None, radius_applies=kind), e
        )
