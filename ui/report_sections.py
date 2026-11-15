"""Report sections of v10 T22 (ui.export_ui.ExportPanel.sections): the setup comparison with the
project's setup always in it and a plan-view picture per setup, and the gantry placement results."""

from __future__ import annotations

import numpy as np

from core.report import md_html
from core.report_svg import plan_view, stops_from_text, stops_text


def _outline(s) -> np.ndarray:
    """World xy of the part's floor footprint (convex hull), or nothing."""
    if s.part is None:
        return np.zeros((0, 2))
    from core.standoff import footprint

    try:
        fp = footprint(s.part_mesh(), s.part.T_world_part)
    except (OSError, ValueError):
        return np.zeros((0, 2))
    return np.asarray(fp.exterior.coords)[:-1] if fp is not None and not fp.is_empty else np.zeros((0, 2))


def comparison_rows(s, sol) -> list[dict]:
    """The rows of Compare setups, the project's first. Without a stored project row (a comparison
    from before v10), the project's selected plan stands in for it."""
    own = [dict(c["metrics"], setup=k) for k, c in s.compared.items() if c.get("metrics", {}).get("project")]
    others = [
        dict(c.get("metrics", {}), setup=k)
        for k, c in s.compared.items()
        if not c.get("metrics", {}).get("project")
    ]
    if not own and sol is not None:
        robot = (s.view_stash or {}).get("robot_name", s.robot_name)
        n = len(s.targets.points)
        own = [
            {
                "setup": f"{robot} (project, selected plan)",
                "stations": len(sol.stations),
                "reached": n - len(sol.unreached) - len(sol.unfit),
                "cycle_s": sol.cycle_s,
                "cost": sol.cost,
                "stops_xy": stops_text(sol.stations),
                "project": True,
            }
        ]
    for r in others:
        if not r.get("stops_xy"):
            c = s.compared.get(r["setup"], {})
            if c.get("results"):
                r["stops_xy"] = stops_text(c["results"][0].stations)
    return own + others


def setup_comparison(s, sol) -> str:
    from ui.compare import table

    rows = comparison_rows(s, sol)
    n = len(s.targets.points)
    note = (
        "Each setup planned the same study (same part, targets, carrier and planner settings); its best plan "
        "is listed, the project's own setup first. The pictures show the part's floor outline and each "
        "setup's stops (robot base positions, in visiting order)."
    )
    outline = _outline(s)
    pics = "".join(
        plan_view(outline, stops_from_text(r.get("stops_xy", "")), str(r["setup"]))
        for r in rows
        if r.get("stops_xy")
    )
    html = md_html(table([{**r, "robot": r["setup"]} for r in rows], n) + "\n\n" + note)
    return html + (f'<div style="display:flex;flex-wrap:wrap">{pics}</div>' if pics else "")


def gantry_placement(s, rows: list[dict]) -> str:
    from ui.frame_ui import table

    n = len(s.targets.points)
    note = (
        "Optimize → Gantry placement: candidate frames scored by the reachability map, the best ones "
        "(and the refined one) planned in full with the station planner's settings; cheapest first."
    )
    return md_html(table(rows, n) + "\n\n" + note)
