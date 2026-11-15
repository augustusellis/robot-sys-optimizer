"""Compare setups -> External axes (v11 task 8): the same part and points planned on other external
axis types: a static arm (one floor base), a rail, a floor XY gantry, an overhead gantry, a mobile
base (with or without a lift), and the carrier packages of the project (package builder).

The axis types are the shipped examples' carriers (scripts/make_examples.py: primitives only, so
they work in any project). A rail or a gantry is first placed relative to this part with the
gantry placement scoring (opt.frame_place.rank_frames: the best-scoring grid frame, no full plans;
an overhead gantry also tries lower beam heights, down to ``OVERHEAD_GAP`` above the part, its legs
stretching), so every axis type gets a fair frame; a mobile base drives anywhere and needs none;
the static arm gets one base placed by the station planner (max stations 1).
"""

from __future__ import annotations

import json
from pathlib import Path

from core.schema import Base

ROOT = Path(__file__).resolve().parents[1]
EXAMPLES = {
    "Rail": "rail_long_part",
    "Floor XY gantry": "floor_xy_rails",
    "Overhead gantry": "gantry_overhead",
    "Mobile base": "mobile_amr",
    "Mobile base with lift": "mobile_lift",
}
STATIC = "Static arm (one base)"


def axis_choices(s) -> dict[str, Base]:
    """{name: Base} of the axis types to compare (not the project's own: it is always planned)."""
    out = {STATIC: s.base.model_copy(update={"mode": "static"})}
    for name, ex in EXAMPLES.items():
        p = ROOT / "examples" / ex / "project.json"
        if p.is_file():
            out[name] = Base.model_validate(json.loads(p.read_text(encoding="utf-8"))["base"])
    pk = s.project_dir / "assets" / "packages"
    for y in sorted(pk.glob("*/package.yaml")) if pk.is_dir() else []:
        if y.parent.name.startswith("_"):
            continue
        try:
            from core.urdf_package import read_urdf, to_base

            urdf = next(y.parent.glob("*.urdf"))
            pkg = read_urdf(urdf)
            if pkg.kind in ("rail", "gantry", "mobile"):
                meshes = s.relative(urdf.parent / "meshes")
                out[f"{pkg.name} (package)"] = to_base(pkg, meshes, s.base)
        except (StopIteration, OSError, ValueError):
            continue
    return out


OVERHEAD_GAP = 1.2  # m: an overhead gantry's lowest beam height above the part's top


def part_top(s) -> float:
    """World z of the part's highest point."""
    m = s.part_mesh()
    T = s.part.T_world_part
    return float((m.vertices @ T[:3, :3].T + T[:3, 3])[:, 2].max())


def kind_of(b: Base) -> str:
    return b.carrier if b.mode == "movable" else "static"


def place(s, base: Base, rmap, cancelled=None) -> tuple[Base, str]:
    """``base`` placed for the session's part: (placed base, note). Rails and gantries: the best
    gantry-placement grid frame; other kinds as they are."""
    if base.mode != "movable" or base.carrier not in ("rail", "gantry"):
        return base, ""
    from core.standoff import footprint
    from opt.frame_place import FrameSettings, frame_text, rank_frames
    from ui.stations import world_targets

    fs = FrameSettings(top=1, refine=False)
    top = part_top(s)
    if base.carrier == "gantry" and base.pose.xyz[2] > top + OVERHEAD_GAP:
        # an overhead gantry: its beams may come down over this part (the legs stretch, v10 T18)
        fs.beam_lo, fs.beam_hi, fs.beam_steps = top + OVERHEAD_GAP, float(base.pose.xyz[2]), 4
    fp = footprint(s.part_mesh(), s.part.T_world_part)
    ranked = rank_frames(
        base, rmap, world_targets(s), s.part_mesh(), s.part.T_world_part, fp, fs,
        resolve=s.resolve, obstacles=s.obstacles(), cancelled=cancelled,
    )  # fmt: skip
    good = [f for f in ranked if not f.reason]
    if not good:
        why = ranked[0].reason if ranked else "no candidate frame"
        return base, f"not placed ({why})"
    return good[0].base, frame_text(good[0])
