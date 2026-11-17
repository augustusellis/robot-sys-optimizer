"""Write ``examples/batch_parts/`` (v13 #9): three part packages for a batch demo with the
``rail_long_part`` example cell (UR20 on a rail along x at y = 0.15 m, orbital sander).

    python scripts/make_batch_example.py

Each golden part is placed beside the rail (near edge 0.6 m from the track, top 0.7 m above the
floor) with a 150 mm grid of targets on its upward faces (group "top", tilt 10 deg, as the rail
example). In the app: open Examples -> rail_long_part, then Optimize -> Several parts (batch) with
this folder (or scripts/batch_run.py examples/rail_long_part examples/batch_parts <out>).
"""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import trimesh  # noqa: E402

from core import part_package as pp  # noqa: E402
from core import schema  # noqa: E402
from core.targets import append_points, projected_grid, select_by_normal  # noqa: E402

OUT = ROOT / "examples" / "batch_parts"
PARTS = {  # name: (mesh, description, max angle of the faces from up, deg)
    "flat_plate": ("assets/parts/flat_plate.stl", "1 x 0.6 m plate", 5.0),
    "curved_panel": ("assets/parts/curved_panel.stl", "panel curved about x (R 1 m)", 35.0),
    "open_box": ("assets/parts/open_box.stl", "open box, the rim and the floor of the pocket", 5.0),
}
NEAR_Y, TOP_Z, SPACING = 0.75, 0.7, 0.15


def main() -> None:
    for name, (rel, desc, angle) in PARTS.items():
        mesh = trimesh.load(ROOT / rel, force="mesh")
        lo, hi = mesh.bounds
        x = -(lo[0] + hi[0]) / 2
        pose = schema.Pose(xyz=(float(x), float(NEAR_Y - lo[1]), float(TOP_Z - hi[2])))
        faces = select_by_normal(mesh, (0, 0, 1), angle)
        P, N = projected_grid(mesh, faces, SPACING)
        targets = append_points(
            schema.Targets(groups=[schema.TargetGroup(name="top", tilt_deg=10.0)]), P, N, "top"
        )
        part = schema.Part(mesh=rel, units="m", pose=pose)
        d = pp.save_part_package(
            name, part, targets, lambda r: ROOT / r, OUT, overwrite=True, description=desc
        )
        print(f"{d.relative_to(ROOT)}: {len(targets.points)} targets")


if __name__ == "__main__":
    main()
