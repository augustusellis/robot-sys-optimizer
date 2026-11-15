"""Surfaces kept exact on part packages with several meshes (v10 T19).

The faces selected in Targets index the project's part mesh. A part package made of several mesh
parts (in the part's own frame) needs them per mesh part: a selected face is found on a mesh part
when one of that part's faces has the same centre (within ``TOL`` of the part's size), so the
selection carries over when the project's part is made of the package's meshes, in any order.
"""

from __future__ import annotations

import numpy as np
from scipy.spatial import cKDTree

from core.transforms import pose_to_matrix

TOL = 1e-6  # x the part's bounding-box diagonal


def split_keep(part_mesh, keep: list[int], meshes: dict, poses: dict) -> dict[str, list[int]]:
    """{mesh part name: its face ids matching the selected faces ``keep`` of ``part_mesh``}."""
    if not keep or not meshes:
        return {}
    keep = [k for k in keep if 0 <= k < len(part_mesh.faces)]
    C = part_mesh.triangles_center[keep]
    tol = TOL * float(np.linalg.norm(part_mesh.extents)) + 1e-9
    out = {}
    for name, m in meshes.items():
        T = pose_to_matrix(poses[name])
        cen = m.triangles_center @ T[:3, :3].T + T[:3, 3]
        d, _ = cKDTree(C).query(cen, distance_upper_bound=max(tol, 1e-7))
        ids = np.flatnonzero(np.isfinite(d)).tolist()
        if ids:
            out[name] = ids
    return out
