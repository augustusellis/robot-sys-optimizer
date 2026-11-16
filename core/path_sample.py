"""Samples of a surface move between two planned points (v12 #14).

One function for the three path builders that sample a move a -> b on the part: the tracer
(core.paths._Tracer.segment), the spin optimizer (core.path_spin.segment_poses) and coordinated
motion (core.coordinated._pass). Before v12 each had its own copy of these lines, so a fix to one
(for instance v11's overrun samples) had to be repeated by hand in the others.

Samples ``step`` apart along the chord, each projected onto the mesh (the last one exactly on b),
offset by the interpolated standoff along the interpolated normal, the orientation slerped between
the two planned poses; ``lift`` raises the middle of the move off the surface (full height from
15 % to 85 % of the way). Poses are in the part frame. Pure geometry: no viser.
"""

from __future__ import annotations

import numpy as np
import trimesh
from scipy.spatial.transform import Rotation, Slerp


def chord_poses(
    mesh: trimesh.Trimesh,
    Ta: np.ndarray,
    Tb: np.ndarray,
    Pa,
    Pb,
    Na,
    Nb,
    so_a: float,
    so_b: float,
    step: float,
    lift: float = 0.0,
) -> tuple[np.ndarray, list[np.ndarray]]:
    """(s in (0, 1], TCP poses) of the samples a -> b (a itself excluded, b included)."""
    Pa, Pb, Na, Nb = (np.asarray(v, float) for v in (Pa, Pb, Na, Nb))
    n = max(1, int(np.ceil(np.linalg.norm(Pb - Pa) / step)))
    s = np.arange(1, n + 1) / n
    chord = Pa + s[:, None] * (Pb - Pa)
    on, _, _ = trimesh.proximity.closest_point(mesh, chord)
    on[-1] = Pb  # land exactly on the planned point
    normal = Na + s[:, None] * (Nb - Na)
    normal /= np.linalg.norm(normal, axis=1, keepdims=True)
    so = so_a + s * (so_b - so_a)
    if lift > 0:  # off the surface: up, across, down
        so = so + lift * np.minimum(1.0, 8 * s * (1 - s))
    rot = Slerp([0.0, 1.0], Rotation.from_matrix([Ta[:3, :3], Tb[:3, :3]]))(s).as_matrix()
    out = []
    for i in range(n):
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = rot[i], on[i] + so[i] * normal[i]
        out.append(T)
    return s, out
