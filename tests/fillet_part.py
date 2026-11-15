"""Test geometry for v10 T12: an inside corner (floor + wall) with a concave fillet of radius R,
as an open surface extruded along y. Floor normal +z, wall normal +x, fillet normals between."""

import numpy as np
import trimesh


def profile(R: float, L: float = 0.4, arc: int = 24) -> tuple[np.ndarray, np.ndarray]:
    """(xz points, xz normals) along the profile: floor (L..R, 0), arc, wall (0, R..L)."""
    floor = np.c_[np.linspace(L, R, 20, endpoint=False), np.zeros(20)]
    th = np.linspace(-np.pi / 2, -np.pi, arc, endpoint=False)
    fil = np.c_[R + R * np.cos(th), R + R * np.sin(th)]
    wall = np.c_[np.zeros(20), np.linspace(R, L, 20)]
    pts = np.vstack([floor, fil, wall])
    nf = np.tile([0.0, 1.0], (20, 1))
    na = -np.c_[np.cos(th), np.sin(th)]  # towards the arc centre: out of the material
    nw = np.tile([1.0, 0.0], (20, 1))
    return pts, np.vstack([nf, na, nw])


def fillet_corner(R: float = 0.01, L: float = 0.4, W: float = 0.3, ny: int = 16) -> trimesh.Trimesh:
    xz, _ = profile(R, L)
    ys = np.linspace(0, W, ny)
    V = np.array([(x, y, z) for y in ys for x, z in xz])
    m = len(xz)
    F = []
    for j in range(ny - 1):
        for i in range(m - 1):
            a, b, c, d = j * m + i, j * m + i + 1, (j + 1) * m + i, (j + 1) * m + i + 1
            F += [(a, c, b), (b, c, d)]
    mesh = trimesh.Trimesh(V, np.array(F), process=False)
    if mesh.face_normals[0] @ [0, 0, 1] < 0:
        mesh.invert()
    return mesh


def fillet_point(R: float, y: float, t: float = 0.5) -> tuple[np.ndarray, np.ndarray]:
    """A point on the fillet (t = 0 floor side .. 1 wall side) and its outward normal."""
    th = -np.pi / 2 - t * np.pi / 2
    p = np.array([R + R * np.cos(th), y, R + R * np.sin(th)])
    n = -np.array([np.cos(th), 0.0, np.sin(th)])
    return p, n
