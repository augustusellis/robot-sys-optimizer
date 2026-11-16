"""v12 item 14: one chord sampler for the three path builders (core.path_sample).

Done when ``chord_poses`` gives the v11 samples (pinned against a copy of the old code on the curved
panel, with and without a lift) and the tracer, the spin optimizer and coordinated motion all use it.
"""

import inspect

import numpy as np
import trimesh
from scipy.spatial.transform import Rotation, Slerp

from core import coordinated, path_spin, paths
from core import golden_parts as gp
from core.path_sample import chord_poses


def _v11(mesh, Ta, Tb, Pa, Pb, Na, Nb, so_a, so_b, step, lift=0.0):
    """core.path_spin.segment_poses as it was in v11."""
    n = max(1, int(np.ceil(np.linalg.norm(Pb - Pa) / step)))
    s = np.arange(1, n + 1) / n
    chord = Pa + s[:, None] * (Pb - Pa)
    on, _, _ = trimesh.proximity.closest_point(mesh, chord)
    on[-1] = Pb
    normal = Na + s[:, None] * (Nb - Na)
    normal /= np.linalg.norm(normal, axis=1, keepdims=True)
    so = so_a + s * (so_b - so_a)
    if lift > 0:
        so = so + lift * np.minimum(1.0, 8 * s * (1 - s))
    rot = Slerp([0.0, 1.0], Rotation.from_matrix([Ta[:3, :3], Tb[:3, :3]]))(s).as_matrix()
    out = []
    for i in range(n):
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = rot[i], on[i] + so[i] * normal[i]
        out.append(T)
    return s, out


def test_same_samples_as_v11():
    mesh = gp.curved_panel()
    a, b = np.radians(-10.0), np.radians(12.0)
    Pa = np.array([0.1, np.sin(a), np.cos(a)])
    Pb = np.array([-0.2, np.sin(b), np.cos(b)])
    Na, Nb = Pa * [0, 1, 1], Pb * [0, 1, 1]
    Ta, Tb = np.eye(4), np.eye(4)
    Ta[:3, :3] = Rotation.from_euler("xyz", [180, 5, 0], degrees=True).as_matrix()
    Tb[:3, :3] = Rotation.from_euler("xyz", [170, -5, 40], degrees=True).as_matrix()
    for lift in (0.0, 0.05):
        s1, T1 = chord_poses(mesh, Ta, Tb, Pa, Pb, Na, Nb, 0.0, 0.002, 0.02, lift)
        s0, T0 = _v11(mesh, Ta, Tb, Pa, Pb, Na, Nb, 0.0, 0.002, 0.02, lift)
        assert np.allclose(s1, s0) and len(T1) == len(T0) > 5
        assert all(np.allclose(x, y, atol=1e-12) for x, y in zip(T1, T0, strict=True))
        assert np.allclose(T1[-1][:3, 3], Pb + 0.002 * Nb / np.linalg.norm(Nb))  # lands on b


def test_the_three_builders_use_it():
    for fn in (paths._Tracer.segment, path_spin.segment_poses, coordinated._pass):
        src = inspect.getsource(fn)
        assert "chord_poses" in src and "closest_point" not in src, fn.__qualname__
