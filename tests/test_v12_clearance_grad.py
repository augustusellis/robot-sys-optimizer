"""v12 item 21: analytic gradient of the part clearance (core.collision.part_clearance_grad).

Done when the gradient equals central finite differences near the part (robot links and the tool),
the clearance is the same as part_clearance, and joint-move smoothing uses it.
Measured (docs/V12_PLAN.md): one distance pass (~5 ms) instead of seven; CRX-30 gantry_overhead
path build 148.9 -> 105.2 s, same cycle time.
"""

import inspect
from pathlib import Path

import numpy as np
import trimesh

from core import traj_opt
from core.collision import CollisionScene
from core.robot import Robot
from core.tool import ToolModel, load_tool

REPO = Path(__file__).resolve().parents[1]


def test_gradient_matches_finite_differences():
    r = Robot.from_registry("ur20")
    ee = load_tool("orbital_sander")
    tm = ToolModel(ee, REPO / ee.urdf)
    part = trimesh.creation.box((0.6, 0.6, 0.4))
    T = np.eye(4)
    T[:3, 3] = (1.0, 0.0, 0.2)
    sc = CollisionScene(r, tm, part, np.eye(4), T)
    rng = np.random.default_rng(1)
    n, eps = 0, 1e-5
    for _ in range(300):
        q = np.radians([0, -90, 90, -90, -90, 0]) + rng.normal(0, 0.6, 6)
        d, g = sc.part_clearance_grad(q)
        if not 0.005 < d < 0.15:
            continue
        assert d == sc.part_clearance(q)
        fd = [
            (sc.part_clearance(q + eps * e) - sc.part_clearance(q - eps * e)) / (2 * eps) for e in np.eye(6)
        ]
        assert np.linalg.norm(g - fd) <= 1e-6 * max(np.linalg.norm(fd), 1e-3), (q, g, fd)
        n += 1
        if n >= 12:
            break
    assert n >= 8


def test_no_part_no_gradient():
    r = Robot.from_registry("ur20")
    d, g = CollisionScene(r, None, None, np.eye(4), None).part_clearance_grad(np.zeros(6))
    assert d == float("inf") and not g.any()


def test_smoothing_uses_it():
    assert "part_clearance_grad" in inspect.getsource(traj_opt.smooth_move)
    assert "clearance_grad" in inspect.signature(traj_opt.smooth_path).parameters
