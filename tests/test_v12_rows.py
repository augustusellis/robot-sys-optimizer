"""v12 items 1-2: pattern rows by visit, travel between coincident points (core.path_direction).

Done when (1) a move between two targets at one position takes its neighbours' travel direction
(no silent spin 0), and a sequence with no direction at all flags its samples; (2) cross-hatch
rows are kept per visit, so the second pass no longer overwrites the first pass's row of a point.
"""

import numpy as np
import pytest

from core import golden_parts as gp
from core import path_direction as pd
from core.collision import CollisionScene
from core.feasibility import FeasibilityChecker
from core.ik import make_ik
from core.paths import trace
from core.patterns import PatternSettings
from core.robot import Robot
from core.schema import Solution, Station, TargetGroup
from core.tool import ToolModel, load_tool
from core.transforms import make_T
from tests.test_v11_direction import REPO

T_PART = make_T(p=(0.9, 0.0, 0.3))
GROUP = TargetGroup(name="top", tilt_deg=5.0)
SQUARE = {
    k: (np.array(p, float), np.array([0.0, 0, 1]))
    for k, p in enumerate([(0, 0, 0), (1, 0, 0), (0, 1, 0), (1, 1, 0)])
}


def test_cross_hatch_rows_are_kept_per_visit():
    # pass 1 rows along x (0: 0->1, 1: 3->2), pass 2 rows along y (2: 2->0, 3: 1->3)
    pids = [0, 1, 3, 2, 2, 0, 1, 3]
    rows = pd.Rows.of(pids, [0, 0, 1, 1, 2, 2, 3, 3])
    # the move 0 -> 1 is a row move in pass 1 and a row change in pass 2
    assert rows.visits(0, 1) == [((0, 0), (0, 0)), ((0, 2), (0, 3))]
    assert not rows.change_at(0) and rows.change_at(5)
    assert not rows.change(0, 1)  # not every visit is a row change
    # pass 2's row change leaves row 2 (2 -> 0, along -y): the tool steps sideways along it
    assert np.allclose(pd.travel(pids, SQUARE, rows, 5), [0, -1, 0])
    assert np.allclose(pd.travel(pids, SQUARE, rows, 0), [1, 0, 0])
    # a dict by point id (v11) lost pass 1: point 0 named row 2 only
    old = dict(zip(pids, [0, 0, 1, 1, 2, 2, 3, 3], strict=True))
    assert old[0] == 2 and old[1] == 3


def test_merged_rows_of_two_sequences_do_not_collide():
    a = pd.Rows.of([0, 1], [0, 0], tag=0)
    b = pd.Rows.of([2, 3], [0, 0], tag=1)
    a.merge(b)
    assert set(a.members) == {(0, 0), (1, 0)} and a.n == 2
    assert a.members[(1, 0)] == [2, 3]


def test_coincident_points_take_the_neighbours_direction():
    pts = {0: (np.zeros(3), np.array([0.0, 0, 1])), 1: (np.array([0.1, 0, 0]), np.array([0.0, 0, 1]))}
    pts[2] = (pts[1][0].copy(), pts[1][1])  # a duplicate of point 1
    pts[3] = (np.array([0.1, 0.1, 0]), np.array([0.0, 0, 1]))
    pids = [0, 1, 2, 3]
    assert np.allclose(pd.travel(pids, pts, pd.Rows(), 1), [0.1, 0, 0])  # from the move before
    assert np.allclose(pd.travel([1, 2, 3], pts, pd.Rows(), 0), [0, 0.1, 0])  # first: the next move
    assert pd.travel([1, 2], pts, pd.Rows(), 0) is None  # nothing to go by
    T = np.eye(4)
    assert pd.spin_for(T, np.array([0.0, 0, 1]), "along_x") is None  # along the tool axis
    assert pd.offsets_for(T, None, "along_x") is None


@pytest.fixture(scope="module")
def cell():
    robot = Robot.from_registry("ur20")
    ee = load_tool("orbital_sander")
    tm = ToolModel(ee, REPO / ee.urdf)
    mesh = gp.flat_plate()
    scene = CollisionScene(robot, tm, mesh, np.eye(4), T_PART)
    checker = FeasibilityChecker(robot, make_ik(robot), tm, scene, np.eye(4), T_PART)
    return checker, mesh, ee.tcps[0].name


def _plan(cell, pts, pattern):
    checker, mesh, tcp = cell
    res = {p: checker.check_point(p, P, N, GROUP) for p, (P, N) in pts.items()}
    assert all(r.status != "unreachable" for r in res.values())
    sol = Solution(stations=[Station(xyz=(0, 0, 0))], assignment={p: 0 for p in pts})
    return trace(
        checker, mesh, sol, res, pts, [np.eye(4)], step=0.02, pattern=pattern,
        direction=lambda t: pd.Direction("along_x") if t == tcp else None,
    )  # fmt: skip


def _grid(n=4, dup=None):
    pts = {}
    for i, (y, x) in enumerate((y, x) for y in np.linspace(-0.1, 0.1, n) for x in np.linspace(-0.2, 0.2, n)):
        pts[i] = (np.array([x, y, 0.02]), np.array([0.0, 0.0, 1.0]))
    if dup is not None:
        pts[100] = (pts[dup][0].copy(), pts[dup][1])
    return pts


def test_duplicate_target_keeps_the_row_direction(cell):
    # rows along y: the planned spin (0 = part x) is 90 deg off, so spin 0 would show
    pts = _grid(dup=5)
    plan = _plan(cell, pts, PatternSettings("Zigzag", angle_deg=90.0))
    assert not plan.counts()["direction"]
    on_dup = [p for p in plan.samples if not p.kind and {p.a, p.b} == {5, 100}]
    assert on_dup  # the zero-length move between the two copies
    for p in on_dup:  # along the row (y), not the planned spin
        assert pd.deviation_deg(p.T_part_tcp, np.array([0.0, 1, 0]), "along_x") <= pd.TOL_DEG


def test_cross_hatch_directional_tool_flags_nothing(cell):
    plan = _plan(cell, _grid(), PatternSettings("Cross-hatch"))
    surf = [p for p in plan.samples if not p.kind and p.a != p.b]
    assert surf and not plan.counts()["direction"]
