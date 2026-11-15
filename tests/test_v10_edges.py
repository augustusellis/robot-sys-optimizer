"""v10 T10: dense reach check at zone edges (core.edge_check).

Done when a deliberately marginal case shows edge gaps the sparse points miss: a thin stay-out
column stands on the plate exactly between two stops' areas. Every sparse target is reached (the
column sits halfway between neighbouring targets), but the verification point between the stops
lands on it.
"""

from pathlib import Path

import numpy as np
import pytest

from core import edge_check as ec
from core import golden_parts as gp
from core.collision import CollisionScene
from core.feasibility import FeasibilityChecker
from core.ik import make_ik
from core.robot import Robot
from core.schema import Pose, Primitive, Solution, Station, TargetGroup, Zone
from core.stayout import obstacles, scene_entries
from core.tool import ToolModel, load_tool
from core.transforms import make_T

REPO = Path(__file__).resolve().parents[1]
T_PART = make_T(p=(0.9, 0.0, 0.3))
GROUP = TargetGroup(name="top", tilt_deg=10.0)
XS, YS = (-0.375, -0.125, 0.125, 0.375), (-0.25, 0.0, 0.25)


@pytest.fixture(scope="module")
def robot():
    return Robot.from_registry("ur20")


def _checker(robot, column: bool) -> FeasibilityChecker:
    ee = load_tool("orbital_sander")
    tm = ToolModel(ee, REPO / ee.urdf)
    obs = []
    if column:  # 20 mm square, 0.4 m tall, on the plate top at part (0, 0)
        c = T_PART[:3, 3] + np.array([0.0, 0.0, 0.02])
        obs = obstacles(
            [
                Primitive(
                    shape="box", name="col", attach="fixed", size=(0.02, 0.02, 0.4), pose=Pose(xyz=tuple(c))
                )
            ]
        )
    scene = CollisionScene(robot, tm, gp.flat_plate(), np.eye(4), T_PART, obstacles=scene_entries(obs))
    return FeasibilityChecker(robot, make_ik(robot), tm, scene, np.eye(4), T_PART)


def _grid():
    P = np.array([(x, y, 0.02) for x in XS for y in YS])
    N = np.tile([0.0, 0.0, 1.0], (len(P), 1))
    return np.arange(len(P)), P, N


def _plan(ids, P, zones=False):
    """Two stops at the same base: x < 0 -> stop 0, else stop 1 (zoned: one stop, two zones)."""
    left = [int(i) for i in ids if P[i, 0] < 0]
    right = [int(i) for i in ids if P[i, 0] > 0]
    st = Station(xyz=(0.0, 0.0, 0.0), yaw=0.0)
    if zones:
        return Solution(
            stations=[st],
            assignment={i: 0 for i in left + right},
            zones=[Zone(station=0, config="A", points=left), Zone(station=0, config="B", points=right)],
        )
    return Solution(stations=[st, st], assignment={**{i: 0 for i in left}, **{i: 1 for i in right}})


def _run(robot, column: bool):
    ids, P, N = _grid()
    chk = _checker(robot, column)
    res = {int(i): chk.check_point(int(i), P[i], N[i], GROUP) for i in ids}
    sol = _plan(ids, P)
    stop_of, zone_of, low = ec.plan_maps(sol, res)
    pts = ec.edge_points(gp.flat_plate(), ids, P, N, stop_of, zone_of, set())
    gaps = ec.check_edges(chk, pts, [np.eye(4), np.eye(4)], lambda _pid: GROUP)
    return res, pts, gaps


def test_edge_points_kinds_and_spacing():
    ids, P, N = _grid()
    sol = _plan(ids, P)
    stop_of = {int(p): k for p, k in sol.assignment.items()}
    pts = ec.edge_points(gp.flat_plate(), ids, P, N, stop_of, {}, {0})
    kinds = {e.kind for e in pts}
    assert kinds == {"stops", "edge", "margin"}
    between = [e for e in pts if e.kind == "stops"]
    # straight neighbours: one midpoint each; diagonal neighbours (1.41 x spacing): three samples
    assert all(abs(e.p[0]) <= 0.0625 + 1e-9 and e.stops == [0, 1] for e in between)
    mid = sorted(round(e.p[1], 4) for e in between if abs(e.p[0]) < 1e-9)
    assert mid == [-0.25, -0.125, 0.0, 0.125, 0.25]
    # edge points lie on the plate's outline within reach of the targets, normals up
    edge = [e for e in pts if e.kind == "edge"]
    assert edge and all(max(abs(e.p[0]) - 0.5, abs(e.p[1]) - 0.3) > -1e-6 for e in edge)
    assert all(e.n[2] > 0.99 for e in edge)  # only the targeted (top) faces
    # a ring of 6 around the low-margin corner point; those that fall on edge or boundary points
    # (or off the plate, projected onto its edge) are thinned away
    ring = [e for e in pts if e.kind == "margin"]
    assert 1 <= len(ring) <= 6 and all(e.near == 0 and np.linalg.norm(e.p - P[0]) < 0.13 for e in ring)
    assert ec.spacing_of(P) == pytest.approx(0.25)


def test_zone_boundary_points():
    ids, P, N = _grid()
    sol = _plan(ids, P, zones=True)
    stop_of = {int(p): k for p, k in sol.assignment.items()}
    zone_of = {int(p): (zi, z.config) for zi, z in enumerate(sol.zones) for p in z.points}
    z = [e for e in ec.edge_points(gp.flat_plate(), ids, P, N, stop_of, zone_of) if e.kind == "zones"]
    assert len(z) == 13 and all(e.configs == ("A", "B") and e.stops == [0] for e in z)


@pytest.mark.slow
def test_marginal_case_shows_gap_the_sparse_points_miss(robot):
    res, pts, gaps = _run(robot, column=True)
    assert all(r.status != "unreachable" for r in res.values())  # the sparse plan looks complete
    between = [e for e in gaps if e.kind == "stops"]
    # the gaps are the verification points around the column, the one on it among them
    assert between and all(np.linalg.norm(e.p[:2]) < 0.1 for e in between)
    assert any(np.allclose(e.p[:2], 0.0, atol=1e-6) for e in between)
    assert all(e.reason == "collision" and "stay-out zone col" in e.notes[0] for e in between)
    assert all(e.kind == "stops" for e in gaps)
    assert f"{len(gaps)} between stops" in ec.summary(pts, gaps)
    # without the column the same verification points are all reached
    _, pts0, gaps0 = _run(robot, column=False)
    assert len(pts0) == len(pts) and not gaps0
    assert ec.summary(pts0, gaps0) == f"all {len(pts0)} edge points reached"


def test_plan_maps_skip_unreached_and_flag_low_margin():
    from core.feasibility import PointResult

    sol = Solution(
        stations=[Station(xyz=(0, 0, 0), yaw=0.0)],
        assignment={1: 0, 2: 0, 3: 0},
        zones=[Zone(station=0, config="A", points=[1, 2])],
    )
    res = {
        1: PointResult(1, "reachable", limit_margin_deg=40.0),
        2: PointResult(2, "reachable", limit_margin_deg=5.0),
        3: PointResult(3, "unreachable", "no_ik"),
    }
    stop_of, zone_of, low = ec.plan_maps(sol, res)
    assert stop_of == {1: 0, 2: 0} and zone_of == {1: (0, "A"), 2: (0, "A")} and low == {2}


@pytest.fixture(scope="module")
def app():
    import viser

    from app import build_app

    server = viser.ViserServer(host="127.0.0.1", port=8931, verbose=False)
    st = build_app(server)
    yield st
    server.stop()


@pytest.mark.slow
def test_edge_check_in_the_app(app):
    from tests.helpers import confirm_all
    from ui.part_heatmap import EDGE

    s, edge, targets = app["session"], app["edge"], app["targets"]
    assert app["project"].load_example("rail_long_part")
    confirm_all(app["stations"])
    edge.auto.value = True
    app["stations"].start(block=True)  # on_done starts the edge check on the best plan
    assert edge._thread is not None
    edge._thread.join()
    r = s.edge_check
    assert r is not None and r.points and edge.current() is r
    assert r.summary() in edge.status.content
    n_targets = len(s.targets.points)
    assert len(r.points) > 0.2 * n_targets  # dense where it matters, not everywhere
    # Colour part by -> Edge check paints faces; the legend names the result
    targets.heat.choice.value = EDGE
    targets.heat.draw()
    assert r.summary() in targets.heat.legend.content and (targets.heat.face_bins >= 0).any()
    # gaps: markers + Densify edges (forced: mark two checked points as gaps)
    for e in r.points[:2]:
        e.reached = False
    r.gaps = [e for e in r.points if not e.reached]
    targets.refresh()
    assert "edge_gaps" in targets._handles
    added = edge.add_gaps()
    assert added == len(r.gaps) and len(s.targets.points) == n_targets + added
    assert s.edge_check is None and edge.densify.disabled
    targets.tools.undo_last()
    assert len(s.targets.points) == n_targets
