"""v11 task 7: directional travel along the paths (core.path_direction).

Done when (a) a scanner TCP (along x, 60 mm wide, 10 % overlap) scans rows 54 mm apart with its x
axis within 2 deg of the travel direction, rows run 20 mm past their ends and the row changes
happen there, off the part; (b) a drum TCP (x across the travel) keeps x within 2 deg of across;
(c) a free TCP (a sander) gets the same paths as before.
"""

from pathlib import Path

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
from core.schema import TCP, Solution, Station, TargetGroup
from core.tool import ToolModel, load_tool
from core.transforms import make_T

REPO = Path(__file__).resolve().parents[1]
T_PART = make_T(p=(0.9, 0.0, 0.3))
GROUP = TargetGroup(name="top", tilt_deg=5.0)


def test_row_pitch_and_spin_geometry():
    assert pd.row_pitch(60.0, 10.0) == pytest.approx(54.0)
    T = np.eye(4)
    T[:3, :3] = np.diag([1.0, -1.0, -1.0])  # tool z down onto a floor-like surface
    d = np.array([0.0, 1.0, 0.0])  # travel along +y
    for mode, axis in (("along_x", 0), ("along_y", 1), ("perpendicular_x", 1)):
        o = pd.spin_for(T, d, mode)
        c, s = np.cos(o), np.sin(o)
        Tr = T @ np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]])
        assert abs(abs(Tr[:3, axis] @ d) - 1.0) < 1e-9, mode
        assert pd.deviation_deg(Tr, d, mode) < 1e-6
    assert pd.deviation_deg(T, d, "along_x") == pytest.approx(90.0)
    assert pd.deviation_deg(T, d, "free") == 0.0
    off = pd.offsets_for(T, d, "along_x")
    assert len(off) == 2 and abs(abs(off[0] - off[1]) - np.pi) < 1e-9  # either way along the line


@pytest.fixture(scope="module")
def cell():
    robot = Robot.from_registry("ur20")
    ee = load_tool("orbital_sander")
    tm = ToolModel(ee, REPO / ee.urdf)
    mesh = gp.flat_plate()
    scene = CollisionScene(robot, tm, mesh, np.eye(4), T_PART)
    checker = FeasibilityChecker(robot, make_ik(robot), tm, scene, np.eye(4), T_PART)
    xs, ys = np.arange(-0.25, 0.2501, 0.05), np.arange(-0.108, 0.1081, 0.054)  # rows 54 mm apart
    pts = {}
    for i, (y, x) in enumerate((y, x) for y in ys for x in xs):
        pts[i] = (np.array([x, y, 0.02]), np.array([0.0, 0.0, 1.0]))
    res = {p: checker.check_point(p, P, N, GROUP) for p, (P, N) in pts.items()}
    assert all(r.status != "unreachable" for r in res.values())
    sol = Solution(stations=[Station(xyz=(0, 0, 0))], assignment={p: 0 for p in pts})
    return checker, mesh, sol, res, pts, ee.tcps[0].name


def _trace(cell, direction=None):
    checker, mesh, sol, res, pts, _ = cell
    return trace(
        checker,
        mesh,
        sol,
        res,
        pts,
        [np.eye(4)],
        step=0.02,
        pattern=PatternSettings("Zigzag"),
        direction=direction,
    )


def _rows(pts):
    ys = sorted({round(float(P[1]), 4) for P, _ in pts.values()})
    return {p: ys.index(round(float(P[1]), 4)) for p, (P, _) in pts.items()}


@pytest.mark.parametrize("mode", ["along_x", "perpendicular_x"])
def test_directional_tool_follows_the_rows_and_overruns(cell, mode):
    _, _, _, _, pts, tcp = cell
    plan = _trace(cell, lambda t: pd.Direction(mode, 0.02) if t == tcp else None)
    surf = [p for p in plan.samples if not p.kind and p.a != p.b]
    assert surf and not plan.counts()["direction"]
    rows = _rows(pts)
    for p in surf:
        assert rows[p.a] == rows[p.b]  # no sideways move on the part: row changes are off it
        d = pts[p.b][0] - pts[p.a][0]
        assert pd.deviation_deg(p.T_part_tcp, d, mode) <= pd.TOL_DEG
    over = [p for p in plan.samples if p.kind == "overrun"]
    assert over
    x = np.array([p.T_part_tcp[0, 3] for p in over])
    assert x.max() == pytest.approx(0.25 + 0.02, abs=1e-6) and x.min() == pytest.approx(
        -0.25 - 0.02, abs=1e-6
    )
    # every row change happened in overrun samples (no surface sample joins two rows)
    assert all(not pd.row_change(rows, p.a, p.b) for p in surf)
    assert sum(1 for p in plan.samples if p.issue in ("collision", "unreachable")) == 0


def test_free_tool_paths_unchanged(cell):
    _, _, _, _, _, tcp = cell
    a = _trace(cell)
    b = _trace(cell, lambda t: pd.Direction("free"))
    assert len(a.samples) == len(b.samples)
    assert all(np.allclose(x.q, y.q) for x, y in zip(a.samples, b.samples, strict=True) if x.q is not None)
    assert not b.counts()["direction"]


def test_tcp_schema_fields():
    t = TCP(name="scan", travel="along_x", scan_width_mm=60.0, overlap_pct=10.0, overrun_mm=20.0)
    assert t.travel == "along_x" and TCP(name="x").travel == "free"
    with pytest.raises(ValueError):
        TCP(name="bad", travel="diagonal")


def test_package_frame_carries_the_travel(tmp_path):
    from core.urdf_package import Frame, Package, Part, to_tool, write_package, zip_folder
    from core.urdf_package_read import open_package

    p = Package(
        kind="tool",
        name="scanner_pkg",
        parts=[Part(name="head", stage="tool", shape="box", size=(0.1, 0.06, 0.05))],
        tcps=[Frame(name="scan", travel="along_y", scan_width_mm=80.0, overlap_pct=20.0, overrun_mm=30.0)],
    )
    t = to_tool(p, "t.urdf").tcps[0]
    assert (t.travel, t.scan_width_mm, t.overlap_pct, t.overrun_mm) == ("along_y", 80.0, 20.0, 30.0)
    write_package(p, None, tmp_path / "t")
    back, _ = open_package(zip_folder(tmp_path / "t"), tmp_path / "back")
    f = back.tcps[0]
    assert (f.travel, f.scan_width_mm, f.overlap_pct, f.overrun_mm) == ("along_y", 80.0, 20.0, 30.0)


@pytest.mark.slow
def test_tool_tab_fields_rows_and_paths_in_the_app():
    import viser

    from app import build_app
    from tests.helpers import confirm_all

    server = viser.ViserServer(host="127.0.0.1", port=8973, verbose=False)
    try:
        app = build_app(server)
        s, tool, sim, targets = app["session"], app["tool"], app["sim"], app["targets"]
        assert app["project"].load_example("rail_long_part")
        tool.refresh()
        f = tool.travel
        f.travel.value = "along TCP x (scanner)"
        f.width.value = 60.0
        f.overlap.value = 10.0
        f.overrun.value = 20.0
        t = s.tool.tcps[0]
        assert (t.travel, t.scan_width_mm, t.overlap_pct, t.overrun_mm) == ("along_x", 60.0, 10.0, 20.0)
        assert "54.0 mm apart" in f.pitch.content
        assert f.use_pitch() == pytest.approx(54.0)
        assert targets.row.value == pytest.approx(54.0) and targets.stepover.value == pytest.approx(54.0)
        confirm_all(s)
        app["workflow"].refresh(force=True)
        app["stations"].start(block=True, quick=True)
        app["results"].refresh()
        sim.pattern.value = "Zigzag"
        sim.build(block=True)
        plan = sim.plan
        assert any(p.kind == "overrun" for p in plan.samples)  # rows run past the part's ends
        pts = {p.id: (np.array(p.p), np.array(p.n)) for p in s.targets.points}
        surf = [p for p in plan.samples if not p.kind and p.a != p.b and not p.issue]
        ok = [
            pd.deviation_deg(p.T_part_tcp, pts[p.b][0] - pts[p.a][0], "along_x") <= pd.TOL_DEG for p in surf
        ]
        flagged = plan.counts()["direction"]
        assert surf and (all(ok) or flagged)  # followed, or the samples that cannot are flagged
    finally:
        app["builder_window"].stop()
        server.stop()
