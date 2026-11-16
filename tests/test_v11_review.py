"""v11 task 12: the unit tests the review (docs/V11_REVIEW.md) found missing, and the fixes of the
edge cases it listed that were cheap to fix (each test says which)."""

import heapq
from types import SimpleNamespace

import numpy as np
import pytest

from core.cycle_time import CycleSettings, _kind, joint_move_time, path_time, runs, trapezoid
from core.path_types import PathPlan, PathSample, StationPath
from core.trajectory import time_path

# --- robot programs ------------------------------------------------------------------------------


def _s(x, q, tcp="tcp", kind="", a=1, b=2, issue=""):
    T = np.eye(4)
    T[0, 3] = x
    return PathSample(
        0, None, a, b, 0.5, T, tcp, None if q is None else np.asarray(q, float), issue, kind=kind
    )


def _plan(samples):
    return PathPlan([StationPath(0, np.eye(4), {}, samples)], 0.01)


def test_program_refuses_several_tcps():
    from core.robot_program import program

    plan = _plan([_s(0.0, np.zeros(6), "pad"), _s(0.01, np.zeros(6), "edge")])
    with pytest.raises(ValueError, match="2 TCPs"):
        program("urscript", plan, np.eye(4), np.eye(4))


@pytest.mark.parametrize("fmt", ["urscript", "fanuc", "rapid"])
def test_program_says_which_samples_it_skips(fmt):
    from core.robot_program import program

    plan = _plan([_s(0.0, np.zeros(6)), _s(0.01, None, issue="unreachable", a=4, b=5), _s(0.02, np.zeros(6))])
    text = program(fmt, plan, np.eye(4), np.eye(4))
    assert "skipped (point 4 -> 5): no joint solution, unreachable" in text
    moves = {"urscript": "movel(", "fanuc": " L P[", "rapid": "MoveL p"}[fmt]
    assert text.count(moves) == 2


def test_config_strings_of_known_poses():
    from core.robot_program import _abb_conf, _fanuc_config

    assert _fanuc_config(np.zeros(6)) == "'N U T, 0, 0, 0'"
    assert _fanuc_config(np.radians([0, 0, -120, 0, -30, 0])).startswith("'F D T")
    assert _fanuc_config(np.radians([170, 0, 0, 200, 10, -200])) == "'N U B, 0, 1, -1'"
    assert _abb_conf(np.zeros(6)) == "[0,0,0,0]"
    assert _abb_conf(np.radians([100, 0, -100, -10, -5, 185])) == "[1,-1,2,7]"


# --- units -------------------------------------------------------------------------------------


def test_mesh_units_boundaries():
    from core.mesh_units import M_FROM, M_UP_TO, MM_ABOVE, guess_units

    assert guess_units(30.0) == ("mm", False)  # 30 mm or 30 inches: ask (was "mm", sure)
    assert guess_units(MM_ABOVE + 1) == ("mm", True) and guess_units(MM_ABOVE) == ("mm", False)
    assert guess_units(M_UP_TO) == ("m", True) and guess_units(M_FROM) == ("m", True)
    assert guess_units(M_FROM / 2)[1] is False and guess_units(12.0) == ("m", False)


# --- timing ------------------------------------------------------------------------------------


def test_zero_velocity_limit_means_no_limit():
    cs = CycleSettings(joint_speed=1.0, joint_acc=2.0)
    dq = np.array([1.0, 0, 0, 0, 0, 0])
    lim = np.array([0.0, 2, 2, 2, 2, 2])  # a URDF without a velocity for joint 1
    assert joint_move_time(dq, cs, lim) == pytest.approx(trapezoid(1.0, 1.0, 2.0))
    Q = np.c_[np.linspace(0, 1, 51), np.zeros((51, 5))]
    plan = _plan([_s(0.0, q, kind="home") for q in Q])
    t = path_time(plan, CycleSettings(joint_speed=1.0, joint_acc=2.0, joint_jerk=0.0, settle_s=0.0), lim)
    assert t["joint"] == pytest.approx(trapezoid(1.0, 1.0, 2.0), rel=1e-3)


def test_jerk_limit_on_a_triangle_profile():
    """A short move never reaches full speed (acceleration reverses at once): the averaged profile
    must still keep the jerk within the limit (checked at tau / 2, as the filter does)."""
    Q = np.c_[np.linspace(0, 0.2, 81), np.zeros(81)]
    j = 5.0
    t = time_path(Q, 1.0, 1.4, j)
    h = 1e-3
    g = np.arange(0, t[-1], h)
    s = np.interp(g, t, Q[:, 0])
    c = int(round((1.4 / j) / 2 / h))  # tau / 2 samples
    jerk = np.diff(s[::c], n=3) / (c * h) ** 3
    assert np.abs(jerk).max() <= j * 1.1
    assert t[-1] > time_path(Q, 1.0, 1.4)[-1]  # and slower than without the limit


def test_runs_and_kinds_table():
    car = {"s": 0.0}
    a = _s(0.0, np.zeros(6))
    cases = [
        (_s(0.01, np.zeros(6)), "process"),
        (_s(0.01, np.zeros(6), kind="lift"), "approach"),
        (_s(0.01, np.zeros(6), kind="retreat"), "approach"),
        (_s(0.01, np.zeros(6), kind="home"), "joint"),
        (_s(0.01, np.zeros(6), kind="overrun"), "process"),
        (_s(0.01, None), "joint"),
        (PathSample(1, None, 1, 2, 0.5, np.eye(4), "tcp", np.zeros(6)), "carrier"),  # another stop
    ]
    for b, want in cases:
        assert _kind(a, b) == want, (b.kind, want)
    ca = PathSample(0, None, 1, 2, 0.5, np.eye(4), "tcp", np.zeros(6), carrier=car)
    cb = PathSample(0, None, 1, 2, 0.6, np.eye(4), "tcp", np.zeros(6), carrier={"s": 0.1})
    assert _kind(ca, cb) == "process"  # coordinated: the carrier moves on the surface
    seq = [
        _s(0, np.zeros(6)),
        _s(0.01, np.zeros(6)),
        _s(0.02, np.zeros(6), kind="lift"),
        _s(0.03, np.zeros(6)),
    ]
    # the move onto the lift sample and the one leaving it are both approach moves
    assert runs(seq) == [("process", 0, 1), ("approach", 1, 3)]


# --- simplifications: same results as before -------------------------------------------------------


def _smooth_old(X, path_s, width):
    out = np.empty_like(X)
    for i, s in enumerate(path_s):
        m = np.abs(path_s - s) <= width / 2
        out[i] = X[m].mean(axis=0)
    return out


def test_smooth_matches_the_quadratic_version():
    from core.coordinated import _smooth

    rng = np.random.default_rng(3)
    path_s = np.cumsum(rng.uniform(0.0, 0.04, 400))
    path_s[50:53] = path_s[50]  # repeated samples (zero-length steps)
    X = rng.normal(size=(400, 2))
    assert np.allclose(_smooth(X, path_s, 0.15), _smooth_old(X, path_s, 0.15))


def _dijkstra_old(P, nb, sources):
    d = np.full(len(P), np.inf)
    heap = [(0.0, int(s)) for s in np.flatnonzero(sources)]
    for _, s in heap:
        d[s] = 0.0
    heapq.heapify(heap)
    while heap:
        di, i = heapq.heappop(heap)
        if di > d[i]:
            continue
        for j in nb[i]:
            nd = di + float(np.linalg.norm(P[j] - P[i]))
            if nd < d[j]:
                d[j] = nd
                heapq.heappush(heap, (nd, j))
    d[~np.isfinite(d)] = 0.0
    return d


def test_surface_distance_matches_the_heap_version():
    from core.patterns import _neighbours
    from core.patterns_surface import boundary, surface_distance

    rng = np.random.default_rng(1)
    P = np.c_[rng.uniform(0, 1, 300), rng.uniform(0, 0.5, 300), np.zeros(300)]
    P[10] = P[11]  # a duplicate point (a zero-length edge)
    N = np.tile([0.0, 0, 1], (300, 1))
    nb = _neighbours(P, 0.12)
    src = boundary(P, N, nb)
    assert np.allclose(surface_distance(P, nb, src), _dijkstra_old(P, nb, src))
    assert np.all(surface_distance(P, nb, np.zeros(300, bool)) == 0)  # no source: all 0


# --- gaps from the plan ------------------------------------------------------------------------------


def test_footprint_of_contact_geometry():
    from pathlib import Path

    from core.surface_radius import footprint_of
    from core.tool import ToolModel, load_tool

    ee = load_tool("orbital_sander")
    tm = ToolModel(ee, Path(__file__).resolve().parents[1] / ee.urdf)
    assert footprint_of(tm, "pad", 0.025) == pytest.approx(0.075, abs=2e-3)  # the pad's radius
    none = ee.model_copy(update={"tcps": [ee.tcps[0].model_copy(update={"contact_links": []})]})
    tm.ee = none
    assert footprint_of(tm, "pad", 0.025) == 0.025  # no contact geometry: the default
    assert footprint_of(tm, "nope", 0.03) == 0.03


def test_candidate_frames_with_beam_heights():
    import json
    from pathlib import Path

    from shapely.geometry import box

    from core.schema import Base
    from opt.frame_place import FrameSettings, candidate_frames

    root = Path(__file__).resolve().parents[1]
    b = Base.model_validate(json.loads((root / "examples/gantry_overhead/project.json").read_text())["base"])
    fp = box(-1, -0.15, 1, 0.15)
    flat = candidate_frames(b, fp, FrameSettings(shift=0.3, shift_step=0.3))
    tall = candidate_frames(
        b, fp, FrameSettings(shift=0.3, shift_step=0.3, beam_lo=3.0, beam_hi=4.0, beam_steps=3)
    )
    assert len(tall) == 3 * len(flat)
    assert sorted({round(p.xyz[2], 6) for p, _ in tall}) == [3.0, 3.5, 4.0]
    assert {round(p.xyz[2], 6) for p, _ in flat} == {4.4}


def test_coordinated_axes_of_a_gantry():
    import json
    from pathlib import Path

    from core.carrier import Carrier
    from core.coordinated import axis_coords, follow_dofs
    from core.schema import Base

    root = Path(__file__).resolve().parents[1]
    b = Base.model_validate(json.loads((root / "examples/floor_xy_rails/project.json").read_text())["base"])
    c = Carrier(b)
    assert follow_dofs(c) == ["x", "y"]
    P = np.array([[1.0, 2.0, 0.5]])
    L = axis_coords(c, P)
    T = np.linalg.inv(c.T_frame)
    assert L.shape == (1, 2) and np.allclose(L[0], (T[:3, :3] @ P[0] + T[:3, 3])[:2])


def test_use_frame_refused_while_viewing_a_compared_setup():
    from ui.frame_ui import FramePanel

    s = SimpleNamespace(view_stash={"robot_name": "ur20"})
    panel = SimpleNamespace(rows=[{"frame": "f"}], s=s)
    assert FramePanel.use_frame(panel, None, 0) is False
