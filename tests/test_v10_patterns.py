"""v10 T15: coverage patterns along the surface, wide turn-arounds, a pattern per target group."""

import numpy as np
import pytest

from core.path_order import ordered_sequences
from core.patterns import GROUP_DEFAULT, PATTERNS, PatternSettings, order_points
from core.patterns_surface import boundary, surface_distance, turn_lifts
from core.schema import Solution, Station, TargetGroup

H = 0.02  # point spacing


def _cylinder_patch(R=0.2, wrap=4 * np.pi / 3):
    """Points on a cylinder side wrapping ``wrap`` rad (the mean normal is nearly zero)."""
    th = np.arange(-wrap / 2, wrap / 2 + 1e-9, H / R)
    zs = np.arange(0, 0.3 + 1e-9, H)
    P = np.array([(R * np.cos(t), R * np.sin(t), z) for t in th for z in zs])
    N = np.c_[P[:, 0], P[:, 1], np.zeros(len(P))] / R
    return P, N


def _stats(P, o):
    st = np.linalg.norm(np.diff(P[o.order], axis=0), axis=1)
    return st.sum(), int((st > 2 * H).sum())


def test_surface_patterns_cover_a_wrapped_face_without_jumps():
    P, N = _cylinder_patch()
    for pat in ("Contour-parallel", "Spiral (along the surface)"):
        o = order_points(P, N, PatternSettings(pat))
        assert sorted(o.order) == list(range(len(P)))  # every point once
        length, jumps = _stats(P, o)
        assert jumps == 0, pat
        assert np.all(np.diff(o.rows) >= 0)  # passes from the boundary inward
    # the projected patterns lose their way on the same face
    _, jumps_c = _stats(P, order_points(P, N, PatternSettings("Concentric")))
    _, jumps_s = _stats(P, order_points(P, N, PatternSettings("Spiral")))
    assert jumps_c > 0 and jumps_s > 100


def test_boundary_and_distance_along_the_surface():
    P, N = _cylinder_patch()
    from core.patterns import _neighbours

    nb = _neighbours(P, 1.75 * H)
    b = boundary(P, N, nb)
    z = P[:, 2]
    ang = np.arctan2(P[:, 1], P[:, 0])
    edge = (z < 1e-9) | (z > z.max() - 1e-9) | (ang < ang.min() + 1e-9) | (ang > ang.max() - 1e-9)
    assert np.array_equal(b, edge)
    d = surface_distance(P, nb, b)
    mid = np.argmin(np.abs(ang) + np.abs(z - 0.15))
    assert d[mid] == pytest.approx(0.15, abs=0.03)  # half the height, along the surface


def test_turn_lifts_only_on_wide_row_changes():
    P = np.array([[0, 0, 0], [0.1, 0, 0], [0.1, 0.02, 0], [0, 0.02, 0], [0, 0.2, 0]], float)
    rows = np.array([0, 0, 1, 1, 2])
    assert list(turn_lifts(P, rows, footprint=0.03)) == [False, False, False, True]
    assert not turn_lifts(P, rows, footprint=0.0).any()


def test_group_pattern_overrides_the_paths_pattern():
    assert GROUP_DEFAULT not in PATTERNS
    g = TargetGroup(name="top", pattern="Contour-parallel")
    assert g.pattern == "Contour-parallel" and TargetGroup(name="x").pattern is None
    P, N = _cylinder_patch(wrap=np.pi / 2)
    ids = list(range(len(P)))
    sol = Solution(stations=[Station(xyz=(0, 0, 0))], assignment={i: 0 for i in ids})

    class R:
        q = np.zeros(6)
        status = "reachable"

    res = {i: R() for i in ids}
    pts = {i: (P[i], N[i]) for i in ids}
    groups = {i: "top" for i in ids}
    zig = ordered_sequences(sol, res, pts, PatternSettings("Zigzag"), group_of=groups)
    own = ordered_sequences(
        sol,
        res,
        pts,
        PatternSettings("Zigzag"),
        group_of=groups,
        pattern_of=lambda name: PatternSettings("Contour-parallel") if name == "top" else None,
    )
    ref = order_points(P, N, PatternSettings("Contour-parallel"))
    assert own[0][0].pids == [ids[i] for i in ref.order] and own[0][0].pids != zig[0][0].pids
    # no Paths pattern: the group's own one still applies
    alone = ordered_sequences(
        sol, res, pts, None, group_of=groups, pattern_of=lambda n: PatternSettings("Contour-parallel")
    )
    assert alone[0][0].pids == own[0][0].pids


def test_app_group_pattern_and_turn_lift_reach_the_paths():
    import viser

    from app import build_app
    from ui import sim_fields

    server = viser.ViserServer(host="127.0.0.1", port=8945, verbose=False)
    try:
        app = build_app(server)
        s, sim = app["session"], app["sim"]
        assert app["project"].load_example("rail_long_part")
        name = s.targets.groups[0].name
        groups = [g.model_copy(update={"pattern": "Spiral (along the surface)"}) for g in s.targets.groups]
        s.targets = s.targets.model_copy(update={"groups": groups})
        sim.turn_lift.value = True
        pattern, pattern_of = sim_fields.group_patterns(sim, sim.pattern_settings())
        assert pattern_of(name).pattern == "Spiral (along the surface)"
        assert pattern_of("no such group") is None
        # the sander's pad (75 mm radius) sizes the turn lifts: rows further apart than 150 mm
        assert pattern_of(name).turn_lift_m == pytest.approx(0.15, abs=1e-3)
        if pattern is not None:
            assert pattern.turn_lift_m == pytest.approx(0.15, abs=1e-3)
        sim.turn_lift.value = False
    finally:
        server.stop()
