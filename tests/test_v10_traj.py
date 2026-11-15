"""v10 T13: trajectory optimisation of the joint moves between points (core.traj_opt)."""

import numpy as np
import pytest

from core import traj_opt as to

C, RAD = np.array([0.0, 0.0]), 0.5  # a round obstacle in a 2-joint space


def collides(q) -> bool:
    return float(np.linalg.norm(np.asarray(q)[:2] - C)) < RAD


def clearance(q) -> float:
    return float(np.linalg.norm(np.asarray(q)[:2] - C)) - RAD


def _seed():
    """An RRT-like zig-zag around the obstacle from (-1, 0) to (1, 0)."""
    return [
        np.array(p) for p in ((-1.0, 0.0), (-0.9, 0.8), (-0.2, 0.62), (0.3, 1.1), (0.7, 0.55), (1.0, 0.0))
    ]


def test_resample_even_and_ends_kept():
    Q = to.resample([np.zeros(2), np.array([1.0, 0]), np.array([1.0, 1.0])], 5)
    assert np.allclose(Q[0], 0) and np.allclose(Q[-1], [1, 1])
    assert np.allclose(np.linalg.norm(np.diff(Q, axis=0), axis=1), 0.5)


def test_smooth_path_shorter_clear_and_kept_in_limits():
    seed = _seed()
    lo, hi = np.array([-2.0, -2.0]), np.array([2.0, 1.0])
    out, st = to.smooth_path(collides, clearance, seed, lo, hi, step=0.02, d_safe=0.1)
    assert st.used and st.cost < st.cost_seed
    assert st.length < st.length_seed  # shorter
    Q = np.asarray(out)
    assert np.allclose(Q[0], seed[0]) and np.allclose(Q[-1], seed[-1])  # ends fixed
    assert (Q >= lo - 1e-12).all() and (Q <= hi + 1e-12).all()
    dense = to.resample(out, 400)
    assert not any(collides(q) for q in dense)
    assert min(clearance(q) for q in Q) >= min(st.clear_seed, 0.09) - 1e-4  # not much closer


def test_no_better_keeps_the_seed():
    straight = [np.array([-1.0, 2.0]), np.array([1.0, 2.0])]  # already straight and clear
    out, st = to.smooth_path(collides, clearance, straight, [-3, -3], [3, 3], 0.02)
    assert not st.used and len(out) == 2
    # a seed whose smoothed version would cut through the obstacle: the dense check refuses it
    blocked = lambda q: collides(q) or abs(q[1] - 0.7) < 0.05  # noqa: E731 - a wall the smoothing would cross
    out, st = to.smooth_path(blocked, clearance, _seed(), [-2, -2], [2, 2], 0.01, d_safe=0.0, w_clear=0.0)
    assert not any(blocked(q) for q in to.resample(out, 800)) or not st.used


@pytest.fixture(scope="module")
def app():
    import viser

    from app import build_app

    server = viser.ViserServer(host="127.0.0.1", port=8937, verbose=False)
    st = build_app(server)
    yield st
    server.stop()


@pytest.mark.slow
def test_rail_paths_smoothed_no_new_problems(app):
    from tests.helpers import confirm_all

    s, sim = app["session"], app["sim"]
    assert app["project"].load_example("rail_long_part")
    confirm_all(s)
    app["workflow"].refresh(force=True)
    app["stations"].start(block=True, quick=True)
    app["results"].refresh()
    out = {}
    for on in (False, True):
        sim.smooth.value = on
        sim.build(block=True)
        p = sim.plan
        out[on] = (sum(p.counts().values()), p.smooth)
    assert out[True][0] <= out[False][0]  # smoothing never adds flagged samples
    assert not out[False][1]  # off: no move optimised
    for st in out[True][1]:
        assert st.used is False or (st.length <= st.length_seed * 1.5 and st.cost < st.cost_seed)
