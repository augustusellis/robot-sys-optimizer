"""v10 T14: time-parameterised trajectories (core.trajectory) for the cycle time and playback."""

import numpy as np
import pytest

from core.cycle_time import CycleSettings, path_time, trapezoid
from core.path_types import PathPlan, PathSample, StationPath
from core.trajectory import joint_rates, time_path


def test_straight_move_is_the_trapezoid():
    Q = np.c_[np.linspace(0, 1, 101), np.zeros(101)]
    assert time_path(Q, 1.0, 1.4)[-1] == pytest.approx(trapezoid(1.0, 1.0, 1.4), rel=1e-4)
    short = np.c_[np.linspace(0, 0.3, 31), np.zeros(31)]  # triangle: never reaches cruise speed
    assert time_path(short, 1.0, 1.4)[-1] == pytest.approx(trapezoid(0.3, 1.0, 1.4), rel=1e-3)


def test_jerk_limit_gives_the_s_curve():
    Q = np.c_[np.linspace(0, 1, 101), np.zeros(101)]
    t = time_path(Q, 1.0, 1.4, 5.0)
    assert t[-1] == pytest.approx(1 / 1.0 + 1.0 / 1.4 + 1.4 / 5.0, rel=2e-3)  # + a / j
    V, A, _ = joint_rates(Q, t)
    # the averaged profile never exceeds the limits; finite differences between the 100 waypoints
    # read the acceleration up to ~10 % high (each segment's speed is a step)
    assert np.abs(V).max() <= 1.0 + 1e-6 and np.abs(A).max() <= 1.4 * 1.15


def test_limits_hold_on_a_curved_path_per_joint():
    s = np.linspace(0, 1, 200)
    Q = np.c_[np.sin(2 * s), 0.5 * s, np.cos(3 * s) - 1]
    v, a = np.array([1.0, 0.2, 2.0]), np.array([2.0, 0.5, 3.0])
    t = time_path(Q, v, a)
    V, A, _ = joint_rates(Q, t)
    assert (np.abs(V) <= v * 1.001 + 1e-9).all()
    assert (np.abs(A[1:-1]) <= a * 1.05 + 1e-6).all()  # zero-order discretisation: a few %
    assert t[-1] > 0.5 / 0.2  # joint 2 (0.5 rad at 0.2 rad/s) sets the pace


def _sample(x, q, kind=""):
    T = np.eye(4)
    T[0, 3] = x
    return PathSample(0, 0, 1, 2, 0.5, T, "tip", np.asarray(q, float), kind=kind)


def test_path_time_surface_run_capped_by_tcp_speed_and_times():
    xs = np.linspace(0.0, 0.5, 51)  # 0.5 m on the surface, small joint motion
    plan = PathPlan([StationPath(0, np.eye(4), {}, [_sample(x, [x * 0.2, 0, 0, 0, 0, 0]) for x in xs])], 0.01)
    cs = CycleSettings(settle_s=0.0)
    times: list = []
    topp = path_time(plan, cs, times=times)
    trap = path_time(plan, CycleSettings(settle_s=0.0, timing="trapezoid"))
    # the TCP's 100 mm/s sets the pace: the same as the trapezoid (no jerk limit on the TCP)
    assert topp["process"] == pytest.approx(trap["process"], rel=0.03)
    assert len(times) == len(xs) and np.all(np.diff(times) >= 0) and times[-1] == pytest.approx(topp["total"])
    # joints slower than the TCP would need: the joint limit sets the pace
    slow = CycleSettings(settle_s=0.0, joint_speed=0.01)
    assert path_time(plan, slow)["process"] > 0.1 / 0.01 * 0.9


def test_trapezoid_mode_keeps_the_pre_v10_numbers():
    q0, q1 = np.zeros(6), np.r_[1.0, 0.5, 0, 0, 0, 0]
    home = [_sample(0.0, q0, "home"), _sample(0.0, (q0 + q1) / 2, "home"), _sample(0.0, q1, "home")]
    plan = PathPlan([StationPath(0, np.eye(4), {}, home)], 0.01)
    cs = CycleSettings(settle_s=1.0, timing="trapezoid")
    out = path_time(plan, cs)
    assert out["joint"] == pytest.approx(trapezoid(1.0, cs.joint_speed, cs.joint_acc))
    assert out["settle"] == 1.0 and out["total"] == pytest.approx(out["joint"] + 1.0)
    topp = path_time(plan, CycleSettings(settle_s=1.0))
    assert topp["joint"] > out["joint"]  # the jerk limit adds a / j


@pytest.mark.slow
def test_rail_paths_timed_and_played_at_program_timing():
    import viser

    from app import build_app
    from tests.helpers import confirm_all

    server = viser.ViserServer(host="127.0.0.1", port=8939, verbose=False)
    try:
        app = build_app(server)
        s, sim, cyc = app["session"], app["sim"], app["cycle"]
        assert app["project"].load_example("rail_long_part")
        confirm_all(s)
        app["workflow"].refresh(force=True)
        app["stations"].start(block=True, quick=True)
        app["results"].refresh()
        sim.build(block=True)
        topp = dict(sim.cycle)
        assert len(sim.times) == len(sim.plan.samples) and np.all(np.diff(sim.times) >= 0)
        cyc.timing.value = "Trapezoid per move (pre-v10)"
        assert s.cycle.timing == "trapezoid"
        trap = sim._cycle_of(sim.plan)
        assert topp["total"] != pytest.approx(trap["total"]) and topp["process"] > 0
        cyc.timing.value = "Time-optimal (speed, acceleration, jerk limits)"
        # playback at program timing reaches the end and stops
        sim.timed.value = True
        sim.frame.value = len(sim.plan.samples) - 5
        sim.toggle()
        sim._player.join(timeout=30)
        assert int(sim.frame.value) == len(sim.plan.samples) - 1
    finally:
        server.stop()
