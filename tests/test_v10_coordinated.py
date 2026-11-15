"""v10 T16: coordinated rail / gantry motion (core.coordinated).

Done when the rail example can be built both ways and the cycle times are compared.
"""

import numpy as np
import pytest

from core.cycle_time import CycleSettings, _kind, path_time
from core.path_types import PathPlan, PathSample, StationPath


def _s(x, s_rail, kind=""):
    T = np.eye(4)
    T[0, 3] = x
    return PathSample(0, None, 1, 2, 0.5, T, "tip", np.zeros(6), kind=kind, carrier={"s": s_rail})


def test_a_moving_carrier_on_the_surface_is_an_arm_run_not_a_carrier_move():
    a, b = _s(0.0, 0.0), _s(0.01, 0.01)
    assert _kind(a, b) == "process"
    assert _kind(a, _s(0.01, 0.01, "carrier")) == "carrier"
    assert _kind(a, _s(0.01, 0.01, "home")) == "carrier"  # a joint move does not drive the carrier


def test_coordinated_run_no_faster_than_the_carrier():
    xs = np.linspace(0, 1.0, 101)  # the tool and the carrier both move 1 m
    plan = PathPlan([StationPath(0, np.eye(4), {"s": 0.0}, [_s(x, x) for x in xs])], 0.01)
    fast = CycleSettings(settle_s=0.0, process_speed=1.0, carrier_speed=0.2)
    for timing in ("topp", "trapezoid"):
        out = path_time(plan, CycleSettings(**{**fast.__dict__, "timing": timing}))
        assert out["carrier"] == 0.0 and out["process"] >= 1.0 / 0.2 * 0.99, timing


@pytest.mark.slow
def test_rail_both_ways_cycle_times_compared():
    import viser

    from app import build_app
    from tests.helpers import confirm_all
    from ui.sim_fields import MOTION, motion_line

    server = viser.ViserServer(host="127.0.0.1", port=8949, verbose=False)
    try:
        app = build_app(server)
        s, sim = app["session"], app["sim"]
        assert app["project"].load_example("rail_long_part")
        confirm_all(s)
        app["workflow"].refresh(force=True)
        app["stations"].start(block=True)
        app["results"].refresh()
        sol = app["results"].selected()
        assert len(sol.stations) >= 2
        assert sim.motion_mode.visible  # a movable rail
        out = {}
        for m in MOTION:
            sim.motion_mode.value = m
            sim.build(block=True)
            out[m] = (dict(sim.cycle), sim.plan)
        (cu, _), (cc, pc) = out[MOTION[0]], out[MOTION[1]]
        assert sum(pc.counts().values()) == 0  # coordinated paths are clean
        reached = {p for p, r in s.result_points[sol.label].items() if r.status != "unreachable"}
        assert {x.b for x in pc.samples if not x.kind} >= reached  # every reached point visited
        sv = [x.carrier["s"] for x in pc.samples if not x.kind]
        lo, hi = s.carrier().bounds[0]
        assert max(sv) - min(sv) > 1.0 and lo - 1e-9 <= min(sv) and max(sv) <= hi + 1e-9  # it follows
        assert cc["carrier"] == 0.0 and cu["carrier"] > 0  # no stop-to-stop drives
        assert cc["total"] < cu["total"]
        line = motion_line(sim, sol.label)
        assert "coordinated" in line and "stop and go" in line
        sections = dict(app["results"].export.sections(sol))
        assert "Carrier motion: coordinated" in sections["Carrier motion"]  # its own section (v10 T22)
        # playback moves the carrier along with the arm
        moved = []
        sim.move_carrier = lambda v, moved=moved: moved.append(dict(v))
        for i in (len(pc.samples) // 3, 2 * len(pc.samples) // 3):
            sim.go_to(i)
        assert len(moved) == 2 and moved[0]["s"] != moved[1]["s"]
        sim.motion_mode.value = MOTION[0]
    finally:
        server.stop()
