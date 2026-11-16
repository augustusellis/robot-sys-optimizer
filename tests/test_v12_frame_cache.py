"""v12 item 20: frame scoring shares coverage rows and stop checks between frames.

Done when the candidate stops of every frame lie on one world lattice (frames shifted against each
other share stop poses), and scoring with the shared cache gives exactly the scores without it.
Measured (docs/V12_PLAN.md): gantry_overhead, 51 frames, 42.3 -> 8.0 s.
"""

import numpy as np
import pytest
import viser

from core.carrier import Carrier
from opt.frame_place import FrameSettings, candidate_frames, lattice_anchor, score_frame, with_frame


@pytest.fixture(scope="module")
def setup():
    from app import build_app
    from core.robot import Robot
    from core.standoff import footprint
    from opt.reachability import map_for_tcps
    from ui.stations import world_targets

    server = viser.ViserServer(host="127.0.0.1", port=8985, verbose=False)
    st = build_app(server)
    try:
        assert st["project"].load_example("gantry_overhead")
        s = st["session"]
        rmap = map_for_tcps(Robot(s.robot.spec), s.tool_model, [c.name for c in s.tool.tcps])
        fp = footprint(s.part_mesh(), s.part.T_world_part)
        yield s, rmap, world_targets(s), fp
    finally:
        st["builder_window"].stop()
        server.stop()


def test_stops_lie_on_one_world_lattice(setup):
    s, *_, fp = setup
    fs = FrameSettings()
    frames = candidate_frames(s.base, fp, fs)[:6]
    step = fs.candidate_step
    for pose, h in frames:
        c = Carrier(with_frame(s.base, pose, h))
        a = lattice_anchor(c, step)
        for v in c.samples(step, anchor=a)[:20]:
            p = c.T_world_carriage(v)[:3, 3]
            for k in range(sum(d in ("x", "y", "s") for d in c.dofs)):
                axis = c.T_frame[:3, k]
                r = (p @ axis) / step
                assert abs(r - round(r)) < 1e-6  # on the lattice through the world origin


def test_cache_gives_the_same_scores(setup):
    s, rmap, t, fp = setup
    fs = FrameSettings()
    frames = candidate_frames(s.base, fp, fs)[:8]
    cache: dict = {}
    for pose, h in frames:
        b = with_frame(s.base, pose, h)
        args = (b, rmap, t, s.part_mesh(), s.part.T_world_part, fs, s.resolve, s.obstacles())
        plain, shared = score_frame(*args), score_frame(*args, cache=cache)
        assert plain.reason == shared.reason
        assert plain.uncovered == shared.uncovered and plain.stops == shared.stops
        assert plain.manip == pytest.approx(shared.manip, abs=1e-12)
    rows = [k for k in cache if k != "accept"]
    assert len(rows) < 8 * 200 and cache["accept"]  # rows and stop checks were shared
    assert np.isfinite(sum(v[0].sum() for k, v in cache.items() if k != "accept"))
