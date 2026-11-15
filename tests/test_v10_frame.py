"""v10 T18: gantry placement: continuous refinement, distinct top frames, beam height with legs."""

import json
from pathlib import Path

import numpy as np
import pytest

from core.schema import Base, Pose
from opt.frame_place import FrameScore, FrameSettings, pick_top, refine_frame, with_frame

REPO = Path(__file__).resolve().parents[1]


def _gantry() -> Base:
    return Base.model_validate(
        json.loads((REPO / "examples/gantry_overhead/project.json").read_text())["base"]
    )


def test_raising_the_frame_stretches_the_legs_only():
    b = _gantry()
    up = with_frame(b, b.pose.model_copy(update={"xyz": (0.0, 0.0, 4.9)}), b.T_mount.xyz[2])
    old = {x.name: x for x in b.bodies}
    for x in up.bodies:
        o = old[x.name]
        if x.name.startswith("post"):  # on the floor: 0.5 m taller, bottom still at z = 0
            assert x.size[2] == pytest.approx(o.size[2] + 0.5)
            assert up.pose.xyz[2] + x.pose.xyz[2] == pytest.approx(0.0)
        else:  # beams, bridge, carriage: move with the frame
            assert x == o
    from core.carrier import Carrier

    c = Carrier(up)
    lows = [
        float((bd.mesh.vertices @ bd.T[:3, :3].T + bd.T[:3, 3])[:, 2].min())
        for bd in c.static_bodies()
        if bd.name.startswith("post")
    ]
    assert lows and all(abs(z) < 1e-6 for z in lows)
    with pytest.raises(ValueError):
        with_frame(b, b.pose.model_copy(update={"xyz": (0.0, 0.0, -0.1)}), b.T_mount.xyz[2])


def _f(x, y, unc=0, stops=2, manip=0.5, yaw=0.0):
    return FrameScore(Pose(xyz=(x, y, 4.4), rpy=(0.0, 0.0, yaw)), -0.85, unc, stops, manip)


def test_pick_top_skips_frames_that_score_alike_nearby():
    ranked = [_f(0, 0), _f(0.3, 0, manip=0.49), _f(1.2, 0), _f(0.3, 0, stops=3), _f(0, 0, yaw=1.57)]
    top = pick_top(ranked, 3, 0.6)
    assert [(f.pose.xyz[0], f.stops, f.pose.rpy[2]) for f in top] == [
        (0, 2, 0.0),
        (1.2, 2, 0.0),
        (0.3, 3, 0.0),
    ]


def test_refinement_finds_the_better_spot_between_grid_points():
    target = np.array([0.37, -0.12])

    def score(pose, h):
        d = float(np.linalg.norm(np.array(pose.xyz[:2]) - target))
        return FrameScore(pose, h, 0, 2, 1.0 - d)

    st = FrameSettings(shift_step=0.3, refine_evals=60)
    start = score(Pose(xyz=(0.3, 0.0, 4.4)), -0.85)
    best = refine_frame(start, score, st)
    assert np.linalg.norm(np.array(best.pose.xyz[:2]) - target) < 0.03 < np.linalg.norm([0.07, 0.12])
    assert best.key() < start.key()
