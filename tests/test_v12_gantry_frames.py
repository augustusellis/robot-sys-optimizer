"""v12 items 25-26: Gantry placement testing feedback (2026-11-16).

Done when (25) after *Use this frame* (and a typed frame-pose edit) every drawn carrier body is
where the model has it, fixed posts and beams included; (26) on floor_xy_rails the ranking has
usable frames beside the part, the user's frame is scored and marked, and beside frames keep the
standoff measured to the gantry's swept outline.
"""

from types import SimpleNamespace

import numpy as np
import pytest
import viser
from shapely.affinity import affine_transform

from core.carrier import Carrier
from core.schema import Solution, Station
from opt.frame_place import (
    FrameSettings,
    beside_frames,
    frame_text,
    pick_top,
    rank_frames,
    swept_outline,
    with_frame,
)
from tests.helpers import confirm_all

E = SimpleNamespace(client=None)


@pytest.fixture(scope="module")
def app():
    from app import build_app

    server = viser.ViserServer(host="127.0.0.1", port=8986, verbose=False)
    st = build_app(server)
    yield st
    st["builder_window"].stop()
    server.stop()


def _drawn_off(app) -> dict[str, float]:
    s, car = app["session"], app["carrier"]
    c = Carrier(s.base, resolve=s.resolve)
    model = {b.name: b.T[:3, 3] for b in c.static_bodies() + c.moving_bodies(car.current(c))}
    return {n: float(np.linalg.norm(np.asarray(h.position) - model[n])) for n, h in car._body_handles.items()}


def test_use_this_frame_draws_the_fixed_bodies_too(app):  # 25
    s, frame = app["session"], app["frame"]
    assert app["project"].load_example("gantry_overhead")
    confirm_all(s)
    p = s.base.pose
    turned = p.model_copy(
        update={"xyz": (p.xyz[0] - 0.15, p.xyz[1] - 0.3, p.xyz[2]), "rpy": (0.0, 0.0, np.pi / 2)}
    )
    moved = with_frame(s.base, turned, s.base.T_mount.xyz[2])
    sol = Solution(label="P", stations=[Station(xyz=(0, 0, 0), carrier={"x": 0.5, "y": 0.1})])
    frame.rows = [{"frame": "test frame", "base": moved, "plans": [(sol, {})], "stamps": {}}]
    assert frame.use_frame(E, 0)
    off = _drawn_off(app)
    assert {"post_0", "beam_a", "bridge", "carriage"} <= set(off)
    assert max(off.values()) < 1e-9, off  # v11: posts 4.5-5.0 m, beams 2.2-2.9 m off


def test_typed_frame_pose_redraws_the_fixed_bodies(app):  # 25
    s, car = app["session"], app["carrier"]
    assert app["project"].load_example("gantry_overhead")
    p = s.base.pose
    car.load(
        s.base.model_copy(update={"pose": p.model_copy(update={"xyz": (p.xyz[0] + 0.4, p.xyz[1], p.xyz[2])})})
    )
    assert car.commit(E)
    assert max(_drawn_off(app).values()) < 1e-9


@pytest.fixture(scope="module")
def xy(app):
    from core.robot import Robot
    from core.standoff import footprint
    from opt.reachability import map_for_tcps
    from ui.stations import world_targets

    assert app["project"].load_example("floor_xy_rails")
    s = app["session"]
    rmap = map_for_tcps(Robot(s.robot.spec), s.tool_model, [t.name for t in s.tool.tcps])
    fp = footprint(s.part_mesh(), s.part.T_world_part)
    fs = FrameSettings(z_lo=s.base.T_mount.xyz[2], z_hi=s.base.T_mount.xyz[2])
    ranked = rank_frames(
        s.base, rmap, world_targets(s), s.part_mesh(), s.part.T_world_part, fp, fs,
        resolve=s.resolve, obstacles=s.obstacles(),
    )  # fmt: skip
    return s, fp, fs, ranked


def test_floor_gantry_gets_frames_beside_the_part(xy):  # 26
    s, fp, fs, ranked = xy
    usable = [f for f in ranked if not f.reason and not f.own]
    assert len(usable) >= 2  # v11: none ("No frame works")
    best = usable[0]
    assert best.uncovered == 0
    c = Carrier(best.base, resolve=s.resolve)
    hull = c._hulls(c.static_bodies())
    assert hull.distance(fp) >= s.base.standoff_m.carrier_part_min - 0.01  # beside, not over the wall


def test_own_frame_is_scored_marked_and_listed(xy):  # 26
    s, _, fs, ranked = xy
    mine = [f for f in ranked if f.own]
    assert len(mine) == 1 and not mine[0].reason and mine[0].pose == s.base.pose
    assert "(your frame)" in frame_text(mine[0])
    top = pick_top(ranked, 1, fs.distinct_m)
    assert any(f.own for f in top)  # listed even when it is not the best


def test_beside_frames_keep_the_standoff_to_the_swept_outline(xy):  # 26
    s, fp, fs, _ = xy
    outline = swept_outline(s.base, s.resolve)
    poses = beside_frames(s.base, fp, fs)
    assert len(poses) == 4 * 2 * len(np.arange(fs.d_min, fs.d_max + 1e-9, fs.step)) * 5
    for pose in poses[:: max(1, len(poses) // 40)]:
        y, (x0, y0) = pose.rpy[2], pose.xyz[:2]
        m = [np.cos(y), -np.sin(y), np.sin(y), np.cos(y), x0, y0]
        d = affine_transform(outline, m).distance(fp)
        assert fs.d_min - 0.02 <= d <= fs.d_max + 0.05, d
