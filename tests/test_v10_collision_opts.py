"""v10 T19: padding per robot link, surfaces kept exact on multi-mesh parts, Cancel for Simplify."""

import numpy as np
import pytest
import trimesh

from core import collision
from core.collision import CollisionScene, link_joints
from core.robot import Robot
from core.schema import Pose, Primitive
from core.stayout import obstacles, scene_entries


@pytest.fixture
def robot():
    return Robot.from_registry("ur20")


def _scene(robot, zone):
    return CollisionScene(robot, None, None, np.eye(4), None, obstacles=scene_entries(obstacles([zone])))


def test_padding_one_link_only(robot):
    q = np.radians([0, -90, 90, -90, -90, 0])
    joints = link_joints(robot)
    assert {"forearm_link", "wrist_1_link"} <= set(joints)
    # a small zone 40 mm beside the forearm's collision shape: clear unless the forearm is padded
    T = robot.frame_pose(q, "forearm_link")
    p = T[:3, 3] + np.array([0.0, 0.0, 0.0])
    free = None
    for gap in np.arange(0.0, 0.6, 0.01):  # walk out sideways until the forearm is clear
        zone = Primitive(
            shape="box",
            name="z",
            attach="fixed",
            size=(0.05, 0.05, 0.05),
            pose=Pose(xyz=tuple(p + np.array([0.0, -gap, 0.0]))),
        )
        if not _scene(robot, zone).collides(q):
            free = gap + 0.02
            break
    assert free is not None
    zone = Primitive(
        shape="box",
        name="z",
        attach="fixed",
        size=(0.05, 0.05, 0.05),
        pose=Pose(xyz=tuple(p + np.array([0.0, -free, 0.0]))),
    )
    old = collision.geom_state()
    try:
        assert not _scene(robot, zone).collides(q)
        collision.set_geom_state(({}, (), 0.0, None, 1.0, {joints["forearm_link"]: 0.05}))
        sc = _scene(robot, zone)
        assert sc.collides(q)  # padded forearm reaches the zone
        forearm = [g for g in sc.robot_ids if sc.gm.geometryObjects[g].parentJoint == joints["forearm_link"]]
        assert forearm and sc.pad_of(forearm[0]) == pytest.approx(0.05)
        collision.set_geom_state(({}, (), 0.0, None, 1.0, {joints["wrist_3_link"]: 0.05}))
        assert not _scene(robot, zone).collides(q)  # another link padded: still clear
    finally:
        collision.set_geom_state(old)


def test_parse_link_pad(robot):
    from ui.collision_settings import parse_link_pad

    out, msg = parse_link_pad("forearm_link 20, wrist_1_link=10; nope 5, upper_arm_link x", robot)
    j = link_joints(robot)
    assert out == {j["forearm_link"]: 0.02, j["wrist_1_link"]: 0.01}
    assert "2 link(s)" in msg and "nope 5" in msg and "upper_arm_link x" in msg
    assert parse_link_pad("", robot) == ({}, "")


def test_kept_faces_split_onto_mesh_parts():
    from ui.package_keep import split_keep

    a = trimesh.creation.box(extents=(0.2, 0.2, 0.2))
    b = trimesh.creation.box(extents=(0.1, 0.1, 0.1))
    pose_b = Pose(xyz=(0.5, 0.0, 0.0))
    b_world = b.copy()
    b_world.apply_translation((0.5, 0, 0))
    whole = trimesh.util.concatenate([b_world, a])  # the project's part: b first, then a
    keep = [0, 1, len(b.faces) + 4]  # two faces of b, one of a
    per = split_keep(whole, keep, {"a": a, "b": b}, {"a": Pose(), "b": pose_b})
    assert per == {"b": [0, 1], "a": [4]}


def test_simplify_can_be_cancelled(monkeypatch):
    import viser

    from core.urdf_package import Frame, Package, Part
    from ui import package_collision as pc

    server = viser.ViserServer(host="127.0.0.1", port=8955, verbose=False)
    try:
        from app import build_app

        app = build_app(server)
        b = app["builder"]
        b.pkg = Package(
            kind="tool",
            name="t",
            parts=[Part(name=f"p{k}", stage="tool", shape="box", size=(0.1, 0.1, 0.1)) for k in range(3)],
            tcps=[Frame(name="tcp")],
        )
        calls = []

        def slow(p, part, d):
            calls.append(part.name)
            b.collision._stop.set()  # Cancel pressed while the first part is simplified
            return None

        monkeypatch.setattr(pc, "part_collision", slow)
        b.collision.simplify()
        assert calls == ["p0"] and set(b.collision.done) == {"p0"}
        assert "Cancelled: 2 part(s) not simplified." in b.collision.info.content
        assert not b.collision.cancel.visible and not b.collision.run.disabled
    finally:
        app["builder_window"].stop()
        server.stop()
