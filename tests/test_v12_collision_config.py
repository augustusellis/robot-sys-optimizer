"""v12 item 10: the collision settings as one value (core.collision_config).

Done when a scene uses the config it is given (not whatever the process's globals say), the check
pool's spec and worker jobs carry that one value, and the old module globals are gone.
"""

import pickle

import numpy as np
import trimesh

from core import collision
from core import collision_config as cc
from core.collision import CollisionScene
from core.robot import Robot
from core.tool import ToolModel, load_tool
from opt.parallel import CheckerSpec
from tests.test_v11_direction import REPO


def _part():
    m = trimesh.creation.box((0.4, 0.4, 0.4))
    T = np.eye(4)
    T[:3, 3] = (0.9, 0.0, 0.5)
    return m, T


def test_scene_uses_its_config_not_the_current_one():
    r = Robot.from_registry("ur20")
    part, T = _part()
    old = cc.current()
    try:
        cc.update(arm_pad=0.2, dress_m=0.05)  # the GUI's settings ...
        own = CollisionScene(r, None, part, np.eye(4), T, config=cc.CollisionConfig())
        assert own.arm_pad == 0.0 and own.margins.dress == 0.0  # ... do not reach a scene given its own
        default = CollisionScene(r, None, part, np.eye(4), T)
        assert default.arm_pad == 0.2 and default.margins.dress == 0.05
    finally:
        cc.use(old)


def test_config_is_one_picklable_value_in_the_check_spec():
    r = Robot.from_registry("ur20")
    part, T = _part()
    cfg = cc.CollisionConfig(arm_pad=0.01, link_pad={3: 0.02}, part_keep=(1, 2))
    ee = load_tool("orbital_sander")
    tm = ToolModel(ee, REPO / ee.urdf)
    spec = CheckerSpec(r.spec, tm, part, np.eye(4), T, collision=cfg)
    back = pickle.loads(pickle.dumps(spec))
    assert back.collision == cfg and back.collision.key() == cfg.key()
    assert CheckerSpec(r.spec, tm, part, np.eye(4), T).key() != spec.key()  # pools keyed by it
    checker = back.build()
    assert checker.scene.config == cfg and checker.scene.link_pad == {3: 0.02}


def test_jobs_carry_the_config(monkeypatch):
    from ui import jobs

    seen = {}

    class FakeRunner:
        def run(self, target, payload, on_msg, cancelled):
            seen.update(payload)

    monkeypatch.setattr(jobs, "USE_WORKER", True)
    monkeypatch.setattr(jobs, "runner", lambda: FakeRunner())
    old = cc.current()
    try:
        cc.update(dress_m=0.03)
        jobs.call("x:y", {"spec": {}}, None, lambda: False, in_process=False)
        assert seen["collision"].dress_m == 0.03  # the worker uses exactly the GUI's value
    finally:
        cc.use(old)


def test_old_globals_are_gone():
    for name in ("DRESS_M", "HOSE", "GEOM", "PART_KEEP", "ARM_PAD", "LINK_PAD", "PART_LOW", "geom_state"):
        assert not hasattr(collision, name), name
