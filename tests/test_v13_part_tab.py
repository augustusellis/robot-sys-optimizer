"""v13 #4: the package builder's Part tab places the part in the world, and saves / reopens it with
its targets as a part package."""

import numpy as np
import pytest
import viser

from core import part_package as pp
from core.schema import TargetGroup, TargetPoint, Targets


@pytest.fixture(scope="module")
def app():
    from app import build_app

    server = viser.ViserServer(host="127.0.0.1", port=8986, verbose=False)
    a = build_app(server)
    yield a
    a["builder_window"].stop()
    server.stop()


def _targets(n: int) -> Targets:
    pts = [TargetPoint(id=i, p=(0.1 * i, 0.0, 0.01), n=(0, 0, 1), group="top") for i in range(n)]
    return Targets(groups=[TargetGroup(name="top")], points=pts)


def test_place_save_and_reopen(app, tmp_path):
    b, s = app["builder"], app["session"]
    panel = b.part_pkg
    b._set_kind("part")
    assert panel.folder.visible
    b.name.value = "plate A"
    panel.xyz.value = (1200.0, -300.0, 450.0)
    panel.rpy.value = (0.0, 0.0, 30.0)
    assert panel.place_in_project()  # the builder holds no mesh: the project's part is placed
    assert np.allclose(s.part.T_world_part[:3, 3], (1.2, -0.3, 0.45))
    assert panel.editing == "plate A" and all(w.visible for w in panel.notes)
    s.targets = _targets(4)
    panel.parts_folder.value = str(tmp_path)
    dest = panel.save_package()
    assert dest == tmp_path / "plate_A"
    pkg = pp.load_part_package(dest)
    assert len(pkg.targets.points) == 4 and np.allclose(pkg.part.pose.xyz, (1.2, -0.3, 0.45))
    assert np.isclose(pkg.part.pose.rpy[2], np.radians(30.0))
    assert panel.save_package() is None  # the name is taken: not replaced without asking
    assert panel.save_package(overwrite=True) == dest

    s.targets = _targets(1)  # something else in the project
    s.part.T_world_part = np.eye(4)
    assert panel.list_folder() == ["plate_A"]
    assert panel.open_package(name="plate_A")
    assert len(s.targets.points) == 4 and np.allclose(s.part.T_world_part[:3, 3], (1.2, -0.3, 0.45))
    assert panel.xyz.value == pytest.approx((1200.0, -300.0, 450.0))
    assert len(app["targets"].s.targets.points) == 4


def test_tab_only_on_parts(app):
    b = app["builder"]
    b._set_kind("tool")
    assert not b.part_pkg.folder.visible
