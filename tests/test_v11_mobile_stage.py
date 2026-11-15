"""v11 task 9: a mobile robot with a vertical stage, built in the package builder.

Done when a mobile package built with *Add mobile base* + *Add vertical stage* exports a URDF whose
robot mount rides the lift joint (forward kinematics) and, as an external axis, plans with the lift
as a DOF.
"""

import numpy as np
import pytest
import viser

from core.carrier import Carrier
from core.schema import Base
from core.urdf_package import to_base, write_package
from ui.package_presets import CHASSIS, PLATE, add_mobile_base, add_vertical_stage


@pytest.fixture
def builder(tmp_path):
    from ui.package_builder import PackageBuilder

    class S:
        project_dir = tmp_path
        robot_name = "ur20"

    server = viser.ViserServer(host="127.0.0.1", port=8977, verbose=False)
    b = PackageBuilder(server, S())
    b._set_kind("mobile")
    yield b
    server.stop()


def test_buttons_build_chassis_mast_and_lift(builder):
    b = builder
    assert b.presets.stage_btn.visible
    add_mobile_base(b)
    mast, plate = add_vertical_stage(b, mast_height=1.2, travel=0.8)
    stages = {x.name: x.stage for x in b.pkg.parts}
    assert stages == {"chassis": "chassis", "mast": "chassis", "lift_plate": "lift"}
    assert b.pkg.lift == (0.0, 0.8) and b.tlift.value == (0.0, 800.0)  # the builder's field follows
    top = CHASSIS[2]
    assert mast.pose.xyz[2] == pytest.approx(top) and plate.pose.xyz[2] == pytest.approx(
        top + 1.2 - 0.8 - PLATE[2]
    )
    assert b.pkg.mount.xyz[2] == pytest.approx(plate.pose.xyz[2] + PLATE[2])
    with pytest.raises(ValueError):
        add_vertical_stage(b, travel=0.0)
    b._set_kind("tool")
    assert not b.presets.stage_btn.visible


def test_urdf_mount_rides_the_lift_and_plans_as_a_dof(builder, tmp_path):
    import pinocchio as pin

    b = builder
    add_vertical_stage(b, mast_height=1.5, travel=1.0)  # adds the chassis too
    assert any(x.name == "chassis" for x in b.pkg.parts)
    urdf = write_package(b.pkg, None, tmp_path / "pkg" / "amr")
    model = pin.buildModelFromUrdf(str(urdf))
    data = model.createData()
    names = list(model.names)
    assert "lift" in names
    fid = model.getFrameId("robot_mount", pin.FrameType.BODY)  # the link (its joint frame has the same name)
    zs = []
    for lift in (0.0, 0.6):
        q = pin.neutral(model)
        q[model.joints[names.index("lift")].idx_q] = lift
        pin.framesForwardKinematics(model, data, q)
        zs.append(float(data.oMf[fid].translation[2]))
    assert zs[1] - zs[0] == pytest.approx(0.6, abs=1e-9)
    assert zs[0] == pytest.approx(b.pkg.mount.xyz[2], abs=1e-9)
    base = to_base(b.pkg, (tmp_path / "pkg" / "amr" / "meshes").as_posix(), Base())
    c = Carrier(base)
    assert base.carrier == "mobile" and "lift" in c.dofs and base.joints.lift == (0.0, 1.0)
    assert {x.name: x.attach for x in base.bodies}["lift_plate"] == "lift"
    lo = c.T_world_base({"x": 0, "y": 0, "yaw": 0, "lift": 0.0})
    hi = c.T_world_base({"x": 0, "y": 0, "yaw": 0, "lift": 1.0})
    assert hi[2, 3] - lo[2, 3] == pytest.approx(1.0) and np.allclose(hi[:2, 3], lo[:2, 3])
