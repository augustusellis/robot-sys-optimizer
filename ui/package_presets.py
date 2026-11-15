"""Package builder -> Mobile robot: build the full robot from a chassis and a vertical stage
(v11 task 9).

**Add mobile base** puts a chassis box on the floor (stage *chassis*: it drives with x, y, yaw).
**Add vertical stage** puts a mast on the chassis (stage *chassis*), a carriage plate that rises
on it (stage *lift*: core.urdf_package writes it on the prismatic ``lift`` joint, link
``lift_stage``), sets the *Lift travel* and moves the *Robot mount* onto the carriage plate, so the
URDF's ``robot_mount`` frame rides the lift: world -> x -> y -> yaw -> chassis -> lift -> mount.
Both are ordinary parts afterwards (resize, move or replace them with meshes).
"""

from __future__ import annotations

from core.schema import Pose
from core.urdf_package import Part

CHASSIS = (1.0, 0.7, 0.35)  # m, L x W x H
MAST = (0.18, 0.18)  # m, the mast's section
PLATE = (0.4, 0.4, 0.04)  # m, the carriage plate the robot stands on


def add_mobile_base(b, size=CHASSIS) -> Part:
    """A chassis box on the floor (bottom face on z = 0)."""
    part = Part(name="chassis", stage="chassis", shape="box", size=tuple(float(v) for v in size))
    b._add(part)
    return b.pkg.parts[-1]


def chassis_top(b) -> float:
    """Height of the highest chassis-stage box / cylinder top (0 without one)."""
    top = 0.0
    for x in b.pkg.parts:
        if x.stage != "chassis" or x.shape == "mesh":
            continue
        h = x.size[2] if x.shape == "box" else x.height
        top = max(top, float(x.pose.xyz[2]) + float(h))
    return top


def add_vertical_stage(b, mast_height: float = 1.2, travel: float = 0.8) -> tuple[Part, Part]:
    """A mast on the chassis and a carriage plate on the lift stage; *Lift travel* = (0, travel);
    the robot mount on the plate's top at the lowest lift position."""
    if b.pkg.kind != "mobile":
        raise ValueError("a vertical stage is for a mobile robot package")
    if travel <= 0 or mast_height <= 0:
        raise ValueError("the mast height and the lift travel must be greater than 0")
    if not any(x.stage == "chassis" for x in b.pkg.parts):
        add_mobile_base(b)
    z0 = chassis_top(b)
    x0 = -0.25 * CHASSIS[0]  # towards the back, leaving the front free
    mast = Part(
        name="mast", stage="chassis", shape="box", size=(MAST[0], MAST[1], float(mast_height)),
        pose=Pose(xyz=(x0, 0.0, z0)),
    )  # fmt: skip
    b._add(mast)
    plate_z = z0 + max(0.0, float(mast_height) - float(travel) - PLATE[2])
    plate = Part(
        name="lift_plate", stage="lift", shape="box", size=PLATE,
        pose=Pose(xyz=(x0 + 0.5 * (MAST[0] + PLATE[0]), 0.0, plate_z)),
    )  # fmt: skip
    b._add(plate)
    mount = Pose(xyz=(plate.pose.xyz[0], 0.0, plate_z + PLATE[2]))
    b.pkg = b.pkg.model_copy(update={"lift": (0.0, float(travel)), "mount": mount})
    b.load_widgets()
    return b.pkg.parts[-2], b.pkg.parts[-1]


class MobilePresets:
    """The two buttons (shown for mobile robot packages) with the mast height and lift travel."""

    def __init__(self, gui, b):
        from ui.widgets import notify

        self.b = b
        self.base_btn = gui.add_button("Add mobile base", hint="a chassis box on the floor (resize it after)")
        self.mast = gui.add_number("Mast height (mm)", 1200.0, min=100.0, max=5000.0, step=50.0)
        self.travel = gui.add_number("Stage travel (mm)", 800.0, min=10.0, max=5000.0, step=50.0)
        self.stage_btn = gui.add_button(
            "Add vertical stage", hint="mast + lifting plate; the robot mount goes on the plate"
        )
        self.base_btn.on_click(lambda _e: add_mobile_base(b))

        def stage(e) -> None:
            try:
                add_vertical_stage(b, self.mast.value / 1000, self.travel.value / 1000)
            except ValueError as err:
                notify(e, "No vertical stage", str(err))

        self.stage_btn.on_click(stage)

    def refresh(self) -> None:
        on = self.b.pkg.kind == "mobile"
        for w in (self.base_btn, self.mast, self.travel, self.stage_btn):
            w.visible = on
