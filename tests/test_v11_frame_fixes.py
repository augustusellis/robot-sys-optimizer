"""v11 tasks 1-3: Use this frame keeps the setup confirmed and moves the bridge with the frame; a
gantry whose bridge would travel through the part is refused; optimizers by external axis type."""

import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import viser

from core import golden_parts as gp
from core.carrier import Carrier
from core.schema import Base, Solution, Station
from core.transforms import make_T
from opt.frame_place import sweep_clear, with_frame
from ui.optimizer_select import available

REPO = Path(__file__).resolve().parents[1]
E = SimpleNamespace(client=None)


def _base(example: str) -> Base:
    return Base.model_validate(json.loads((REPO / f"examples/{example}/project.json").read_text())["base"])


def test_bridge_through_the_part_is_refused():
    wall = gp.tall_wall()  # 2 x 0.3 x 3 m standing on the floor
    floor = _base("floor_xy_rails")
    # the floor gantry beside the wall (as shipped): the bridge travels clear of it
    assert sweep_clear(Carrier(floor), wall, np.eye(4)) == ""
    # its rails straddling the wall: the bridge (0.1 m high, on the floor) would run into it
    astride = with_frame(floor, floor.pose.model_copy(update={"xyz": (0.0, 0.0, 0.0)}), floor.T_mount.xyz[2])
    assert sweep_clear(Carrier(astride), wall, np.eye(4)) == "the bridge would travel through the part"
    # an overhead gantry's bridge passes above the same wall
    assert sweep_clear(Carrier(_base("gantry_overhead")), wall, np.eye(4)) == ""
    assert sweep_clear(Carrier(_base("rail_long_part")), wall, make_T(p=(0, 0, 0))) == ""  # not a gantry


def test_optimizers_by_axis_type():
    assert "Gantry placement" in available("rail") and "Gantry placement" in available("gantry")
    assert "Gantry placement" not in available("mobile") and "Gantry placement" not in available(None)
    assert "Station planner" in available(None)


def test_use_this_frame_keeps_steps_confirmed_and_moves_the_bridge():
    from app import build_app
    from tests.helpers import confirm_all
    from ui.staleness import step_state

    server = viser.ViserServer(host="127.0.0.1", port=8965, verbose=False)
    try:
        app = build_app(server)
        s, frame, picker = app["session"], app["frame"], app["picker"]
        assert app["project"].load_example("floor_xy_rails")
        confirm_all(s)
        picker.sync(s)
        assert "Gantry placement" in picker.choice.options
        p = s.base.pose
        moved = with_frame(
            s.base,
            p.model_copy(update={"xyz": (p.xyz[0] + 1.0, p.xyz[1] - 0.3, p.xyz[2])}),
            s.base.T_mount.xyz[2],
        )
        sol = Solution(label="P", stations=[Station(xyz=(0, 0, 0), carrier={"x": 0.5, "y": 0.1})])
        frame.rows = [{"frame": "test frame", "base": moved, "plans": [(sol, {})], "stamps": {}}]
        assert frame.use_frame(E, 0)
        assert s.base.pose.xyz[0] == p.xyz[0] + 1.0
        assert all(step_state(s, k) == "confirmed" for k in s.steps)  # nothing to confirm again
        from ui.workflow import setup_reason

        assert not setup_reason(s)  # Plan stations stays usable
        # the bridge is on the moved frame at the plan's first stop (x 0.5 m along the moved rails)
        c = s.carrier()
        T = c.T_world_axis({"x": 0.5, "y": 0.1})
        assert np.allclose(s.T_world_base[:2, 3], (c.T_world_base({"x": 0.5, "y": 0.1}))[:2, 3])
        assert T[0, 3] == p.xyz[0] + 1.0 + 0.5
        # a workspace without a rail / gantry does not offer Gantry placement
        assert app["project"].load_example("mobile_amr")
        picker.sync(s)
        assert "Gantry placement" not in picker.choice.options
    finally:
        app["builder_window"].stop()
        server.stop()
