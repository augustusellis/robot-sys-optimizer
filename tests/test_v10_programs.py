"""v10 T21: robot programs (URScript, FANUC TP, ABB RAPID) from the built paths."""

import re

import numpy as np
import pytest
import viser
from scipy.spatial.transform import Rotation

from core.robot_program import default_format, program


def test_default_format_by_robot():
    assert default_format("ur20", "ur") == "urscript"
    assert default_format("crx10ia", "fanuc") == "fanuc"
    assert default_format("gofa10", "matrix") == "rapid"


def _ur_T(s: str) -> np.ndarray:
    v = [float(x) for x in s.split(",")]
    T = np.eye(4)
    T[:3, :3] = Rotation.from_rotvec(v[3:]).as_matrix()
    T[:3, 3] = v[:3]
    return T


@pytest.fixture(scope="module")
def built():
    from app import build_app
    from tests.helpers import confirm_all

    server = viser.ViserServer(host="127.0.0.1", port=8959, verbose=False)
    app = build_app(server)
    s = app["session"]
    assert app["project"].load_example("rail_long_part")
    confirm_all(s)
    app["workflow"].refresh(force=True)
    app["stations"].start(block=True, quick=True)
    app["results"].refresh()
    app["sim"].build(block=True)
    yield app
    server.stop()


@pytest.mark.slow
def test_three_formats_from_the_rail_paths(built):
    s, sim = built["session"], built["sim"]
    plan = sim.plan
    tcp = plan.samples[0].tcp
    T_ft = s.tool_model.T_flange_tcp(tcp)
    car = s.carrier()
    n_lin = sum(1 for p in plan.samples if p.kind not in ("carrier", "home") and p.q is not None)
    n_stops = len(plan.stations)
    ur = program("urscript", plan, T_ft, s.part.T_world_part, car, s.cycle)
    assert ur.count("movel(") == n_lin and "set_tcp(p[" in ur
    assert all(f"uf_{k + 1} = p[" in ur for k in range(n_stops)) and ur.count("rso_carrier([") == n_stops - 1
    # a surface target, composed as the controller does, is the TCP of the planned joints
    m = re.search(r"movel\(pose_trans\(uf_(\d+), p\[([^\]]+)\]\)", ur)
    k = int(m.group(1)) - 1
    uf = _ur_T(re.search(rf"uf_{k + 1} = p\[([^\]]+)\]", ur).group(1))
    T_base_tcp = uf @ _ur_T(m.group(2))
    first = next(p for p in plan.stations[k].samples if p.kind not in ("carrier", "home") and p.q is not None)
    assert np.allclose(T_base_tcp, s.robot.fk(first.q) @ T_ft, atol=1e-5)
    fanuc = program("fanuc", plan, T_ft, s.part.T_world_part, car, s.cycle)
    assert fanuc.startswith("/PROG  RSO_PATH") and "/POS" in fanuc and fanuc.rstrip().endswith("/END")
    assert fanuc.count("CONFIG :") == n_lin and all(f"UFRAME_NUM={k + 1}" in fanuc for k in range(n_stops))
    assert "E1=" not in fanuc  # stop and go: no extended axis
    rapid = program("rapid", plan, T_ft, s.part.T_world_part, car, s.cycle)
    assert rapid.count("MoveL p") == n_lin and rapid.count("CONST robtarget") == n_lin
    assert all(f"rso_wobj{k + 1} :=" in rapid for k in range(n_stops)) and "ConfL \\Off;" in rapid
    # the Export button writes the robot's own language (UR20: URScript)
    path, data = built["results"].export.export_program(built["results"].selected())
    assert path.name == "rso_path.script" and data.decode().count("movel(") == n_lin


@pytest.mark.slow
def test_coordinated_program_has_the_carrier_as_extended_axis(built):
    from ui.sim_fields import MOTION

    s, sim = built["session"], built["sim"]
    sim.motion_mode.value = MOTION[1]
    try:
        sim.build(block=True)
        plan = sim.plan
        T_ft = s.tool_model.T_flange_tcp(plan.samples[0].tcp)
        fanuc = program("fanuc", plan, T_ft, s.part.T_world_part, s.carrier(), s.cycle)
        e1 = [float(v) for v in re.findall(r"E1= (-?[\d.]+)  mm", fanuc)]
        assert e1 and max(e1) - min(e1) > 1000  # the rail moves more than a metre along the passes
        rapid = program("rapid", plan, T_ft, s.part.T_world_part, s.carrier(), s.cycle)
        assert "rso_wobj2" not in rapid and re.search(r"\],\[-?[\d.]+,9E9", rapid)
        ur = program("urscript", plan, T_ft, s.part.T_world_part, s.carrier(), s.cycle)
        assert "no external axes" in ur and ur.count("rso_carrier([") >= ur.count("movel(")
    finally:
        sim.motion_mode.value = MOTION[0]
