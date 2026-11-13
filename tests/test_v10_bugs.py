"""V10 part A (docs/V10_PLAN.md tasks 1-6): bugs found in the v9 review."""

from types import SimpleNamespace

import pytest
import viser

from core.schema import TCP, Pose
from core.tool_contacts import carry_contacts
from tests.helpers import confirm_all

E = SimpleNamespace(client=None)


@pytest.fixture(scope="module")
def app():
    from app import build_app

    server = viser.ViserServer(host="127.0.0.1", port=8851, verbose=False)
    st = build_app(server)
    yield st
    st["builder_window"].stop()
    server.stop()


# --- task 1: edited tools keep their contact pad ------------------------------------------------


def test_carry_contacts_reports_what_is_lost():
    t = TCP(name="pad", contact_links=["pad_link", "gone"], contact_faces={"body": [1, 2], "pad_link": [3]})
    out, lost = carry_contacts([t], {"body": 10, "pad_link": 8}, {"pad_link": 8, "body": 12})
    assert out[0].contact_links == ["pad_link"] and out[0].contact_faces == {"pad_link": [3]}
    assert len(lost) == 2 and any("gone" in m for m in lost) and any("body" in m for m in lost)


def test_tool_package_writes_one_link_per_part(tmp_path):
    from core.tool import ToolModel
    from core.urdf_package import Collision, Frame, Package, Part, read_urdf, to_tool, write_package

    p = Package(
        kind="tool",
        name="t",
        parts=[
            Part(
                name="body",
                stage="tool",
                shape="cylinder",
                radius=0.04,
                height=0.15,
                pose=Pose(xyz=(0, 0, 0.1)),
            ),
            Part(name="pad", stage="tool", shape="box", size=(0.1, 0.1, 0.02), link="pad_link"),
        ],
        tcps=[Frame(name="tcp")],
        collision=Collision(pad_mm=1.0),
    )
    urdf = write_package(p, None, tmp_path / "t")
    links = {hl.name: hl for hl in ToolModel(to_tool(p, str(urdf)), urdf).links}
    assert set(links) == {"body", "pad_link"}
    # the collision copy sits where the visual does (yourdfpy node-name clash, found in v10)
    assert abs(links["body"].collision.bounds[1, 2] - 0.251) < 1e-6
    back = read_urdf(urdf, "tool")
    assert {x.link for x in back.parts} == {"body", "pad_link"}


@pytest.mark.slow
def test_edited_sander_keeps_its_pad(app):
    """Done when: the sander edited and used through the builder still reaches its points with the
    pad as its contact geometry."""
    from ui import package_edit

    s, b, opt = app["session"], app["builder"], app["optimize"]
    assert app["project"].load_example("rail_long_part")
    assert s.tool.tcps[0].contact_links == ["pad_link"]
    confirm_all(s)
    app["workflow"].refresh(force=True)
    app["stations"].start(block=True, quick=True)
    sol = app["results"].selected()
    assert sol is not None and not sol.unreached
    assert package_edit.edit_tool(b, E)
    assert {x.link for x in b.pkg.parts} == {"sander_body", "pad_link"}
    assert b._use(E) and not b.contact_lost
    ee = s.tool
    assert "packages" in ee.urdf and ee.tcps[0].contact_links == ["pad_link"]
    assert {hl.name for hl in s.tool_model.links} == {"sander_body", "pad_link"}
    opt.start(block=True)  # the check from the plan's stops
    assert all(r.status != "unreachable" for r in s.feasibility.values()), opt.summary.content
    # the pad really is what touches: without it as a contact the points collide
    s.set_tool(ee.model_copy(update={"tcps": [ee.tcps[0].model_copy(update={"contact_links": []})]}))
    opt.start(block=True)
    assert any(r.reason == "collision" for r in s.feasibility.values())


# --- task 2: saving no longer clears plans ------------------------------------------------------


def _example_copy(tmp_path, name="rail_long_part"):
    import shutil
    from pathlib import Path

    from ui.session import Session, load_project

    src = Path(__file__).resolve().parents[1] / "examples" / name
    shutil.copytree(src, tmp_path / name)
    s = Session()
    load_project(s, tmp_path / name)
    return s


def test_save_that_bundles_the_part_keeps_plans(tmp_path):
    """Done when: a save that bundles an outside part keeps the plans and feasibility results."""
    import shutil

    from ui import staleness
    from ui.session import save_project

    s = _example_copy(tmp_path)
    outside = tmp_path / "outside"
    outside.mkdir()
    shutil.copy(s.resolve(s.part.mesh_path), outside / "my_part.stl")
    s.part.mesh_path = (outside / "my_part.stl").as_posix()
    from core.schema import Solution, Station

    s.results = [Solution(label="P", stations=[Station(xyz=(0, 0, 0))])]
    s.stamps = {"P": staleness.stamp_for(s, "plan"), "feasibility": staleness.stamp_for(s, "feasibility")}
    fresh: set[str] = set()
    assert not staleness.drop_stale_results(s, fresh) and fresh == {"P"}
    save_project(s)
    assert s.part.mesh_path.startswith("assets/imported/") and s.bundled
    assert staleness.solution_changes(s, s.results[0]) == []
    assert staleness.feasibility_changes(s) == []
    assert not staleness.drop_stale_results(s, fresh) and [r.label for r in s.results] == ["P"]
    # a real change of the file still counts
    m = s.part_mesh().copy()
    m.apply_translation((0.0, 0.0, 0.001))
    m.export(s.resolve(s.part.mesh_path))
    assert staleness.solution_changes(s, s.results[0]) == ["part"]


def test_v9_stamps_migrate(tmp_path):
    from ui import staleness

    s = _example_copy(tmp_path)
    old = staleness.components(s, legacy=True)
    s.stamps = {"P": dict(old)}
    s.steps = {"Cell": {k: old[k] for k in staleness.STEP_COMPONENTS["Cell"]}}
    assert staleness.changed(s.stamps["P"], staleness.components(s))  # paths vs contents
    assert staleness.migrate(s) > 0
    assert staleness.changed(s.stamps["P"], staleness.components(s)) == []
    assert staleness.step_state(s, "Cell") == "confirmed"


# --- task 3: no session swap while viewing a compared robot -------------------------------------


def test_snapshot_never_touches_the_session(tmp_path):
    """Done when: snapshot leaves every session field untouched (a concurrent reader)."""
    import threading

    from core.schema import Solution, Station

    s = _example_copy(tmp_path)
    own = [Solution(label="own plan", stations=[Station(xyz=(0, 0, 0))])]
    viewed = [Solution(label="UR10e plan", stations=[Station(xyz=(1, 0, 0))])]
    s.view_stash = {
        "robot": s.robot,
        "robot_name": "ur20",
        "robot_urdf": None,
        "q": s.q,
        "home_deg": None,
        "results": own,
        "result_points": {},
        "stamps": {},
    }
    s.robot_name, s.results = "ur10e", viewed  # what the compare view shows
    before = dict(s.__dict__)
    seen, stop = set(), threading.Event()

    def reader() -> None:
        while not stop.is_set():
            seen.add((s.robot_name, s.results[0].label))

    th = threading.Thread(target=reader)
    th.start()
    try:
        for _ in range(30):
            p = s.snapshot()
            assert p.robot.name == "ur20" and [r.label for r in p.results] == ["own plan"]
    finally:
        stop.set()
        th.join()
    assert seen == {("ur10e", "UR10e plan")}
    assert all(s.__dict__[k] is v for k, v in before.items())
