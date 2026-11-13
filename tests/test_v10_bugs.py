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
