"""v10 T11: what may touch the part, chosen in the package builder (ui/package_contacts.py).

Done when a sander built from STL parts with only its pad face picked reaches its points, and the
body still counts as a collision.
"""

from types import SimpleNamespace

import numpy as np
import pytest
import viser

from core.schema import Pose
from core.transforms import make_T, pose_to_matrix
from core.urdf_package import Frame, Package, Part, check, to_tool
from tests.helpers import confirm_all

E = SimpleNamespace(client=None)


def _pkg(**frame) -> Package:
    return Package(
        kind="tool",
        name="t",
        parts=[
            Part(name="body", stage="tool", shape="mesh", mesh="body.stl", link="sander_body"),
            Part(name="pad", stage="tool", shape="box", size=(0.1, 0.1, 0.02)),
        ],
        tcps=[Frame(name="tcp", **frame)],
    )


def test_contacts_go_to_the_tool_tcps_by_link():
    p = _pkg(contact_parts=["pad"], contact_faces={"body": [3, 1]})
    assert not check(p)
    t = to_tool(p, "t.urdf").tcps[0]
    assert t.contact_links == ["pad"] and t.contact_faces == {"sander_body": [1, 3]}
    bad = _pkg(contact_parts=["gone"], contact_faces={"pad": [0]})
    msgs = check(bad)
    assert any("'gone' does not exist" in m for m in msgs) and any("mesh parts ('pad')" in m for m in msgs)


def test_rename_and_remove_keep_contacts_consistent():
    from core import tool_contacts as tc

    f = [Frame(name="tcp", contact_parts=["pad"], contact_faces={"body": [1]})]
    r = tc.rename_part(f, "body", "shell")[0]
    assert r.contact_faces == {"shell": [1]} and r.contact_parts == ["pad"]
    d = tc.drop_part(f, "pad")[0]
    assert d.contact_parts == [] and d.contact_faces == {"body": [1]}
    assert tc.drop_part(f, "body", faces_only=True)[0].contact_faces == {}


def test_package_yaml_round_trip(tmp_path):
    from core.urdf_package import write_package, zip_folder
    from core.urdf_package_read import open_package

    p = Package(
        kind="tool",
        name="t",
        parts=[
            Part(name="body", stage="tool", shape="box", size=(0.1, 0.1, 0.1)),
            Part(
                name="pad",
                stage="tool",
                shape="cylinder",
                radius=0.05,
                height=0.02,
                pose=Pose(xyz=(0, 0, 0.1)),
            ),
        ],
        tcps=[Frame(name="tcp", pose=Pose(xyz=(0, 0, 0.12)), contact_parts=["pad"])],
    )
    write_package(p, None, tmp_path / "t")
    back, _ = open_package(zip_folder(tmp_path / "t"), tmp_path / "back")
    assert back.tcps[0].contact_parts == ["pad"]


@pytest.fixture(scope="module")
def app():
    from app import build_app

    server = viser.ViserServer(host="127.0.0.1", port=8933, verbose=False)
    st = build_app(server)
    yield st
    st["builder_window"].stop()
    server.stop()


@pytest.mark.slow
def test_sander_with_only_its_pad_face_picked(app):
    from core.carrier import primitive_mesh
    from core.collision import CollisionScene
    from core.export import station_pose
    from core.ik import make_ik
    from ui import package_edit

    s, b, opt = app["session"], app["builder"], app["optimize"]
    assert app["project"].load_example("rail_long_part")
    confirm_all(s)
    app["workflow"].refresh(force=True)
    app["stations"].start(block=True, quick=True)
    sol = app["results"].selected()
    assert sol is not None and not sol.unreached
    # the sample sander is two STL parts; editing it brings its contact (the whole pad) back
    assert package_edit.edit_tool(b, E)
    pad = next(x for x in b.pkg.parts if x.link == "pad_link")
    assert all(x.shape == "mesh" for x in b.pkg.parts) and b.pkg.tcps[0].contact_parts == [pad.name]
    b.sel = b.pkg.parts.index(pad)
    b.load_widgets()
    assert b.contacts.whole.value and b.contacts.colour(pad, (0, 0, 0)) != (0, 0, 0)
    b.contacts.set_whole(False)  # only the pad's working face may touch
    b.contacts.mode.value = "Click + (surface)"
    mesh = primitive_mesh(b.primitive(pad))
    T_tcp = b.root_pose() @ pose_to_matrix(b.pkg.tcps[0].pose)
    z = T_tcp[:3, 2]  # out of the tool: the click comes from below the pad, looking up into it
    assert b.contacts.click(mesh, T_tcp[:3, 3] + 0.05 * z, -z) > 0
    t = b.pkg.tcps[0]
    ids = t.contact_faces[pad.name]
    T_part = pose_to_matrix(pad.pose)  # the TCP's z in the pad's frame: the working face's normal
    z_part = T_part[:3, :3].T @ pose_to_matrix(b.pkg.tcps[0].pose)[:3, 2]
    assert t.contact_parts == [] and np.allclose(mesh.face_normals[ids] @ z_part, 1.0, atol=1e-6)
    assert len(ids) < len(mesh.faces) / 2  # the face, not the whole pad
    assert "faces of" in b.contacts.info.content
    assert any(h.name.endswith("_contact") for h in b._handles if hasattr(h, "name"))
    assert b._use(E) and not b.contact_lost
    tcp = s.tool.tcps[0]
    assert tcp.contact_links == [] and tcp.contact_faces == {"pad_link": ids}
    # the same faces on the link's mesh in the project's tool
    hl = s.tool_model.link("pad_link")
    assert len(hl.mesh.faces) == len(mesh.faces)
    centres = mesh.triangles_center[ids] @ T_part[:3, :3].T + T_part[:3, 3]  # links sit on the mount
    assert np.allclose(
        hl.T_head_link @ np.c_[hl.mesh.triangles_center[ids], np.ones(len(ids))].T,
        np.c_[centres, np.ones(len(ids))].T,
        atol=1e-6,
    )
    opt.start(block=True)  # the check from the plan's stops
    res = s.feasibility
    assert all(r.status != "unreachable" for r in res.values()), opt.summary.content
    # the body (and the rest of the pad) still collide: push the tool 30 mm into the part
    pid, r = next(iter(res.items()))
    k = sol.assignment[pid]
    T_base = station_pose(s.base, sol.stations[k], s.T_world_base)
    scene = CollisionScene(s.robot, s.tool_model, s.part_mesh(), T_base, s.part.T_world_part)
    assert not scene.collides(r.q, r.tcp)
    T = s.robot.fk(r.q) @ make_T(p=(0, 0, 0.03))
    sols = make_ik(s.robot).solve(T)
    q = sols[np.argmin(np.abs(sols - r.q).sum(axis=1))]
    assert scene.collides(q, r.tcp)
    hits = scene.colliding_pairs(q, r.tcp)
    assert any("sander_body" in str(h) or "pad_link" in str(h) for h in hits), hits
