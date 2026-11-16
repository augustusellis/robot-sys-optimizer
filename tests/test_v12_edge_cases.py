"""v12 items 3-7: silent wrong output from docs/V11_REVIEW.md section 4.

Done when (3) a robot of unknown language gets no URScript guess; (4) a mesh with flipped face
normals measures the same surface radius, and disagreeing normals are reported, not read as flat;
(5) the edge check names groups with targets on both sides of a sheet; (6) a part over 5 m in
millimetres is an unsure unit guess; (7) a project opens with a missing file found by name in its
folder, keeps the left-out items when saved, and a located file restores them.
"""

import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import trimesh
import viser

from core import edge_check as ec
from core.mesh_units import MM_SURE_UP_TO, guess_units
from core.robot_program import default_format
from core.surface_radius import Fit, surface_radii

E = SimpleNamespace(client=None)


def test_unknown_robot_has_no_default_program_format():  # 3
    assert default_format("ur20", "ur") == "urscript"
    assert default_format("uploaded_arm", "matrix") is None


def _side_points(n=6, r=0.05):
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    N = np.c_[np.cos(a), np.sin(a), np.zeros(n)]
    return r * N, N


def test_flipped_normals_measure_the_same_radius():  # 4
    mesh = trimesh.creation.cylinder(radius=0.05, height=0.4, sections=128)
    P, N = _side_points()
    ok = surface_radii(mesh, P, N, 0.02)
    flipped = mesh.copy()
    flipped.invert()
    assert np.allclose(flipped.face_normals, -mesh.face_normals)
    bad = surface_radii(flipped, P, N, 0.02)
    for a, b in zip(ok, bad, strict=True):
        assert a.convex == pytest.approx(0.05, rel=0.1) and b.convex == pytest.approx(a.convex, rel=1e-3)
        assert not b.unsure


def test_disagreeing_normals_are_reported_not_flat():  # 4
    plane = trimesh.creation.box((0.4, 0.4, 0.01))
    plane = trimesh.Trimesh(*trimesh.remesh.subdivide_to_size(plane.vertices, plane.faces, 0.01))
    top = np.flatnonzero(plane.face_normals[:, 2] > 0.9)
    P = np.array([[0.0013, 0.0021, 0.005]])
    _, _, tri = trimesh.proximity.closest_point(plane, P)
    f = plane.faces.copy()
    f[tri] = f[tri][:, ::-1]  # only the face under the point is flipped
    mixed = trimesh.Trimesh(plane.vertices, f, process=False)
    assert tri[0] in top
    r = surface_radii(mixed, P, np.array([[0.0, 0, 1]]), 0.02)[0]
    assert r.unsure and np.isinf(r.concave)
    fit = Fit(unsure=[7])
    assert "face normals disagree" in fit.lines() and "7" in fit.lines()


def test_edge_check_names_two_sided_groups():  # 5
    xs = np.linspace(0, 0.2, 5)
    top = np.c_[xs, np.zeros(5), np.full(5, 0.01)]
    bot = np.c_[xs, np.zeros(5), np.zeros(5)]
    P = np.r_[top, bot]
    N = np.r_[np.tile([0, 0, 1.0], (5, 1)), np.tile([0, 0, -1.0], (5, 1))]
    ids = np.arange(10)
    assert ec.two_sided(ids, P, N, {i: "sheet" for i in range(10)}) == ["sheet"]
    groups = {i: ("top" if i < 5 else "bottom") for i in range(10)}
    assert ec.two_sided(ids, P, N, groups) == []  # one side per group: nothing to say


def test_long_mm_parts_are_an_unsure_guess():  # 6
    assert guess_units(4000.0) == ("mm", True)
    assert guess_units(MM_SURE_UP_TO + 1) == ("mm", False)


@pytest.fixture(scope="module")
def app():
    from app import build_app

    server = viser.ViserServer(host="127.0.0.1", port=8983, verbose=False)
    st = build_app(server)
    yield st
    st["builder_window"].stop()
    server.stop()


def _project_with_part(app, tmp_path):
    s, project = app["session"], app["project"]
    assert project.load_example("rail_long_part")
    dest = tmp_path / "study"
    project.save_now(dest)
    box = trimesh.creation.box((0.4, 0.3, 0.2))
    box.apply_translation((0, 0, 0.1))
    d = json.loads((dest / "project.json").read_text(encoding="utf-8"))
    d["part"]["mesh"] = "D:/old_pc/parts/my_part.stl"  # an absolute path from another PC
    (dest / "project.json").write_text(json.dumps(d), encoding="utf-8")
    return s, project, dest, box


def test_missing_file_found_by_name_in_the_project(app, tmp_path):  # 7
    s, project, dest, box = _project_with_part(app, tmp_path)
    (dest / "parts").mkdir()
    box.export(dest / "parts" / "my_part.stl")  # copied next to the project
    assert project._load(E, dest)
    assert s.part is not None and Path(s.part.mesh_path).as_posix() == "parts/my_part.stl"
    assert any("same name" in w for w in s.load_warnings) and not s.missing


def test_missing_part_is_kept_and_can_be_located(app, tmp_path):  # 7
    from ui import project_missing

    s, project, dest, box = _project_with_part(app, tmp_path)
    assert project._load(E, dest)
    assert s.part is None and [m["kind"] for m in s.missing] == ["part"]
    assert project_missing.items(s) == ["part mesh my_part.stl"]
    project.save_now()  # saving keeps the part (it was dropped for good before v12)
    d = json.loads((dest / "project.json").read_text(encoding="utf-8"))
    assert d["part"]["mesh"] == "D:/old_pc/parts/my_part.stl"
    data = trimesh.exchange.stl.export_stl(box)
    assert project_missing.locate(project, 0, data, E)
    assert s.part is not None and not s.missing
    assert Path(s.part.mesh_path).as_posix() == "assets/imported/parts/my_part.stl"
