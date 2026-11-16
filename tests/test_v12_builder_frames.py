"""v12 items 27-28: package builder feedback (2026-11-16).

Done when (27) a STEP file added to a tool package with *Load relative to* = *STEP coordinate
system* has that coordinate system on the robot flange (one in the file: used; several: the
dialog's choice; none: the flange with a note), *Robot flange* keeps the file origin there, and
the choice is shown for tool packages only; (28) every mm / deg field of the app and the builder
shows four decimals and keeps a typed 12.3456 mm unrounded.
"""

import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import viser
from viser._gui_api import UploadedFile

from core.transforms import pose_to_matrix
from ui import package_load
from ui.package_frames import frames
from ui.precision import DIGITS, wants

E = SimpleNamespace(client=None)


def _step(folder: Path, datums: str) -> Path:
    import cadquery as cq

    body = cq.Workplane("XY").box(60, 60, 20).faces(">Z").workplane().circle(10).extrude(120)
    path = folder / f"probe_{len(datums)}.step"
    cq.exporters.export(body, str(path))
    text = path.read_text()
    i = text.rindex("ENDSEC;")
    path.write_text(text[:i] + datums + text[i:])
    return path


@pytest.fixture(scope="module")
def steps(tmp_path_factory):
    pytest.importorskip("cadquery")
    from tests.step_fixture import DATUMS

    folder = tmp_path_factory.mktemp("step")
    one = DATUMS.split("#9005")[0]  # CSYS_TCP only
    return {"two": _step(folder, DATUMS), "one": _step(folder, one), "none": _step(folder, "")}


@pytest.fixture
def builder(tmp_path):
    from ui.package_builder import PackageBuilder

    class S:
        project_dir = tmp_path
        robot_name = "ur20"

    server = viser.ViserServer(host="127.0.0.1", port=8987, verbose=False)
    yield PackageBuilder(server, S())
    server.stop()


def _import(b, path, mode):
    b.load_from.value = mode
    b.upload._impl.value = UploadedFile(path.name, path.read_bytes())
    b._on_mesh(E)
    t_end = time.time() + 120
    while b.import_bar.visible and time.time() < t_end:
        time.sleep(0.1)
    return b.pkg.parts[-1]


def test_robot_flange_keeps_the_file_origin(builder, steps):
    b = builder
    assert b.pkg.kind == "tool" and b.load_from.visible and b.load_from.value == package_load.FLANGE
    part = _import(b, steps["two"], package_load.FLANGE)
    assert np.allclose(pose_to_matrix(part.pose), np.eye(4))
    assert "file origin on the robot flange" in b.import_info.content


def test_one_step_csys_goes_on_the_flange(builder, steps):
    b = builder
    _import(b, steps["one"], package_load.STEP_CSYS)
    T = frames(b.pkg)[f"{b.pkg.parts[-1].name} / CSYS_TCP"]
    assert np.allclose(T, np.eye(4), atol=1e-9)  # the CSYS is the flange now
    assert "CSYS_TCP" in b.import_info.content


def test_several_step_csys_ask_then_place(builder, steps):
    b = builder
    part = _import(b, steps["two"], package_load.STEP_CSYS)
    assert np.allclose(pose_to_matrix(part.pose), np.eye(4))  # no browser: the dialog is not shown
    assert "pick the mounting face" in b.import_info.content
    package_load.place(b, part.name, "CSYS_FLANGE")  # what the dialog's button does
    assert np.allclose(frames(b.pkg)[f"{part.name} / CSYS_FLANGE"], np.eye(4), atol=1e-9)
    # the flange datum pointed down (-z) 10 mm below the plate: the plate now hangs off the flange
    assert pose_to_matrix(b.pkg.parts[-1].pose)[2, 2] == pytest.approx(-1.0)


def test_no_step_csys_and_other_kinds(builder, steps):
    b = builder
    part = _import(b, steps["none"], package_load.STEP_CSYS)
    assert np.allclose(pose_to_matrix(part.pose), np.eye(4))
    assert "no named coordinate systems" in b.import_info.content
    b.pkg = b.pkg.model_copy(update={"kind": "part"})
    b.load_widgets()
    assert not b.load_from.visible  # parts and carriers keep the package frame


@pytest.fixture(scope="module")
def app():
    from app import build_app

    server = viser.ViserServer(host="127.0.0.1", port=8988, verbose=False)
    st = build_app(server)
    yield server, st
    st["builder_window"].stop()
    server.stop()


def _fields(server):
    return [h for h in server.gui._gui_input_handle_from_uuid.values() if hasattr(h, "precision")]


def test_every_mm_and_deg_field_has_four_decimals(app):  # 28
    server, st = app
    for srv in (server, st["builder_window"].server):
        hs = [h for h in _fields(srv) if wants(h.label)]
        assert len(hs) > 10
        low = [h.label for h in hs if h.precision < DIGITS]
        assert not low, low
    others = [h for h in _fields(server) if not wants(h.label) and h.label == "Triangles"]
    assert all(h.precision == 0 for h in others)  # counts stay whole numbers


def test_typed_decimals_are_kept(app):  # 28
    server, st = app
    b = st["builder_window"].builder
    b.tcp_pose.xyz.value = (12.3456, 0.0, 100.0)
    assert b.pkg.tcps[b.sel_tcp].pose.xyz[0] == pytest.approx(0.0123456, abs=1e-12)
    b.tx.value = (-1500.1234, 1500.5678)
    assert b.tx.value == (-1500.1234, 1500.5678)
    s, cell = st["session"], st["cell"]
    x, y, z = cell.part_pose.xyz.value
    cell.part_pose.xyz.value = (1000.1234, y, z)  # Robot & cell -> part pose
    assert s.part.T_world_part[0, 3] == pytest.approx(1.0001234, abs=1e-12)
