"""v11 task 10: coordinate systems of STEP files as frames in the package builder.

Done when the STEP probe (two named datums) imported in the builder lists them; one assigned to the
TCP gives the TCP that pose; one assigned to a mobile package's robot mount moves the mount; and
the datums survive saving and opening the package.
"""

import time
from types import SimpleNamespace

import numpy as np
import pytest
import viser
from viser._gui_api import UploadedFile

from core.transforms import pose_to_matrix
from ui.package_frames import TARGETS, assign, frames

E = SimpleNamespace(client=None)


@pytest.fixture(scope="module")
def probe(tmp_path_factory):
    pytest.importorskip("cadquery")
    from tests.step_fixture import probe_step

    return probe_step(tmp_path_factory.mktemp("step"))


@pytest.fixture
def builder(tmp_path):
    from ui.package_builder import PackageBuilder

    class S:
        project_dir = tmp_path
        robot_name = "ur20"

    server = viser.ViserServer(host="127.0.0.1", port=8981, verbose=False)
    yield PackageBuilder(server, S())
    server.stop()


def _import(b, path):
    b.upload._impl.value = UploadedFile(path.name, path.read_bytes())
    b._on_mesh(E)
    t_end = time.time() + 120
    while b.import_bar.visible and time.time() < t_end:
        time.sleep(0.1)


def test_step_frames_listed_and_assigned(builder, probe, tmp_path):
    from core.urdf_package import write_package, zip_folder
    from core.urdf_package_read import open_package

    b = builder
    _import(b, probe)
    assert b.pkg.parts and "2 STEP frame(s)" in b.import_info.content
    names = set(frames(b.pkg))
    assert names == {"probe / CSYS_TCP", "probe / CSYS_FLANGE"}
    assert set(b.step_frames.frame.options) == names and b.step_frames.go.visible
    # the TCP gets the datum's pose (130 mm up the probe, z along +z)
    assign(b, "probe / CSYS_TCP", TARGETS[0])
    T = pose_to_matrix(b.pkg.tcps[0].pose)
    assert np.allclose(T[:3, 3], (0, 0, 0.13), atol=1e-6) and np.allclose(T[:3, 2], (0, 0, 1), atol=1e-6)
    # the flange datum (z down, 10 mm below the plate) as the package origin: the TCP is re-expressed
    assign(b, "probe / CSYS_FLANGE", TARGETS[3])
    T = pose_to_matrix(b.pkg.tcps[0].pose)
    assert np.allclose(T[:3, 3], (0, 0, -0.14), atol=1e-6)  # 140 mm from the flange datum, along its -z
    urdf = write_package(b.pkg, b.work / "meshes", tmp_path / "pkg" / "probe")
    back, _ = open_package(zip_folder(urdf.parent), tmp_path / "open")
    assert set(frames(back)) == names  # the datums are saved with the part


def test_robot_mount_and_vertical_stage_from_a_datum(builder, probe):
    from ui.package_presets import add_vertical_stage

    b = builder
    b._set_kind("mobile")
    _import(b, probe)  # pretend the STEP part is the chassis with its datums
    assign(b, "probe / CSYS_TCP", TARGETS[1])
    assert np.allclose(pose_to_matrix(b.pkg.mount)[:3, 3], (0, 0, 0.13), atol=1e-6)
    with pytest.raises(ValueError):
        assign(b, "probe / CSYS_TCP", TARGETS[2])  # no lift stage yet
    add_vertical_stage(b, 1.0, 0.5)
    plate = next(x for x in b.pkg.parts if x.stage == "lift")
    m0 = pose_to_matrix(b.pkg.mount)
    assign(b, "probe / CSYS_FLANGE", TARGETS[2])
    plate2 = next(x for x in b.pkg.parts if x.stage == "lift")
    assert np.allclose(pose_to_matrix(plate2.pose), frames(b.pkg)["probe / CSYS_FLANGE"], atol=1e-6)
    # the robot mount rode along with the stage
    shift = pose_to_matrix(plate2.pose) @ np.linalg.inv(pose_to_matrix(plate.pose))
    assert np.allclose(pose_to_matrix(b.pkg.mount), shift @ m0, atol=1e-6)
    with pytest.raises(ValueError):
        assign(b, "nope", TARGETS[0])
