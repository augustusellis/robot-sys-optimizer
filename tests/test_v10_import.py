"""v10 T20: Cancel for STEP import, mesh units guessed from the size (asked when unsure)."""

import time
from pathlib import Path
from types import SimpleNamespace

import pytest
import trimesh
import viser
from viser._gui_api import UploadedFile

from core.mesh_units import ImportCancelledError, guess_units, step_in_process

E = SimpleNamespace(client=None)


def test_guess_units():
    assert guess_units(350.0) == ("mm", True)  # a 350 mm part
    assert guess_units(1.2) == ("m", True)  # a 1.2 m part
    assert guess_units(12.0)[1] is False  # 12 m or 12 mm: ask
    assert guess_units(0.001)[1] is False


@pytest.fixture(scope="module")
def step_file(tmp_path_factory):
    pytest.importorskip("cadquery")
    from tests.step_fixture import probe_step

    d = tmp_path_factory.mktemp("step")
    return probe_step(d)


def test_step_meshed_in_a_process_and_cancelled(step_file, tmp_path):
    out = tmp_path / "probe.stl"
    step_in_process(Path(step_file), out, lambda: False)
    m = trimesh.load(str(out), force="mesh")
    assert len(m.faces) > 10 and max(m.extents) < 1.0  # metres
    t0 = time.monotonic()
    with pytest.raises(ImportCancelledError):
        step_in_process(Path(step_file), tmp_path / "again.stl", lambda: time.monotonic() - t0 > 0.5)
    assert time.monotonic() - t0 < 10 and not (tmp_path / "again.stl").exists()
    bad = tmp_path / "bad.step"
    bad.write_bytes(b"ISO-10303-21; not a STEP file")
    with pytest.raises(RuntimeError):
        step_in_process(bad, tmp_path / "bad.stl", lambda: False)


def test_builder_guesses_units_and_asks_when_unsure(tmp_path):
    from ui.package_actions import set_units
    from ui.package_builder import AUTO, PackageBuilder

    class S:
        project_dir = tmp_path
        robot_name = "ur20"

    server = viser.ViserServer(host="127.0.0.1", port=8957, verbose=False)
    try:
        b = PackageBuilder(server, S())
        assert b.units.value == AUTO

        def upload(name: str, extents) -> None:
            b.upload._impl.value = UploadedFile(
                name, trimesh.exchange.stl.export_stl(trimesh.creation.box(extents))
            )
            b._on_mesh(E)
            t_end = time.time() + 60
            while b.import_bar.visible and time.time() < t_end:
                time.sleep(0.05)

        upload("plate.stl", (300.0, 200.0, 10.0))
        assert (
            b.pkg.parts[-1].units == "mm"
            and "units **mm** from its size (300 mm long" in b.import_info.content
        )
        upload("beam.stl", (1.5, 0.1, 0.1))
        assert b.pkg.parts[-1].units == "m"
        upload("odd.stl", (12.0, 3.0, 3.0))  # 12 m or 12 mm?
        assert "units **unsure**" in b.import_info.content
        set_units(b, "odd", "mm")  # what the dialog's button does
        assert b.pkg.parts[-1].units == "mm" and b.part_units.value == "mm" and b.part_units.visible
        b.units.value = "in"  # an explicit choice is used as it is
        upload("bracket.stl", (4.0, 2.0, 1.0))
        assert b.pkg.parts[-1].units == "in"
    finally:
        server.stop()
