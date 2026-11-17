"""v13 #6: Optimize -> Several parts (batch): scan, tick, run from the panel."""

import pytest
import viser

from tests.test_v13_batch import make_parts


@pytest.fixture(scope="module")
def app():
    from app import build_app

    server = viser.ViserServer(host="127.0.0.1", port=8987, verbose=False)
    a = build_app(server)
    assert a["project"].load_example("rail_long_part")
    yield a
    a["builder_window"].stop()
    server.stop()


def test_scan_ticks_and_reasons(app, tmp_path):
    b = app["batch"]
    b.parts_folder.value = ""
    assert b.scan() == [] and "Scan a parts folder" in b.why_not()
    make_parts(tmp_path / "parts")
    (tmp_path / "parts" / "junk").mkdir()
    (tmp_path / "parts" / "junk" / "part.json").write_text("{}", encoding="utf-8")
    b.parts_folder.value = str(tmp_path / "parts")
    assert b.scan() == ["rail_a", "rail_b"]
    assert "junk" in b.bad_md.content
    assert b.out.value.endswith("batch_results")
    b.checks["rail_a"].value = b.checks["rail_b"].value = False
    assert b.why_not() == "Tick at least one part."
    b.checks["rail_b"].value = True
    assert [n for n, _ in b.ticked()] == ["rail_b"] and b.why_not() == ""


@pytest.mark.slow
def test_run_from_the_panel(app, tmp_path):
    b = app["batch"]
    make_parts(tmp_path / "parts")
    b.parts_folder.value = str(tmp_path / "parts")
    b.scan()
    b.checks["rail_b"].value = False
    b.out.value = str(tmp_path / "out")
    opened = []
    b.on_done = opened.append
    b.start(block=True)
    assert [r["status"] for r in b.rows] == ["done"], b.rows
    assert "✔ **rail_a**" in b.status.content and opened == [tmp_path / "out"]
    assert (tmp_path / "out" / "_setup" / "project.json").is_file()
    assert (tmp_path / "out" / "rail_a" / "project.json").is_file()
    assert not b.run.disabled and b.run.visible
