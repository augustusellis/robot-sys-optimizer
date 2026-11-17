"""v13 #5: batch runs (ui.batch_run / ui.batch_part / ui.batch_summary)."""

from pathlib import Path

import numpy as np
import pytest

from core import part_package as pp
from core import schema
from ui import batch_summary
from ui.batch_run import plan_workers, run_batch

REPO = Path(__file__).resolve().parents[1]
RAIL = REPO / "examples" / "rail_long_part"


def make_parts(folder: Path, broken: bool = False) -> list[tuple[str, Path]]:
    """Two packages of the rail example's part at two places (and one whose targets ask for a TCP
    the project's tool does not have)."""
    proj = schema.load_project(RAIL / "project.json")
    out = []
    for name, dx in (("rail_a", 0.0), ("rail_b", 0.4)):
        part = proj.part.model_copy(update={"pose": schema.Pose(xyz=(dx, 0.8, 0.5))})
        out.append((name, pp.save_part_package(name, part, proj.targets, lambda r: REPO / r, folder)))
    if broken:
        groups = [g.model_copy(update={"tcps": ["nope"]}) for g in proj.targets.groups]
        t = proj.targets.model_copy(update={"groups": groups})
        out.append(("bad_tcp", pp.save_part_package("bad_tcp", proj.part, t, lambda r: REPO / r, folder)))
    return out


def test_plan_workers():
    gb = 2**30
    assert plan_workers(1, 16, 40 * gb) == (1, "one part")
    assert plan_workers(10, 16, 40 * gb)[0] == 8  # half the cores
    assert plan_workers(3, 16, 40 * gb)[0] == 3  # no more than the parts
    assert plan_workers(10, 16, 4 * gb)[0] == 2  # the memory decides
    w, why = plan_workers(10, 2, 40 * gb)
    assert w == 1 and why.startswith("one part at a time")
    assert plan_workers(10, 16, 1 * gb)[0] == 1


def test_summary_files(tmp_path):
    rows = [
        {
            "part": "a",
            "status": "done",
            "reached": 3,
            "points": 3,
            "report": str(tmp_path / "a" / "report.pdf"),
        },
        {"part": "b<x>", "status": "failed", "reason": "no plan"},
    ]
    batch_summary.write(tmp_path, {"project": "p", "parts_folder": ""}, rows)
    data = batch_summary.read(tmp_path)
    assert [r["part"] for r in data["rows"]] == ["a", "b<x>"]
    page = (tmp_path / "summary.html").read_text(encoding="utf-8")
    assert "1 of 2 parts planned" in page and "b&lt;x&gt;" in page and 'href="a/report.pdf"' in page
    assert "Parts folder" not in page  # empty values are left out
    assert (tmp_path / "summary.csv").read_text(encoding="utf-8").startswith("part,status")
    with pytest.raises(ValueError):
        batch_summary.read(tmp_path / "nothing")


@pytest.mark.slow
def test_batch_parallel_serial_and_a_failing_part(tmp_path):
    parts = make_parts(tmp_path / "parts", broken=True)
    seen = []
    rows = run_batch(RAIL, parts, tmp_path / "out", workers=2, on_progress=lambda *m: seen.append(m))
    by = {r["part"]: r for r in rows}
    assert by["rail_a"]["status"] == by["rail_b"]["status"] == "done", rows
    assert by["bad_tcp"]["status"] == "failed" and "unknown TCPs" in by["bad_tcp"]["reason"]
    assert {(n, st) for n, st, _ in seen} >= {
        ("rail_a", "planning"),
        ("rail_a", "done"),
        ("bad_tcp", "failed"),
    }
    from ui.session import Session, load_project

    for name in ("rail_a", "rail_b"):
        r = by[name]
        assert r["reached"] == r["points"] == 25 and r["cycle_s"] and r["stops"] >= 1
        assert Path(r["report"]).is_file()
        s = Session()
        load_project(s, Path(r["folder"]))  # opens alone: the part mesh is in its assets
        assert s.results and s.result_points and not s.load_warnings
        assert "assets/imported" in s.part.mesh_path
    data = batch_summary.read(tmp_path / "out")
    assert [r["part"] for r in data["rows"]] == ["rail_a", "rail_b", "bad_tcp"]

    serial = run_batch(RAIL, parts[:1], tmp_path / "serial", workers=1)[0]
    assert serial["status"] == "done" and serial["cost"] == by["rail_a"]["cost"]
    a, b = Session(), Session()
    load_project(a, Path(by["rail_a"]["folder"]))
    load_project(b, Path(serial["folder"]))
    xa = [st.xyz for st in a.results[0].stations]
    xb = [st.xyz for st in b.results[0].stations]
    assert np.allclose(xa, xb)  # the same plan, run alone or beside another part


@pytest.mark.slow
def test_cancel_leaves_a_summary(tmp_path):
    parts = make_parts(tmp_path / "parts")
    rows = run_batch(RAIL, parts, tmp_path / "out", workers=1, cancelled=lambda: True)
    assert all(r["status"] in ("cancelled", "failed") for r in rows)
    assert len(batch_summary.read(tmp_path / "out")["rows"]) == 2
