"""Batch: PDF report per job (BatchManifest.pdf, --pdf) and the time-left estimate."""

import json
from pathlib import Path

import pytest

from core import batch as bt
from core.schema import Solution, Station
from tests.test_batch_job import _make_batch
from ui import batch_job as bj


def test_eta_text():
    assert bt.eta_text([], 3) == ""  # nothing finished yet: no guess
    assert bt.eta_text([60.0], 0) == ""
    assert bt.eta_text([0.0, 0.0], 2) == ""  # jobs without a run time do not count
    assert bt.eta_text([20.0], 2) == "about 1 min left"
    assert bt.eta_text([60.0, 600.0, 120.0], 4) == "about 8 min left"  # the median (120 s), not the mean
    assert bt.eta_text([100.0, 200.0], 2) == "about 5 min left"
    assert bt.eta_text([3600.0], 3) == "about 3.0 h left"


def test_manifest_pdf_flag_and_old_files(tmp_path):
    path = tmp_path / bt.MANIFEST_NAME
    path.write_text(json.dumps({"version": 1, "cell_project": "../cell", "jobs": []}), encoding="utf-8")
    assert bt.load_manifest(path).pdf is False  # a version 1 file loads
    bt.set_pdf(path, True)
    m = bt.load_manifest(path)
    assert m.pdf is True and m.cell_project == "../cell"
    stamp = path.stat().st_mtime_ns
    bt.set_pdf(path, True)  # unchanged: not rewritten
    assert path.stat().st_mtime_ns == stamp
    data = json.loads(path.read_text(encoding="utf-8"))
    data["version"] = bt.VERSION + 1
    path.write_text(json.dumps(data), encoding="utf-8")
    with pytest.raises(ValueError, match="newer version"):
        bt.load_manifest(path)


@pytest.fixture
def fake(monkeypatch):
    """A planner that returns one plan at once and an export_all that records its pdf flag."""
    from ui import plan_job, report_make

    seen = {"pdf": []}

    def station_job(payload, emit, cancelled):
        emit("progress", ("planning", 0.5))
        sol = Solution(label="1 station", stations=[Station(xyz=(0, 0, 0))], cost=1.0)
        emit("solution", (sol, {}))
        return {"best": sol, "rules_status": None}

    def export_all(session, sol, out_dir, pdf=False, **kw):
        seen["pdf"].append(pdf)
        out_dir.mkdir(parents=True, exist_ok=True)
        (out_dir / "report.html").write_text("<html></html>", encoding="utf-8")
        out = {"report.html": out_dir / "report.html"}
        if pdf:
            out["report.pdf"] = "PdfError: no browser found (Edge or Chrome)"
        return out

    monkeypatch.setattr(plan_job, "station_job", station_job)
    monkeypatch.setattr(report_make, "export_all", export_all)
    return seen


def test_pdf_flag_reaches_the_export_and_a_failure_is_reported(tmp_path, fake):
    m, bdir = _make_batch(tmp_path, n=1)
    assert bj.drive(bdir, lambda *a: None, in_process=True)["status"] == "done"
    assert fake["pdf"] == [False]
    job = bt.load_manifest(bdir / bt.MANIFEST_NAME).jobs[0]
    job.status = "pending"
    bt.update_job(bdir / bt.MANIFEST_NAME, job)
    bt.set_pdf(bdir / bt.MANIFEST_NAME, True)
    assert bj.drive(bdir, lambda *a: None, in_process=True)["status"] == "done"
    assert fake["pdf"] == [False, True]
    job = bt.load_manifest(bdir / bt.MANIFEST_NAME).jobs[0]
    assert job.status == "done"  # the plan is done; the missing PDF is said in the error column
    assert job.error.startswith("export failed: report.pdf (PdfError: no browser found")
    assert job.summary["exports"]["report.pdf"].startswith("error:")


def test_progress_carries_the_time_left_after_the_first_job(tmp_path, fake, monkeypatch):
    _m, bdir = _make_batch(tmp_path, n=3)
    ticks = iter(range(0, 100000, 150))  # every clock read is 150 s later: jobs "take" minutes
    monkeypatch.setattr(bj.time, "monotonic", lambda: float(next(ticks)))
    prog = []
    bj.drive(bdir, lambda k, d: prog.append(d[0]) if k == "progress" else None, in_process=True)
    first = [p for p in prog if p.startswith("job 1/3")]
    later = [p for p in prog if p.startswith("job 2/3") or p.startswith("job 3/3")]
    assert first and not any("left" in p for p in first)  # nothing to go by yet
    assert later and all(" - about " in p and p.endswith(" left") for p in later)


def test_cli_pdf_flag_sets_the_manifest(tmp_path, fake, capsys):
    _m, bdir = _make_batch(tmp_path, n=1)
    assert bj.main([str(bdir), "--in-process", "--pdf"]) == 0
    assert bt.load_manifest(bdir / bt.MANIFEST_NAME).pdf is True and fake["pdf"] == [True]
    assert "1/1 job(s) done" in capsys.readouterr().out
    assert bj.main([str(Path(tmp_path) / "nowhere"), "--pdf"]) == 1
    assert "batch failed" in capsys.readouterr().out


def test_queue_table_still_importable_from_the_panel_module():
    from ui.batch_queue_ui import job_table

    assert job_table(bt.BatchManifest()) == "_The queue is empty._" and job_table is bt.job_table
