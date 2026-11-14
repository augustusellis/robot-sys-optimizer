"""Batch runner (ui.batch_job, plan Part 2 step B4): per-job isolation, cancel / resume,
the manifest on disk after every step, planner settings snapshot, fresh worker per job."""

import subprocess
import sys
import threading
from pathlib import Path

import pytest

from core import batch as bt
from core.golden_parts import flat_plate
from core.schema import Solution, Station
from core.target_gen import GenSettings
from opt.placement import PlannerSettings
from ui import batch_job as bj
from ui.session import empty_cell, save_project

ROOT = Path(__file__).resolve().parents[1]


def _planner(**kw) -> dict:
    return {"version": 1, "mode": "cost", "settings": bj.settings_dict(PlannerSettings(**kw)), "workers": 1}


def _make_batch(tmp_path, n=3, cell=None):
    cell_dir = tmp_path / "cell"
    save_project(cell or empty_cell(), cell_dir)
    bdir = tmp_path / "batch"
    (bdir / "parts").mkdir(parents=True)
    mesh = flat_plate()
    mesh.export(bdir / "parts" / "plate.stl")
    m = bt.BatchManifest(cell_project=bt.rel_cell(cell_dir, bdir))
    top = [i for i, nrm in enumerate(mesh.face_normals) if nrm[2] > 0.9]
    sha = bt.mesh_sha1(bdir / "parts" / "plate.stl")
    for _ in range(n):
        bt.add_job(
            m,
            part="parts/plate.stl",
            selection=top,
            gen=GenSettings(row_m=0.2, col_m=0.2),
            mesh_sha1=sha,
            planner=_planner(),
        )
    bt.save_manifest(m, bdir / bt.MANIFEST_NAME)
    return m, bdir


def _disk(bdir) -> dict:
    return {j.id: j.status for j in bt.load_manifest(bdir / bt.MANIFEST_NAME).jobs}


@pytest.fixture
def fake(monkeypatch):
    """station_job streams two plans (1 and 2 stops); export_all writes a stub report. Records
    the job's on-disk status while it plans."""
    from ui import plan_job, report_make

    state = {"cancel_after_first": None, "seen": [], "bdir": None}

    def station_job(payload, emit, cancelled):
        s = payload["session"]
        assert payload["spec"]["settings"].max_stations == 1  # static base: one station
        job_id = Path(s.project_dir).name
        state["seen"].append((job_id, _disk(state["bdir"])[job_id]))
        emit("progress", ("planning", 0.5))
        emit("warn", "check pool fell back to serial checks")
        a = Solution(label="1 station", stations=[Station(xyz=(0, 0, 0))], cost=5.0, unreached=[1])
        emit("solution", (a, {}))
        if state["cancel_after_first"] is not None:
            state["cancel_after_first"].set()
        if cancelled():
            return {"best": a, "rules_status": None}
        b = Solution(label="2 stations", stations=[Station(xyz=(0, 0, 0))] * 2, cost=3.0)
        emit("solution", (b, {}))
        return {"best": b, "rules_status": None}

    def export_all(session, sol, out_dir, pdf=False, **kw):
        out_dir.mkdir(parents=True, exist_ok=True)
        (out_dir / "report.html").write_text(f"<html>{sol.label}</html>", encoding="utf-8")
        return {"report.html": out_dir / "report.html", "tcp_poses.csv": "ValueError: no tool loaded"}

    monkeypatch.setattr(plan_job, "station_job", station_job)
    monkeypatch.setattr(report_make, "export_all", export_all)
    return state


def test_isolation_resume_and_summary(tmp_path, fake):
    from ui.session import Session, load_project

    m, bdir = _make_batch(tmp_path)
    fake["bdir"] = bdir
    j2 = m.jobs[1]
    j2.mesh_sha1 = "0" * 40  # corrupt: the mesh "changed" since it was queued
    bt.save_manifest(m, bdir / bt.MANIFEST_NAME)
    msgs, after_each = [], []

    def on_msg(kind, data):
        msgs.append((kind, data))
        if kind == "job":
            after_each.append(_disk(bdir))

    out = bj.drive(bdir, on_msg, in_process=True)
    assert out["status"] == "done"
    ids = [j.id for j in m.jobs]
    assert after_each == [
        {ids[0]: "done", ids[1]: "pending", ids[2]: "pending"},
        {ids[0]: "done", ids[1]: "failed", ids[2]: "pending"},
        {ids[0]: "done", ids[1]: "failed", ids[2]: "done"},
    ]
    assert fake["seen"] == [(ids[0], "running"), (ids[2], "running")]  # running on disk while planning
    disk = bt.load_manifest(bdir / bt.MANIFEST_NAME)
    d1, d2, d3 = disk.jobs
    assert "changed since it was queued" in d2.error and "\n" not in d2.error
    assert (bdir / ids[1] / "error.txt").read_text(encoding="utf-8").startswith(d2.error)
    assert d1.summary["stations"] == 2 and d1.summary["cost"] == 3.0 and d1.summary["plans"] == 2
    assert d1.summary["report"] == f"{ids[0]}/export/report.html" and (bdir / d1.summary["report"]).is_file()
    assert d1.error.startswith("export failed: tcp_poses.csv") and d1.status == "done"
    assert d1.summary["reached"] == d1.summary["total"] > 0 and d1.run_s >= 0
    assert d1.auto_confirmed and d1.out_dir == ids[0]
    warns = [d for k, d in msgs if k == "warn"]
    assert warns and warns[0].startswith(f"{ids[0]}: check pool")
    prog = [d for k, d in msgs if k == "progress"]
    assert prog[0][0].startswith("job 1/3") and all(0 <= f <= 1 for _, f in prog)
    s = Session()
    load_project(s, bdir / ids[0])  # the job's study: both plans, best first, stamped
    assert [r.label for r in s.results] == ["2 stations", "1 station"] and s.results[0].run_s is not None
    assert set(s.stamps) >= {r.id for r in s.results} and all(r.id for r in s.results)  # v12 #8: ids
    csv = (bdir / "summary.csv").read_text(encoding="utf-8").splitlines()
    assert csv[0].startswith("job,name,part,status") and len(csv) == 4 and ",failed," in csv[2]
    assert "report.html" in (bdir / "summary.html").read_text(encoding="utf-8")
    # resume: done and failed jobs are skipped; a reset job runs again
    fake["seen"].clear()
    assert bj.drive(bdir, lambda *a: None, in_process=True)["jobs"] == []
    d2.status, d2.mesh_sha1 = "pending", m.jobs[0].mesh_sha1
    bt.update_job(bdir / bt.MANIFEST_NAME, d2)
    rows = bj.batch_job({"batch_dir": str(bdir)}, lambda *a: None, lambda: False)  # one-process entry
    assert [r["id"] for r in rows] == [ids[1]] and rows[0]["status"] == "done"
    assert fake["seen"] == [(ids[1], "running")]


def test_cancel_keeps_plans_and_stops(tmp_path, fake):
    from core.feasibility import CancelledError
    from ui.session import Session, load_project

    m, bdir = _make_batch(tmp_path)
    fake["bdir"] = bdir
    ev = threading.Event()
    fake["cancel_after_first"] = ev
    msgs = []
    done = threading.Event()
    t = bj.run_batch(
        bdir, lambda k, d: (msgs.append((k, d)), k == "finished" and done.set()), ev.is_set, True
    )
    t.join(60)
    assert done.is_set()
    fin = next(d for k, d in msgs if k == "finished")
    assert fin["status"] == "cancelled" and [r["status"] for r in fin["jobs"]] == ["partial"]
    ids = [j.id for j in m.jobs]
    assert _disk(bdir) == {ids[0]: "partial", ids[1]: "pending", ids[2]: "pending"}
    s = Session()
    load_project(s, bdir / ids[0])
    assert [r.label for r in s.results] == ["1 station"]  # the plan finished before Cancel is kept
    # run_job itself re-raises the cancel (it stops batch_job's loop)
    j = bt.load_manifest(bdir / bt.MANIFEST_NAME).jobs[1]
    with pytest.raises(CancelledError):
        bj.run_job(bt.load_manifest(bdir / bt.MANIFEST_NAME), j, bdir, lambda *a: None, ev.is_set)
    assert _disk(bdir)[ids[1]] == "partial"


def test_crashed_worker_marks_the_job_failed(tmp_path):
    m, bdir = _make_batch(tmp_path, n=1)
    job = m.jobs[0]
    job.status = "running"
    bt.update_job(bdir / bt.MANIFEST_NAME, job)
    from ui.jobs import WorkerError

    err = RuntimeError(
        "the worker process ended unexpectedly (exit code -9: killed (possibly out of memory))"
    )
    bj._mark_crashed(bdir / bt.MANIFEST_NAME, job.id, err)
    back = bt.load_manifest(bdir / bt.MANIFEST_NAME).jobs[0]
    assert back.status == "failed" and "possibly out of memory" in back.error
    assert (bdir / job.id / "error.txt").is_file()
    assert bj.one_line(WorkerError("KeyError", "'x'", "tb")) == "KeyError: 'x'"


def test_fresh_worker_per_job_isolates_failures(tmp_path):
    """Real worker processes (one per job): both jobs fail at the mesh check, each recorded."""
    m, bdir = _make_batch(tmp_path, n=2)
    for j in m.jobs:
        j.mesh_sha1 = "f" * 40
    bt.save_manifest(m, bdir / bt.MANIFEST_NAME)
    rows = []
    out = bj.drive(bdir, lambda k, d: k == "job" and rows.append(d))
    assert out["status"] == "done" and [r["status"] for r in rows] == ["failed", "failed"]
    assert all("changed since it was queued" in r["error"] for r in rows)
    assert all((bdir / r["id"] / "error.txt").is_file() for r in rows)


def test_planner_settings_roundtrip_and_spec():
    import json

    from core.collision_config import current
    from core.collision_mesh import Geom
    from core.cycle_time import CycleSettings

    st = PlannerSettings(
        max_stations=7, candidate_step=0.05, refine_method="tpe", cycle=CycleSettings(settle_s=2)
    )
    d = json.loads(json.dumps(bj.settings_dict(st)))  # JSON-safe
    assert bj.planner_settings_from({"settings": d}) == st
    s = empty_cell()  # static base: one station
    cfg = current().replace(dress_m=0.02, geom={"part": Geom("hull")}, link_pad={3: 0.01}, arm_pad=0.004)
    planner = {"mode": "quick", "settings": d, "collision": bj.collision_dict(cfg)}
    spec = bj.spec_for(json.loads(json.dumps(planner)), s)
    assert (
        spec["settings"].max_stations == 1 and not spec["settings"].refine and spec["settings"].patience == 1
    )
    assert spec["collision"] == cfg  # v12 #10: the queued CollisionConfig, through JSON and back
    from core.schema import Optimizer

    assert spec["weights"] == (s.extra.get("optimizer") or Optimizer()).weights  # the cell's
    assert "objective" not in spec and spec["rules"] is None
    cost = bj.spec_for({"settings": d, "weights": {"stations": 9.0}}, s)
    assert cost["weights"].stations == 9.0 and cost["objective"] == "weighted"
    with pytest.raises(ValueError, match="unknown planner setting"):
        bj.planner_settings_from({"settings": {"warp_drive": 1}})


def test_cell_project_is_relative(tmp_path):
    cell, bdir = tmp_path / "ws" / "study", tmp_path / "ws" / "batch_1"
    rel = bt.rel_cell(cell, bdir)
    assert rel == "../study"
    m = bt.BatchManifest(cell_project=rel)
    assert bt.cell_path(m, bdir) == cell.resolve()


def test_import_is_viser_free():
    code = "import sys, ui.batch_job, ui.report_make, ui.report_text; print('viser' in sys.modules)"
    out = subprocess.run([sys.executable, "-c", code], cwd=ROOT, capture_output=True, text=True, timeout=120)
    assert out.returncode == 0, out.stderr
    assert out.stdout.strip() == "False"


def test_snapshot_matches_the_station_panel():
    import viser

    from app import build_app
    from core.collision_config import current

    server = viser.ViserServer(host="127.0.0.1", port=8871, verbose=False)
    try:
        app = build_app(server)
        st, s = app["stations"], app["session"]
        st.step.value = 150.0
        snap = bj.planner_snapshot(st)
        assert snap["mode"] == "cost" and snap["objective"] == st.objective_m.key
        assert bj.planner_settings_from(snap, s) == st.planner_settings()
        spec = bj.spec_for(snap, s)
        assert spec["weights"] == st.costs.commit()
        assert spec["collision"] == current()
        assert bj.planner_snapshot(st, quick=True)["mode"] == "quick"
    finally:
        server.stop()


@pytest.mark.slow
def test_real_two_job_batch(tmp_path):
    """Two golden-plate jobs (top face; half of it), planned in-process, with exports."""
    import numpy as np

    from core.schema import load_project as read_project
    from core.transforms import make_T
    from ui.session import PartState, Session, load_project

    cell = empty_cell()
    cell.load_tool_from_library("orbital_sander")
    cell.part = PartState("assets/parts/flat_plate.stl", "m", make_T(p=(0.9, 0.0, 0.3)))
    m, bdir = _make_batch(tmp_path, n=2, cell=cell)
    pose = make_T(p=(0.9, 0.0, 0.3)).tolist()
    mesh = flat_plate()
    c = mesh.triangles_center
    for j in m.jobs:
        j.pose, j.gen = pose, GenSettings(row_m=0.15, col_m=0.15)
    m.jobs[1].selection = [f for f in m.jobs[0].selection if c[f, 0] < 0]  # the -x half of the top
    m.jobs[0].planner = m.jobs[1].planner = _planner(refine=False)
    bt.save_manifest(m, bdir / bt.MANIFEST_NAME)
    out = bj.drive(bdir, lambda *a: None, in_process=True)
    jobs = bt.load_manifest(bdir / bt.MANIFEST_NAME).jobs
    assert out["status"] == "done" and [j.status for j in jobs] == ["done", "done"], [j.error for j in jobs]
    a, b = (j.summary for j in jobs)
    assert a["total"] > b["total"] > 0 and a["stations"] == b["stations"] == 1
    for j in jobs:
        exp = bdir / j.id / "export"
        assert {"tcp_poses.csv", "userframes.csv", "report.html"} <= {p.name for p in exp.iterdir()}
        assert j.summary["report"] and not j.error
        p = read_project(bdir / j.id / "project.json")
        assert p.results and p.work.result_points
        s = Session()
        load_project(s, bdir / j.id)
        assert s.results[0].label == j.summary["label"] and np.isfinite(s.results[0].cost)


def test_moved_texts_are_the_old_names():
    """The viser-free ui.report_text is what log_panel / tolerance_ui still export."""
    from ui import log_panel, report_text, tolerance_ui

    assert log_panel.unreached_report is report_text.unreached_report
    assert log_panel.REASONS is report_text.REASONS and log_panel.reason_text is report_text.reason_text
    assert tolerance_ui.summary is report_text.tolerance_summary
