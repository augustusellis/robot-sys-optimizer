"""ui.batch_queue_ui.BatchPanel (plan Part 2 step B5): queue, edit, run (faked), open a result."""

import threading

import numpy as np
import pytest
import viser

from core import batch as bt
from tests.helpers import confirm_all


@pytest.fixture(scope="module")
def app():
    from app import build_app

    server = viser.ViserServer(host="127.0.0.1", port=8871, verbose=False)
    state = build_app(server)
    assert state["project"].load_example("rail_long_part")
    yield state
    server.stop()


@pytest.fixture
def panel(app):
    p = app["batch_queue"]
    p.batch_dir = None
    app["targets"].selection = np.zeros(0, dtype=int)
    confirm_all(app["session"])
    return p


def _select(app, n=40):
    t = app["targets"]
    m = t.mesh()
    top = np.flatnonzero(m.face_normals[:, 2] > 0.9)[:n]
    t.selection = top
    t.gen_spacing = None
    return top


def _queue(app, panel, n=2):
    _select(app)
    for _ in range(n):
        assert panel.add_to_queue()


def test_add_refuses_without_selection(app, panel):
    assert panel.add_reason().startswith("Select")
    assert panel.add_to_queue() is False
    assert panel.batch_dir is None


def test_add_job_fields(app, panel):
    top = _select(app)
    assert panel.add_to_queue()
    m = panel.manifest()
    j = m.jobs[0]
    s = app["session"]
    assert j.selection == [int(f) for f in top] and j.units == s.part.units
    assert (panel.batch_dir / j.part).is_file() and j.mesh_sha1 == bt.mesh_sha1(panel.batch_dir / j.part)
    assert j.planner["mode"] == "cost" and j.summary["targets_preview"] > 0
    assert bt.cell_path(m, panel.batch_dir) == s.project_dir.resolve()
    assert "job_001" in panel.table.content
    panel.mode.value = "Quick plan"
    assert panel.add_to_queue()
    assert panel.manifest().jobs[1].planner["mode"] == "quick"
    assert panel.manifest().jobs[1].part == j.part  # identical mesh is reused
    panel.mode.value = "Plan stations"


def test_add_refuses_zero_targets(app, panel, monkeypatch):
    _select(app)
    monkeypatch.setattr("core.target_gen.generate_targets", lambda *a, **k: (np.zeros((0, 3)),) * 2 + ([],))
    assert panel.add_to_queue() is False and panel.batch_dir is None


def test_remove_reorder_reset(app, panel):
    _queue(app, panel, 3)
    ids = lambda: [j.id for j in panel.manifest().jobs]  # noqa: E731
    assert ids() == ["job_001", "job_002", "job_003"]
    panel.pick.value = "job_003"
    assert panel.move(-1) and ids() == ["job_001", "job_003", "job_002"]
    panel.pick.value = "job_001"
    assert panel.move(-1) is False
    assert panel.remove() and ids() == ["job_003", "job_002"]
    panel.pick.value = "job_002"
    assert panel.reset() is False  # pending: nothing to reset
    j = panel.manifest().jobs[1]
    j.status, j.error = "failed", "boom"
    bt.update_job(panel.batch_dir / bt.MANIFEST_NAME, j)
    assert panel.reset() and panel.manifest().jobs[1].status == "pending"
    assert panel.manifest().jobs[1].error == ""


def test_running_blocks_non_pending_edits(app, panel):
    _queue(app, panel, 2)
    j = panel.manifest().jobs[0]
    j.status = "running"
    bt.update_job(panel.batch_dir / bt.MANIFEST_NAME, j)
    ev = threading.Event()
    panel._thread = threading.Thread(target=ev.wait, daemon=True)
    panel._thread.start()
    try:
        panel.pick.value = "job_001"
        assert panel.remove() is False
        assert panel.move(1) is False
        assert panel.add_to_queue() is False
        panel.pick.value = "job_002"
        assert panel.remove() is True
        app["stations"].start(None)  # the Station planner refuses while a batch runs
    finally:
        ev.set()
        panel._thread.join()


def test_run_streams_and_finishes(app, panel, monkeypatch):
    _queue(app, panel, 2)
    seen = []

    def fake_run_batch(batch_dir, on_msg, cancelled=lambda: False, in_process=False):
        def work():
            path = batch_dir / bt.MANIFEST_NAME
            m = bt.load_manifest(path)
            rows = []
            for i, j in enumerate(m.jobs):
                on_msg("progress", (f"job {i + 1}", i / 2))
                j.status = "done" if i == 0 else "failed"
                j.error = "" if i == 0 else "mesh broke"
                j.out_dir = j.id
                if i == 0:
                    j.summary = {"stations": 2, "reached": 5, "total": 5, "cost": 1.5}
                bt.update_job(path, j)
                seen.append(panel.running)
                rows.append({"id": j.id, "status": j.status})
                on_msg("job", rows[-1])
            bt.write_summary(bt.load_manifest(path), batch_dir)
            on_msg("finished", {"status": "done", "jobs": rows, "error": ""})

        t = threading.Thread(target=work, daemon=True)
        t.start()
        return t

    monkeypatch.setattr("ui.batch_job.run_batch", fake_run_batch)
    panel.run(block=True)
    t = panel.table.content
    assert "done" in t and "failed" in t and "mesh broke" in t
    assert "1 done, 1 failed" in panel.status.content and "summary.html" in panel.status.content
    assert panel.last_summary.is_file() and panel.run_btn.visible and not panel.stop.visible
    assert seen == [True, True]


def test_pdf_checkbox_is_saved_with_the_batch(app, panel, monkeypatch):
    _queue(app, panel, 1)
    done = threading.Thread(target=lambda: None)
    done.start()
    monkeypatch.setattr("ui.batch_job.run_batch", lambda *a, **k: done)
    panel.pdf.value = True
    panel.run(block=True)
    assert panel.manifest().pdf is True
    panel.pdf.value = False
    assert panel.open_batch(panel.batch_dir.name) and panel.pdf.value is True  # reopening shows it
    panel.pdf.value = False
    panel.run(block=True)
    assert panel.manifest().pdf is False
    panel.run_btn.visible, panel.stop.visible, panel.progress.visible = True, False, False


def test_run_nothing_pending(app, panel):
    _queue(app, panel, 1)
    j = panel.manifest().jobs[0]
    j.status = "done"
    bt.update_job(panel.batch_dir / bt.MANIFEST_NAME, j)
    panel.run(block=True)
    assert panel.running is False and panel.progress.visible is False


def test_open_result_loads_study(app, panel):
    _queue(app, panel, 1)
    s = app["session"]
    study = panel.batch_dir / "job_001"
    app["project"].save_now(study)
    j = panel.manifest().jobs[0]
    assert panel.open_result() is False  # pending
    j.status, j.out_dir = "done", "job_001"
    bt.update_job(panel.batch_dir / bt.MANIFEST_NAME, j)
    assert panel.open_result() is True
    assert s.project_dir.resolve() == study.resolve()


def test_open_existing_batch(app, panel):
    _queue(app, panel, 1)
    name = panel.batch_dir.name
    panel.batch_dir = None
    assert name in panel.list_batches()
    assert panel.open_batch(name) and len(panel.manifest().jobs) == 1
