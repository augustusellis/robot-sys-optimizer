"""Batch manifest (core.batch) and job materialisation (ui.batch_materialize)."""

import shutil

import pytest

from core import batch as bt
from core.golden_parts import flat_plate
from core.schema import load_project as read_project
from core.target_gen import GenSettings
from ui.batch_materialize import materialize
from ui.session import Session, empty_cell, load_project, save_project
from ui.staleness import STEP_COMPONENTS, step_state


@pytest.fixture
def batch(tmp_path):
    save_project(empty_cell(), tmp_path / "cell")
    bdir = tmp_path / "batch"
    (bdir / "parts").mkdir(parents=True)
    mesh = flat_plate()
    mesh.export(bdir / "parts" / "plate.stl")
    m = bt.BatchManifest(cell_project=str(tmp_path / "cell"))
    top = [i for i, n in enumerate(mesh.face_normals) if n[2] > 0.9]
    job = bt.add_job(
        m,
        part="parts/plate.stl",
        selection=top,
        gen=GenSettings(row_m=0.1, col_m=0.1),
        mesh_sha1=bt.mesh_sha1(bdir / "parts" / "plate.stl"),
    )
    return m, job, bdir, mesh


def test_roundtrip_and_atomic(batch):
    m, job, bdir, _ = batch
    p = bdir / bt.MANIFEST_NAME
    bt.save_manifest(m, p)
    assert not [f for f in bdir.iterdir() if f.name.endswith(".tmp")]
    back = bt.load_manifest(p)
    assert back == m and back.version == bt.VERSION and back.jobs[0].gen.row_m == 0.1
    j2 = bt.add_job(m, part="parts/plate.stl", selection=[0])
    assert j2.id != job.id and len({j.id for j in m.jobs}) == 2


def test_pending_and_resume(batch):
    m, job, _, _ = batch
    j2 = bt.add_job(m, part="x", selection=[0])
    j3 = bt.add_job(m, part="x", selection=[0])
    job.status, j2.status = "done", "running"
    assert [j.id for j in bt.pending_jobs(m)] == [j2.id, j3.id]
    assert j2.status == "pending" and job.status == "done"


def test_check_job(batch):
    m, job, bdir, mesh = batch
    assert bt.check_job(job, bdir) is None
    bad = job.model_copy(update={"selection": [len(mesh.faces) + 5]})
    assert "out of range" in bt.check_job(bad, bdir)
    assert "missing" in bt.check_job(job.model_copy(update={"part": "nope.stl"}), bdir)
    mesh.apply_scale(2.0)
    mesh.export(bdir / "parts" / "plate.stl")
    assert "changed" in bt.check_job(job, bdir)


def test_materialize(batch):
    m, job, bdir, _ = batch
    out = materialize(m, job, bdir)
    assert out == bdir / job.id and (out / "assets" / "parts" / "plate.stl").is_file()
    s = Session()
    load_project(s, out)
    assert s.part.mesh_path == "assets/parts/plate.stl" and not s.results
    assert len(s.targets.points) > 10
    assert {g.name for g in s.targets.groups} == {"default"}
    assert {st: step_state(s, st) for st in STEP_COMPONENTS} == dict.fromkeys(STEP_COMPONENTS, "confirmed")
    assert job.auto_confirmed == list(STEP_COMPONENTS)
    assert read_project(out / "project.json").work.steps


def test_materialize_errors(batch):
    m, job, bdir, _ = batch
    shutil.rmtree(bdir / "parts")
    with pytest.raises(ValueError, match="missing"):
        materialize(m, job, bdir)
