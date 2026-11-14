"""Batch queue manifest ("select now, optimize later"): pure data, no viser, no ui.

``batch.json`` lists jobs (part + face selection + generator and planner settings) that start
from one cell study. ``ui.batch_materialize`` turns a job into a study folder; the runner
(``ui.batch_job``) plans it. Writes are atomic (tmp file, then replace).
"""

from __future__ import annotations

import hashlib
import json
import os
from datetime import datetime
from pathlib import Path
from typing import Literal

import numpy as np
from pydantic import BaseModel, ConfigDict, Field

from core.target_gen import GenSettings

VERSION = 1
MANIFEST_NAME = "batch.json"
Status = Literal["pending", "running", "done", "failed", "cancelled", "partial"]
UNIT_SCALE = {"m": 1.0, "mm": 1e-3, "in": 0.0254}
from core.step_import import STEP_EXTS, StepImportError, read_step  # noqa: E402

MESH_EXTS = (".stl", ".obj", ".ply")
PART_EXTS = MESH_EXTS + STEP_EXTS  # a part may also be a STEP model


def _identity() -> list[list[float]]:
    return np.eye(4).tolist()


class BatchJob(BaseModel):
    model_config = ConfigDict(extra="forbid")

    id: str
    name: str = ""
    part: str  # mesh path relative to the batch folder (copied there at queue time)
    units: str = "m"
    pose: list[list[float]] = Field(default_factory=_identity)  # 4x4 T_world_part
    mesh_sha1: str = ""
    selection: list[int] = Field(default_factory=list)  # face ids
    gen: GenSettings = Field(default_factory=GenSettings)
    planner: dict = Field(default_factory=dict)  # PlannerSettings fields + objective/weights/workers
    status: Status = "pending"
    error: str = ""  # one line
    run_s: float = 0.0
    out_dir: str = ""
    summary: dict = Field(default_factory=dict)  # stations, reached, total, cost
    auto_confirmed: list[str] = Field(default_factory=list)  # setup steps confirmed by the batch


class BatchManifest(BaseModel):
    model_config = ConfigDict(extra="forbid")

    version: int = VERSION
    created: str = Field(default_factory=lambda: datetime.now().isoformat(timespec="seconds"))
    cell_project: str = ""  # cell study folder whose robot, tool, carrier, stay-outs, optimizer are reused
    jobs: list[BatchJob] = Field(default_factory=list)


def save_manifest(manifest: BatchManifest, path: Path) -> Path:
    """Write atomically: a crash never leaves half a file."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text(manifest.model_dump_json(indent=2), encoding="utf-8")
    os.replace(tmp, path)
    return path


def load_manifest(path: Path) -> BatchManifest:
    path = Path(path)
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as err:
        raise ValueError(f"cannot read the batch file {path}: {err}") from err
    if not isinstance(data, dict) or data.get("version", 0) > VERSION:
        raise ValueError(f"{path.name} was written by a newer version of the app; update the app to open it")
    return BatchManifest.model_validate(data)


def add_job(
    manifest: BatchManifest, *, part: str, selection, gen: GenSettings | None = None, **kw
) -> BatchJob:
    """Append a job with a unique id (``job_001``, ...). ``kw``: any other BatchJob field."""
    taken = {j.id for j in manifest.jobs}
    jid = kw.pop("id", None)
    n = len(manifest.jobs) + 1
    if jid is None:
        while f"job_{n:03d}" in taken:
            n += 1
        jid = f"job_{n:03d}"
    elif jid in taken:
        raise ValueError(f"job id '{jid}' already exists in this batch")
    kw.setdefault("name", jid)
    job = BatchJob(id=jid, part=part, selection=[int(f) for f in selection], gen=gen or GenSettings(), **kw)
    manifest.jobs.append(job)
    return job


def reset_interrupted(manifest: BatchManifest) -> int:
    """Resume: a job still 'running' was interrupted, so it is pending again."""
    n = 0
    for j in manifest.jobs:
        if j.status == "running":
            j.status, n = "pending", n + 1
    return n


def pending_jobs(manifest: BatchManifest) -> list[BatchJob]:
    """Jobs to run: pending, plus interrupted 'running' ones (reset to pending). Done jobs are
    skipped; failed, cancelled and partial jobs are not re-run unless the user resets them."""
    reset_interrupted(manifest)
    return [j for j in manifest.jobs if j.status == "pending"]


def mesh_sha1(path: Path) -> str:
    h = hashlib.sha1()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def check_job(job: BatchJob, batch_dir: Path) -> str | None:
    """A one-line reason why the job cannot run, or None."""
    import trimesh

    who = job.name or job.id
    p = Path(batch_dir) / job.part
    if not p.is_file():
        return f"{who}: mesh {job.part} is missing from the batch folder"
    if job.mesh_sha1 and mesh_sha1(p) != job.mesh_sha1:
        return f"{who}: mesh {job.part} changed since it was queued, so the selected faces no longer match"
    if job.units not in UNIT_SCALE:
        return f"{who}: unknown units '{job.units}'"
    if p.suffix.lower() not in PART_EXTS:
        return f"{who}: unsupported mesh type {p.suffix}"
    try:
        if p.suffix.lower() in STEP_EXTS:  # face ids index the tessellated triangles
            nf = len(read_step(p, "m").mesh.faces)
        else:
            nf = len(trimesh.load(p, force="mesh").faces)
    except StepImportError as err:
        return f"{who}: cannot read part {job.part}: {err}"
    except Exception as err:  # noqa: BLE001 - any loader failure is a reason
        return f"{who}: cannot read mesh {job.part}: {err}"
    if nf == 0:
        return f"{who}: mesh {job.part} contains no triangles"
    if not job.selection:
        return f"{who}: no faces selected"
    bad = [f for f in job.selection if f < 0 or f >= nf]
    if bad:
        return f"{who}: {len(bad)} selected face id(s) out of range (mesh has {nf} faces, e.g. {bad[0]})"
    return None


# --- B4: paths, per-job updates and the batch summary ------------------------------------------


def rel_cell(cell: Path | str, batch_dir: Path) -> str:
    """``cell_project`` as stored: relative to the batch folder when possible (both on one
    drive, e.g. ``../study_x``), so moving the workspace keeps the link; else absolute."""
    cell, batch_dir = Path(cell).resolve(), Path(batch_dir).resolve()
    try:
        return Path(os.path.relpath(cell, batch_dir)).as_posix()
    except ValueError:  # another drive (Windows)
        return str(cell)


def cell_path(manifest: BatchManifest, batch_dir: Path) -> Path:
    """The cell study folder: ``cell_project`` resolved against the batch folder."""
    p = Path(manifest.cell_project)
    return p if p.is_absolute() else (Path(batch_dir) / p).resolve()


def update_job(path: Path, job: BatchJob, fallback: BatchManifest | None = None) -> bool:
    """Write one job's entry into the manifest on disk, keeping every other job as the file has
    it (the GUI may queue or reorder jobs while one runs). A job no longer in the file (removed
    meanwhile) is not written back: False. ``fallback`` is saved whole when the file is
    missing or unreadable."""
    path = Path(path)
    try:
        current = load_manifest(path)
    except ValueError:
        if fallback is None:
            raise
        save_manifest(fallback, path)
        return True
    for i, j in enumerate(current.jobs):
        if j.id == job.id:
            current.jobs[i] = job.model_copy(deep=True)
            save_manifest(current, path)
            return True
    return False


SUMMARY_COLUMNS = ("job", "name", "part", "status", "stations", "reached", "total", "cost", "run_s")
SUMMARY_COLUMNS += ("error", "report")


def summary_rows(manifest: BatchManifest) -> list[dict]:
    """One row per job (SUMMARY_COLUMNS); empty cells for what a job does not have."""
    rows = []
    for j in manifest.jobs:
        s = j.summary or {}
        cost = s.get("cost")
        rows.append(
            {
                "job": j.id,
                "name": j.name,
                "part": j.part,
                "status": j.status,
                "stations": s.get("stations", ""),
                "reached": s.get("reached", ""),
                "total": s.get("total", ""),
                "cost": "" if cost is None else f"{cost:.3f}",
                "run_s": f"{j.run_s:.1f}" if j.run_s else "",
                "error": j.error,
                "report": s.get("report", ""),
            }
        )
    return rows


def summary_csv(manifest: BatchManifest) -> str:
    import csv
    import io

    buf = io.StringIO()
    w = csv.DictWriter(buf, SUMMARY_COLUMNS, lineterminator="\n")
    w.writeheader()
    w.writerows(summary_rows(manifest))
    return buf.getvalue()


def summary_html(manifest: BatchManifest) -> str:
    """A self-contained page (no scripts, no external links except the per-job reports)."""
    import html

    def cell(k: str, v) -> str:
        v = str(v)
        if k == "report" and v:
            return f'<td><a href="{html.escape(v, quote=True)}">report</a></td>'
        return f"<td>{html.escape(v)}</td>"

    head = "".join(f"<th>{c}</th>" for c in SUMMARY_COLUMNS)
    body = "\n".join(
        f'<tr class="{html.escape(r["status"])}">' + "".join(cell(k, r[k]) for k in SUMMARY_COLUMNS) + "</tr>"
        for r in summary_rows(manifest)
    )
    return (
        '<!doctype html><html><head><meta charset="utf-8"><title>Batch summary</title><style>'
        "body{font-family:sans-serif;margin:16px}table{border-collapse:collapse}"
        "td,th{border:1px solid #999;padding:3px 8px}.failed td{color:#c62828}"
        ".partial td,.cancelled td{color:#b26a00}@media(prefers-color-scheme:dark){body{background:#1e1e1e;"
        "color:#ddd}.failed td{color:#ef5350}}</style></head><body>"
        f"<h1>Batch summary</h1><p>Cell study: {html.escape(manifest.cell_project)}; created "
        f"{html.escape(manifest.created)}.</p><table><tr>{head}</tr>\n{body}</table></body></html>"
    )


def write_summary(manifest: BatchManifest, batch_dir: Path) -> tuple[Path, Path]:
    """``summary.csv`` and ``summary.html`` in the batch folder."""
    batch_dir = Path(batch_dir)
    out = []
    for name, text in (("summary.csv", summary_csv(manifest)), ("summary.html", summary_html(manifest))):
        p = batch_dir / name
        tmp = p.with_name(name + ".tmp")
        tmp.write_text(text, encoding="utf-8")
        os.replace(tmp, p)
        out.append(p)
    return out[0], out[1]
