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
MESH_EXTS = (".stl", ".obj", ".ply")


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
    if p.suffix.lower() not in MESH_EXTS:
        return f"{who}: unsupported mesh type {p.suffix}"
    try:
        nf = len(trimesh.load(p, force="mesh").faces)
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
