"""Turn a batch job into a study folder (cell + the job's part and generated targets)."""

from __future__ import annotations

import shutil
from pathlib import Path

import numpy as np

from core import batch as bt
from core import targets as tg
from core.schema import TargetGroup, Tolerances
from core.target_gen import generate_targets
from ui.session import PartState, Session, load_mesh, load_project, save_project
from ui.staleness import STEP_COMPONENTS, confirm_step


def ensure_group(session: Session, name: str) -> None:
    """Add target group ``name`` with the tool's tolerances (as the Targets tab does)."""
    if name in {g.name for g in session.targets.groups}:
        return
    tol = session.tool.tolerances if session.tool else Tolerances()
    g = TargetGroup(name=name, spin_deg=tol.spin_deg, tilt_deg=tol.tilt_deg, standoff_m=tol.standoff_m)
    session.targets = session.targets.model_copy(update={"groups": [*session.targets.groups, g]})


def materialize(manifest: bt.BatchManifest, job: bt.BatchJob, batch_dir: Path) -> Path:
    """Create ``batch_dir/<job.id>/`` as a study folder and return it. ValueError (one line)
    if the job cannot run or generates no targets. Sets ``job.auto_confirmed`` / ``out_dir``."""
    batch_dir = Path(batch_dir)
    who = job.name or job.id
    reason = bt.check_job(job, batch_dir)
    if reason:
        raise ValueError(reason)
    cell = Path(manifest.cell_project)
    if not manifest.cell_project or not (cell / "project.json").is_file():
        raise ValueError(f"{who}: the cell project '{manifest.cell_project}' has no project.json")
    out = batch_dir / job.id
    s = Session()
    load_project(s, cell)
    src = batch_dir / job.part
    rel = f"assets/parts/{src.name}"
    (out / "assets" / "parts").mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, out / rel)
    s.project_dir = out
    s.part = PartState(rel, job.units, np.array(job.pose, float))
    s.selection = (rel, [int(f) for f in job.selection])
    s.targets = s.targets.model_copy(update={"points": []})
    s.results, s.result_points, s.feasibility, s.stamps, s.steps = [], {}, {}, {}, {}
    mesh = load_mesh(out / rel, job.units)
    try:
        P, N, _ = generate_targets(mesh, job.selection, job.gen)
    except (ValueError, IndexError) as err:
        raise ValueError(f"{who}: target generation failed: {err}") from err
    if len(P) == 0:
        raise ValueError(f"{who}: no targets generated; check the selection and spacing")
    ensure_group(s, job.gen.group)
    s.targets = tg.append_points(s.targets, P, N, job.gen.group)
    save_project(s, out)
    # confirm on the reloaded session: the step hashes are taken from what a later load sees
    # (a pose of -0.0 saved as 0.0 would otherwise make the Cell step read "changed")
    load_project(s, out)
    for step in STEP_COMPONENTS:
        confirm_step(s, step)
    job.auto_confirmed = list(STEP_COMPONENTS)
    job.out_dir = job.id
    save_project(s, out)
    return out
