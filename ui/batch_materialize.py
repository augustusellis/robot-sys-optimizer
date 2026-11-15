"""Turn a batch job into a study folder (cell + the job's part and generated targets)."""

from __future__ import annotations

import json
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


def up_in_part(T_world_part) -> np.ndarray:
    """World +Z expressed in the part frame (the part pose may be rotated)."""
    return np.asarray(T_world_part, float)[:3, :3].T @ np.array([0.0, 0.0, 1.0])


def detect_for_pose(mesh, T_world_part, pad_radius_m=None):
    """Run the tool-surface detector on a part mesh placed with ``T_world_part``. STEP parts
    pass their CAD face ids (``mesh.metadata["cad_face_ids"]``)."""
    from core.tool_surface_pick import detect_tool_surface

    return detect_tool_surface(
        mesh,
        up=up_in_part(T_world_part),
        cad_face_ids=mesh.metadata.get("cad_face_ids"),
        pad_radius_m=pad_radius_m,
    )


def cell_pad_radius(manifest: bt.BatchManifest, batch_dir: Path) -> float | None:
    """The cell tool's pad radius (m) from its project.json, or None (unknown / unreadable)."""
    try:
        data = json.loads((bt.cell_path(manifest, batch_dir) / "project.json").read_text("utf-8"))
        r = (data.get("end_effector") or {}).get("pad_radius_m")
        return float(r) if r else None
    except (OSError, ValueError, TypeError, AttributeError):
        return None


def apply_rule(job: bt.BatchJob, mesh, who: str, pad_radius_m: float | None = None) -> None:
    """tool_surface job: detect, store confidence + summary, set ``job.selection``. Raises
    ``bt.NeedsReview`` (one line) when the confidence is below ``job.min_confidence``."""
    g = detect_for_pose(mesh, np.array(job.pose, float), pad_radius_m)
    job.confidence = float(g.confidence)
    job.detect_summary = str(g.summary).replace("²", "2")
    if g.confidence < job.min_confidence or len(g.faces) == 0:
        raise bt.NeedsReview(
            f"{who}: tool surface confidence {g.confidence:.2f} is below {job.min_confidence:.2f}; "
            f"review it ({job.detect_summary})"
        )
    job.selection = [int(f) for f in g.faces]


def materialize(manifest: bt.BatchManifest, job: bt.BatchJob, batch_dir: Path) -> Path:
    """Create ``batch_dir/<job.id>/`` as a study folder and return it. ValueError (one line)
    if the job cannot run or generates no targets. Sets ``job.auto_confirmed`` / ``out_dir``."""
    batch_dir = Path(batch_dir)
    who = job.name or job.id
    reason = bt.check_job(job, batch_dir)
    if reason:
        raise ValueError(reason)
    if job.selection_rule == "tool_surface":  # before anything is copied: a doubtful job stays cheap
        apply_rule(job, load_mesh(batch_dir / job.part, job.units), who, cell_pad_radius(manifest, batch_dir))
    cell = bt.cell_path(manifest, batch_dir)
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
