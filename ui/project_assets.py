"""A project's files live in its own folder (v9, issue #12).

``bundle`` (every save): each file the project refers to that is neither inside the project folder
nor shipped with the app (``assets/``, ``examples/`` of the repo) is copied into
``<project>/assets/imported/...`` and the project refers to the copy: the part mesh, the tool's URDF
folder, carrier and stay-out meshes (and their collision meshes), an uploaded robot's URDF folder.
A project then opens on any PC and keeps working when the folder it came from is moved or deleted.

``tolerate_missing`` (every open): a file that cannot be found no longer stops the whole project
from opening; that item is left out and a warning says what to load again.
"""

from __future__ import annotations

import filecmp
import shutil
from pathlib import Path

from core import schema

REPO = Path(__file__).resolve().parents[1]
SHIPPED = (REPO / "assets", REPO / "examples")
IMPORTED = Path("assets") / "imported"


def _inside(path: Path, folder: Path) -> bool:
    try:
        return path.resolve().is_relative_to(folder.resolve())
    except OSError:
        return False


def is_local(path: Path, project_dir: Path) -> bool:
    """In the project folder or shipped with the app: nothing to copy."""
    return _inside(path, project_dir) or any(_inside(path, d) for d in SHIPPED)


def _free(dst: Path, src: Path) -> Path:
    """``dst``, or ``dst`` with _2, _3... when another (different) file / folder has that name."""
    k, out = 2, dst
    while out.exists():
        same = out.is_file() and src.is_file() and filecmp.cmp(out, src, shallow=False)
        if same or (out.is_dir() and src.is_dir() and _inside(src, out)):
            return out
        out = dst.with_name(f"{dst.stem}_{k}{dst.suffix}")
        k += 1
    return out


def _copy_file(src: Path, project_dir: Path, sub: str) -> Path:
    dst = _free(project_dir / IMPORTED / sub / src.name, src)
    if not dst.exists():
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst)
    return dst


def _copy_folder(src: Path, project_dir: Path, sub: str) -> Path:
    dst = _free(project_dir / IMPORTED / sub / src.name, src)
    if not dst.exists():
        shutil.copytree(src, dst)
    return dst


def _external(session, rel: str | None) -> Path | None:
    """The file ``rel`` refers to when it must be copied into the project, else None."""
    if not rel:
        return None
    try:
        path = session.resolve(rel)
    except FileNotFoundError:
        return None
    return None if is_local(path, session.project_dir) else path


def _primitive(session, p: schema.Primitive, sub: str, notes: list[str]) -> schema.Primitive:
    upd = {}
    for field in ("mesh", "collision_mesh"):
        src = _external(session, getattr(p, field, None))
        if src is not None:
            upd[field] = session.relative(_copy_file(src, session.project_dir, sub))
            notes.append(f"{src.name} → {upd[field]}")
    return p.model_copy(update=upd) if upd else p


def bundle(session) -> list[str]:
    """Copy the project's outside files into its folder and point the session at the copies.
    Returns one note per copied item."""
    notes: list[str] = []
    s = session
    part = s.part
    src = _external(s, part.mesh_path) if part is not None else None
    if src is not None:
        old = part.mesh_path
        part.mesh_path = s.relative(_copy_file(src, s.project_dir, "parts"))
        if s.selection and s.selection[0] == old:  # the surface selection belongs to this mesh
            s.selection = (part.mesh_path, s.selection[1])
        notes.append(f"part {src.name} → {part.mesh_path}")
    ee = s.tool
    src = _external(s, ee.urdf) if ee is not None else None
    if src is not None:
        folder = _copy_folder(src.parent, s.project_dir, "tools")
        s.tool = ee.model_copy(update={"urdf": s.relative(folder / src.name)})
        notes.append(f"tool {src.parent.name} → {s.tool.urdf}")
    if s.base.bodies:
        bodies = [_primitive(s, p, "carrier", notes) for p in s.base.bodies]
        if any(a is not b for a, b in zip(bodies, s.base.bodies, strict=True)):
            s.base = s.base.model_copy(update={"bodies": bodies})
    if s.stayouts:
        s.stayouts = [_primitive(s, p, "stayouts", notes) for p in s.stayouts]
    src = _external(s, s.robot_urdf) if s.robot_name is None else None
    if src is not None:
        folder = _copy_folder(src.parent, s.project_dir, "robots")
        s.robot_urdf = s.relative(folder / src.name)
        notes.append(f"robot {src.parent.name} → {s.robot_urdf}")
    return notes


def _exists(session, rel: str | None) -> bool:
    if not rel:
        return True
    try:
        return session.resolve(rel).exists()
    except FileNotFoundError:
        return False


SKIP_DIRS = {"export", "cache", ".git"}
FIND_LIMIT = 20000  # files looked at when searching the project folder for a missing one


def find_in_project(session, rel: str | None) -> str | None:
    """v12 #7: a file of the same name anywhere in the project folder (a project copied to another
    PC with its files put next to it, or a file given with *Locate*), as a project-relative path."""
    root = getattr(session, "project_dir", None)
    if not rel or root is None or not Path(root).is_dir():
        return None
    name = Path(str(rel).replace("\\", "/")).name
    for k, p in enumerate(Path(root).rglob(name)):
        if k > FIND_LIMIT:
            break
        if p.is_file() and not SKIP_DIRS & set(p.relative_to(root).parts):
            return session.relative(p)
    return None


def _check(session, rel, what: str, warn: list[str]):
    """(path to use, found): ``rel`` when it exists, else a same-named file in the project folder."""
    if _exists(session, rel):
        return rel, True
    found = find_in_project(session, rel)
    if found is not None:
        warn.append(f"{what} `{rel}` not found: using `{found}` (same name, in the project folder)")
        return found, True
    return rel, False


def tolerate_missing(session, project: schema.Project) -> tuple[schema.Project, list[str]]:
    """The project without the items whose files are missing, and a warning for each. A file
    with the same name in the project folder is used instead (v12 #7). The left-out part, carrier
    bodies and stay-out zones are kept in ``session.missing`` so that saving keeps them
    (``keep_missing``) and *Locate missing files* can restore them."""
    warn: list[str] = []
    upd: dict = {}
    missing: list[dict] = []
    pr = project.robot
    if pr is not None and pr.name is None:
        urdf, ok = _check(session, pr.urdf, "robot URDF", warn)
        if not ok:
            warn.append(
                f"robot URDF `{pr.urdf}` not found: using the UR20; load the robot again (Robot & cell)"
            )
            upd["robot"] = pr.model_copy(
                update={"name": "ur20", "urdf": None, "home_deg": None, "joint_limits_deg": None}
            )
        elif urdf != pr.urdf:
            upd["robot"] = pr.model_copy(update={"urdf": urdf})
    if project.part is not None:
        mesh, ok = _check(session, project.part.mesh, "part mesh", warn)
        if not ok:
            warn.append(
                f"part mesh `{project.part.mesh}` not found: "
                "locate it or load the part again (Robot & cell → Part)"
            )
            missing.append({"kind": "part", "path": project.part.mesh, "item": project.part})
            upd["part"] = None
        elif mesh != project.part.mesh:
            upd["part"] = project.part.model_copy(update={"mesh": mesh})
    ee = project.end_effector
    if ee is not None and ee.urdf:
        urdf, ok = _check(session, ee.urdf, f"tool `{ee.name}`", warn)
        if not ok:
            warn.append(f"tool `{ee.name}` ({ee.urdf}) not found: load the tool again (Tool tab)")
            upd["end_effector"] = None
        elif urdf != ee.urdf:
            upd["end_effector"] = ee.model_copy(update={"urdf": urdf})
    bodies = []
    for p in project.base.bodies:
        p, ok = _primitive_files(session, p, f"carrier body `{p.name}`", warn)
        if not ok:
            warn.append(f"carrier body `{p.name}` ({p.mesh}) not found: left out (External axis tab)")
            missing.append({"kind": "carrier", "path": p.mesh, "item": p})
            continue
        bodies.append(p)
    if len(bodies) != len(project.base.bodies) or any(
        a is not b for a, b in zip(bodies, project.base.bodies, strict=False)
    ):
        upd["base"] = project.base.model_copy(update={"bodies": bodies})
    keep = []
    for p in project.stayouts:
        p, ok = _primitive_files(session, p, f"stay-out zone `{p.name}`", warn)
        if not ok:
            warn.append(f"stay-out zone `{p.name}` ({p.mesh}) not found: left out")
            missing.append({"kind": "stayout", "path": p.mesh, "item": p})
            continue
        keep.append(p)
    if len(keep) != len(project.stayouts) or any(
        a is not b for a, b in zip(keep, project.stayouts, strict=False)
    ):
        upd["stayouts"] = keep
    session.missing = missing
    return (project.model_copy(update=upd) if upd else project), warn


def _primitive_files(session, p: schema.Primitive, what: str, warn: list[str]):
    """(primitive with found paths, mesh present); a missing collision mesh is dropped (the
    visual mesh is checked instead)."""
    upd = {}
    if p.shape == "mesh":
        mesh, ok = _check(session, p.mesh, what, warn)
        if not ok:
            return p, False
        if mesh != p.mesh:
            upd["mesh"] = mesh
    if p.collision_mesh:
        coll, ok = _check(session, p.collision_mesh, f"{what} collision mesh", warn)
        if not ok or coll != p.collision_mesh:
            upd["collision_mesh"] = coll if ok else None
    return (p.model_copy(update=upd) if upd else p), True


def keep_missing(session, project: schema.Project) -> schema.Project:
    """v12 #7: the project file keeps the items left out at opening (their old paths), so an
    autosave does not drop the part or a body for good; a file located later fills them in."""
    miss = getattr(session, "missing", None) or []
    if not miss:
        return project
    upd: dict = {}
    for m in miss:
        if m["kind"] == "part" and project.part is None:
            upd["part"] = m["item"]
    bodies = [m["item"] for m in miss if m["kind"] == "carrier"]
    names = {b.name for b in project.base.bodies}
    bodies = [b for b in bodies if b.name not in names]
    if bodies:
        upd["base"] = project.base.model_copy(update={"bodies": [*project.base.bodies, *bodies]})
    zones = [m["item"] for m in miss if m["kind"] == "stayout"]
    zones = [z for z in zones if z.name not in {p.name for p in project.stayouts}]
    if zones:
        upd["stayouts"] = [*project.stayouts, *zones]
    return project.model_copy(update=upd) if upd else project


SUB = {"part": "parts", "carrier": "carrier", "stayout": "stayouts"}


def locate(session, index: int, data: bytes) -> Path:
    """Write a located file into ``<project>/assets/imported/<kind>/<original name>`` (the name
    the project refers to, so ``find_in_project`` picks it up on the next opening)."""
    m = session.missing[index]
    name = Path(str(m["path"]).replace("\\", "/")).name
    dst = Path(session.project_dir) / IMPORTED / SUB[m["kind"]] / name
    dst.parent.mkdir(parents=True, exist_ok=True)
    dst.write_bytes(data)
    return dst
