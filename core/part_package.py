"""Part packages (v13 #3): one part with its pose in the world frame and its targets, saved as a
folder so the station optimizer can run on many parts in a batch (ui.batch_part).

Layout: ``<parts folder>/<name>/part.json`` (``PartPackage``) plus the part mesh and, when the
package builder made one, its collision mesh, both next to ``part.json``. Mesh paths in
``part.json`` are file names in that folder, so a package folder can be copied or moved as a whole.

Only the part, its world pose and the targets are in a package (the user's choice, 2026-11-17):
the robot, tool, external axis, stay-outs and optimizer settings come from the project a batch
runs with (``apply_to_project``).
"""

from __future__ import annotations

import json
import re
import shutil
from collections.abc import Callable
from pathlib import Path

from pydantic import Field, ValidationError

from core import schema
from core.schema_base import _Model

PKG_FILE = "part.json"
PKG_VERSION = 1
NAME = re.compile(r"[^A-Za-z0-9_-]+")


def safe(name: str) -> str:
    """The folder name of a package (letters, digits, ``_`` and ``-``)."""
    return NAME.sub("_", name).strip("_") or "part"


class PartPackage(_Model):
    version: int = PKG_VERSION
    name: str = Field(min_length=1, max_length=64)
    description: str = ""
    part: schema.Part  # mesh / collision_mesh: file names in the package folder; pose in the world
    targets: schema.Targets = Field(default_factory=schema.Targets)


def save_part_package(
    name: str,
    part: schema.Part,
    targets: schema.Targets,
    resolve: Callable[[str], Path],
    folder: Path,
    overwrite: bool = False,
    description: str = "",
) -> Path:
    """Write ``<folder>/<safe name>/`` and return that folder. ``resolve`` turns the part's mesh
    paths into files (``Session.resolve``). A folder that already holds a package is only
    replaced with ``overwrite``; a folder holding something else is never touched."""
    dest = Path(folder) / safe(name)
    if dest.exists():
        if not (dest / PKG_FILE).is_file():
            raise ValueError(f"{dest} exists and is not a part package: choose another name")
        if not overwrite:
            raise FileExistsError(f"a part package named {dest.name} is already in {Path(folder)}")
    mesh_src = Path(resolve(part.mesh))
    coll_src = Path(resolve(part.collision_mesh)) if part.collision_mesh else None
    for f in (mesh_src, coll_src):
        if f is not None and not f.is_file():
            raise FileNotFoundError(f"part mesh not found: {f}")
    dest.mkdir(parents=True, exist_ok=True)
    mesh_name = mesh_src.name
    coll_name = None
    if coll_src is not None:
        coll_name = coll_src.name if coll_src.name != mesh_name else f"collision_{coll_src.name}"
    for old in dest.iterdir():  # a replaced package keeps no stale meshes
        if old.is_file() and old.name not in (PKG_FILE, mesh_name, coll_name):
            old.unlink()
    _copy(mesh_src, dest / mesh_name)
    if coll_src is not None:
        _copy(coll_src, dest / coll_name)
    pkg = PartPackage(
        name=name,
        description=description,
        part=part.model_copy(update={"mesh": mesh_name, "collision_mesh": coll_name}),
        targets=targets,
    )
    tmp = dest / (PKG_FILE + ".tmp")
    tmp.write_text(pkg.model_dump_json(indent=2), encoding="utf-8")
    tmp.replace(dest / PKG_FILE)
    return dest


def _copy(src: Path, dst: Path) -> None:
    if src.resolve() != dst.resolve():
        shutil.copy2(src, dst)


def load_part_package(pkg_dir: Path) -> PartPackage:
    """Read and check one package folder; ValueError with the reason when it is not usable."""
    pkg_dir = Path(pkg_dir)
    f = pkg_dir / PKG_FILE
    if not f.is_file():
        raise ValueError(f"no {PKG_FILE} in {pkg_dir}")
    try:
        data = json.loads(f.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as err:
        raise ValueError(f"{PKG_FILE} cannot be read: {err}") from err
    try:
        pkg = PartPackage.model_validate(data)
    except ValidationError as err:
        first = err.errors()[0]
        where = ".".join(str(x) for x in first["loc"])
        raise ValueError(f"{PKG_FILE}: {where}: {first['msg']}") from err
    if pkg.version > PKG_VERSION:
        raise ValueError(f"made by a newer version of the app (package version {pkg.version})")
    for name in (pkg.part.mesh, pkg.part.collision_mesh):
        if name is not None and not (pkg_dir / name).is_file():
            raise ValueError(f"mesh file {name} is missing")
    return pkg


def scan_folder(folder: Path) -> tuple[list[tuple[Path, PartPackage]], list[tuple[Path, str]]]:
    """The packages one level below ``folder`` (sorted by folder name), and the folders with a
    ``part.json`` that cannot be used, with the reason. Other folders are ignored."""
    folder = Path(folder)
    if not folder.is_dir():
        raise ValueError(f"not a folder: {folder}")
    ok, bad = [], []
    for d in sorted(p for p in folder.iterdir() if p.is_dir()):
        if not (d / PKG_FILE).is_file():
            continue
        try:
            ok.append((d, load_part_package(d)))
        except ValueError as err:
            bad.append((d, str(err)))
    return ok, bad


def apply_to_project(project: schema.Project, pkg: PartPackage, pkg_dir: Path) -> schema.Project:
    """A copy of ``project`` with the package's part (absolute mesh paths), world pose and targets.
    Plans, compared robots and the work state are cleared (they belong to another part); the
    robot, tool, external axis, stay-outs and optimizer settings are kept, and so are the saved
    panel settings (``work.ui``). ValueError when the targets ask for TCPs the tool lacks."""
    pkg_dir = Path(pkg_dir).resolve()
    coll = pkg.part.collision_mesh
    part = pkg.part.model_copy(
        update={
            "mesh": (pkg_dir / pkg.part.mesh).as_posix(),
            "collision_mesh": (pkg_dir / coll).as_posix() if coll else None,
        }
    )
    data = project.model_dump()
    data.update(
        part=part.model_dump(),
        targets=pkg.targets.model_dump(),
        results=[],
        compared=[],
        work=schema.WorkState(ui=dict(project.work.ui)).model_dump(),
    )
    try:
        return schema.Project.model_validate(data)
    except ValidationError as err:
        raise ValueError(f"{pkg.name}: {err.errors()[0]['msg']}") from err
