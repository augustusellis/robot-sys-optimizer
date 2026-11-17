"""Package builder, Part tab (v13 #4): the part's position in the world frame, its targets, and
saving it as a part package (core.part_package) into a parts folder for batch runs.

Targets are made with the normal Targets tab of the main window (the user's choice): *Place in the
project* loads the part (the mesh made in this builder, or the project's part when the builder
holds none) at the given world pose; the user adds targets in the main window; *Save part package*
writes the project's part, pose and targets to ``<parts folder>/<name>/``. *Open part package*
brings one back for editing. While a package is being edited the main window's Targets tab shows
a note with its own *Save part package* button (``attach_targets_tab``).
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from core import part_package as pp
from core import schema
from core.schema import Pose
from core.transforms import matrix_to_pose, pose_to_matrix
from ui import batch_prefs
from ui.widgets import notify

PREF = "parts_folder"


def _rpy_deg(T: np.ndarray) -> tuple:
    return tuple(float(np.degrees(a)) for a in matrix_to_pose(T).rpy)


class PartPackagePanel:
    def __init__(self, server, builder):
        self.b, self.s = builder, builder.s
        self.editing: str | None = None  # name of the package being edited
        self.on_loaded = None  # app.py: the project's part / targets changed (cell + Targets redraw)
        self.notes: list = []  # the Targets tab's note + button (attach_targets_tab)
        gui = server.gui
        with gui.add_folder("Part package: world position and targets") as self.folder:
            gui.add_markdown(
                "_For batch runs (Optimize → **Run on several parts**): place the part in the world, add "
                "its targets in the main window's **Targets** tab, then save it into a parts folder._"
            )
            self.xyz = gui.add_vector3("Position in world (mm)", (0.0, 0.0, 0.0), step=1.0)
            self.rpy = gui.add_vector3("Rotation in world (deg)", (0.0, 0.0, 0.0), step=0.5)
            self.from_project = gui.add_button("Take the project part's position")
            self.place = gui.add_button("Place in the project", color="green")
            self.parts_folder = gui.add_text("Parts folder", batch_prefs.get(PREF))
            self.save = gui.add_button("Save part package", color="green")
            self.pick = gui.add_dropdown("Package in the folder", ("(none)",))
            self.list_btn = gui.add_button("List the folder")
            self.open_btn = gui.add_button("Open part package")
            self.info = gui.add_markdown("")
        self.from_project.on_click(lambda e: self.take_pose())
        self.place.on_click(lambda e: self.place_in_project(e))
        self.save.on_click(lambda e: self.ask_save(e))
        self.list_btn.on_click(lambda e: self.list_folder(e))
        self.open_btn.on_click(lambda e: self.open_package(e))
        self.parts_folder.on_update(lambda e: batch_prefs.put(PREF, self.parts_folder.value.strip()))
        self.take_pose()

    # --- helpers ---------------------------------------------------------------------------------

    def refresh(self) -> None:
        self.folder.visible = self.b.pkg.kind == "part"

    def take_pose(self) -> None:
        part = getattr(self.s, "part", None)  # scripts may give the builder a light session
        if part is not None:
            self.xyz.value = tuple(float(v) * 1000 for v in part.T_world_part[:3, 3])
            self.rpy.value = _rpy_deg(part.T_world_part)

    def T_world(self) -> np.ndarray:
        xyz = tuple(float(v) / 1000 for v in self.xyz.value)
        return pose_to_matrix(Pose(xyz=xyz, rpy=tuple(float(np.radians(a)) for a in self.rpy.value)))

    def folder_path(self) -> Path | None:
        text = self.parts_folder.value.strip()
        return Path(text) if text else None

    def _changed(self) -> None:
        if self.b.on_part is not None:
            self.b.on_part()
        if self.on_loaded is not None:
            self.on_loaded()
        self._notes()

    def _notes(self) -> None:
        text = (
            f"**Editing part package _{self.editing}_.** Add its targets here, then **Save part package** "
            "(here or in the package builder's Part tab)."
            if self.editing
            else ""
        )
        for w in self.notes:
            if hasattr(w, "content"):
                w.content = text
            w.visible = bool(self.editing)

    def attach_targets_tab(self, gui) -> None:
        """The main window's Targets tab: a note and a *Save part package* button while editing."""
        md = gui.add_markdown("", visible=False)
        btn = gui.add_button("Save part package", color="green", visible=False)
        btn.on_click(lambda e: self.ask_save(e))
        self.notes = [md, btn]

    # --- actions ---------------------------------------------------------------------------------

    def place_in_project(self, e=None) -> bool:
        """The builder's part (or the project's own part) into the project at the world pose."""
        from ui import package_actions

        if self.b.pkg.parts:
            if not package_actions.use(self.b, e):
                return False
        elif self.s.part is None:
            notify(e, "No part", "Add a mesh in the Parts folder above, or load a part in the project")
            return False
        self.s.part.T_world_part = self.T_world()
        self.editing = self.b.name.value.strip() or self.b.pkg.name
        self._changed()
        self.info.content = (
            f"Placed **{self.editing}** in the project. Add its targets in the main window "
            "(**Targets** tab), then **Save part package**."
        )
        return True

    def ask_save(self, e=None) -> None:
        """*Save part package*, asking first when there are no targets or the name is taken."""
        folder = self.folder_path()
        name = self.editing or self.b.name.value.strip() or self.b.pkg.name
        warn = []
        if not self.s.targets.points:
            warn.append("The part has **no targets**: a batch run would plan nothing for it.")
        if folder is not None and (folder / pp.safe(name) / pp.PKG_FILE).is_file():
            warn.append(f"**{pp.safe(name)}** is already in the parts folder: it will be replaced.")
        client = getattr(e, "client", None)
        if not warn or client is None:
            self.save_package(e, overwrite=client is not None)
            return
        with client.gui.add_modal("Save part package?") as modal:
            client.gui.add_markdown("  \n".join(warn))
            go = client.gui.add_button("Save", color="green")
            stop = client.gui.add_button("Cancel")

        @go.on_click
        def _(ev) -> None:
            modal.close()
            self.save_package(ev, overwrite=True)

        stop.on_click(lambda _ev: modal.close())

    def save_package(self, e=None, overwrite: bool = False) -> Path | None:
        s, folder = self.s, self.folder_path()
        if folder is None:
            notify(e, "No parts folder", "Type the folder the part packages go in (Parts folder)")
            return None
        if s.part is None:
            notify(e, "No part", "Place a part in the project first")
            return None
        name = self.editing or self.b.name.value.strip() or self.b.pkg.name
        part = schema.Part(
            mesh=s.part.mesh_path,
            units=s.part.units,
            pose=matrix_to_pose(s.part.T_world_part),
            collision_keep=[int(f) for f in s.part.collision_keep],
            collision_mesh=s.part.collision_mesh,
        )
        try:
            dest = pp.save_part_package(name, part, s.targets, s.resolve, folder, overwrite=overwrite)
        except (OSError, ValueError) as err:
            notify(e, "Not saved", str(err))
            return None
        batch_prefs.put(PREF, str(folder))
        n = len(s.targets.points)
        self.info.content = f"Saved **{name}** ({n} targets) to `{dest}`."
        notify(e, "Part package saved", str(dest), ok=True)
        self.list_folder()
        return dest

    def list_folder(self, e=None) -> list[str]:
        folder = self.folder_path()
        try:
            ok, bad = pp.scan_folder(folder) if folder is not None else ([], [])
        except ValueError as err:
            notify(e, "Cannot list", str(err))
            return []
        names = [d.name for d, _ in ok]
        self.pick.options = tuple(names) or ("(none)",)
        if bad:
            self.info.content = "Not usable: " + "; ".join(f"`{d.name}`: {why}" for d, why in bad)
        return names

    def open_package(self, e=None, name: str | None = None) -> bool:
        """A saved package back into the project (part, world pose and targets) for editing."""
        from ui.session import PartState

        folder = self.folder_path()
        name = name or self.pick.value
        if folder is None or name == "(none)":
            notify(e, "Nothing to open", "Type the parts folder and pick a package")
            return False
        d = folder / name
        try:
            pkg = pp.load_part_package(d)
        except ValueError as err:
            notify(e, "Cannot open", str(err))
            return False
        part = pkg.part
        coll = (d / part.collision_mesh).resolve().as_posix() if part.collision_mesh else None
        self.s.part = PartState(
            (d / part.mesh).resolve().as_posix(),
            part.units,
            pose_to_matrix(part.pose),
            list(part.collision_keep),
            coll,
        )
        self.s.targets = pkg.targets
        self.editing = pkg.name
        self.b.name.value = pkg.name
        self.take_pose()
        self._changed()
        self.info.content = f"Opened **{pkg.name}** ({len(pkg.targets.points)} targets) into the project."
        return True
