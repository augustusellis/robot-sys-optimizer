"""Package builder -> Frames and travel -> *What may touch the part* (v10 #11).

Per TCP of a tool package: whole parts that may touch the part (the pad, a brush, a nozzle tip) and
faces picked on mesh parts in the builder's 3D view (click = a smooth surface region, brush = a dab
of the brush radius, box = every face in a screen rectangle; add or remove). The picks live on the
package's TCP frames (``urdf_package.Frame.contact_parts`` / ``contact_faces``, by part name, saved
in package.yaml); *Use in this project* writes them into the tool's TCPs (core.urdf_package.to_tool:
the part's link, face ids of that link's visual mesh, which is the part's mesh file as loaded here).
Everything not picked collides with the part. The preview shows contact parts and faces green.
"""

from __future__ import annotations

import numpy as np
import trimesh

from core.transforms import make_T, matrix_to_wxyz, pose_to_matrix, wxyz_to_matrix

GREEN = (60, 190, 90)
MODES = ("Off", "Click + (surface)", "Click − (surface)", "Brush +", "Brush −", "Box +", "Box −")
LIFT = 4e-4  # m: picked faces drawn just above the part's surface


class ContactPicker:
    def __init__(self, server, b):
        self.server, self.b = server, b
        self._rect_cb = None
        self._handles: list = []
        gui = server.gui
        with gui.add_folder("What may touch the part", expand_by_default=False) as self.folder:
            gui.add_markdown(
                "_For the TCP chosen above: the parts or faces that may touch the workpiece (a pad, a "
                "brush, a nozzle tip). Everything else on the tool still counts as a collision. Shown "
                "green in the preview._"
            )
            self.whole = gui.add_checkbox(
                "Selected part may touch", False, hint="the whole part (Parts → Part)"
            )
            self.mode = gui.add_dropdown(
                "Pick faces", MODES, hint="on the selected mesh part, in the 3D view"
            )
            self.angle = gui.add_slider(
                "Surface angle (deg)", 1.0, 60.0, 1.0, 15.0, hint="click: grow across edges up to this"
            )
            self.radius = gui.add_number("Brush radius (mm)", 15.0, min=1.0, max=500.0, step=1.0)
            self.clear = gui.add_button("Clear picked faces of this part")
            from ui.radius_ui import LABELS

            self.min_r = gui.add_number(  # v10 T12
                "Minimum surface radius (mm)", 0.0, min=0.0, max=10000.0, step=1.0, hint="this TCP; 0 = any"
            )
            self.applies = gui.add_dropdown("Radius limit applies to", tuple(LABELS.values()))
            from ui.travel_ui import LABELS as TRAVEL

            self.travel = gui.add_dropdown("Travel direction", tuple(TRAVEL.values()))  # v11 #7
            self.scan_w = gui.add_number("Scan width (mm)", 0.0, min=0.0, max=2000.0, step=1.0)
            self.overrun = gui.add_number("Overrun past edges (mm)", 0.0, min=0.0, max=1000.0, step=5.0)
            self.info = gui.add_markdown("")
        self.whole.on_update(lambda _e: None if b._busy else self.set_whole(self.whole.value))
        self.mode.on_update(lambda _e: self._mode_changed())
        self.min_r.on_update(lambda _e: None if b._busy else self._radius())
        self.applies.on_update(lambda _e: None if b._busy else self._radius())
        for w in (self.travel, self.scan_w, self.overrun):
            w.on_update(lambda _e: None if b._busy else self._travel())
        self.clear.on_click(lambda _e: self.set_faces(self._part().name, []) if self._part() else None)

    # --- state ---------------------------------------------------------------------------------

    def _tcp(self):
        t = self.b.pkg.tcps
        return t[min(self.b.sel_tcp, len(t) - 1)] if t else None

    def _part(self):
        p = self.b.pkg.parts
        return p[min(self.b.sel, len(p) - 1)] if p else None

    def _set_tcp(self, **upd) -> None:
        tcps = list(self.b.pkg.tcps)
        k = min(self.b.sel_tcp, len(tcps) - 1)
        tcps[k] = tcps[k].model_copy(update=upd)
        self.b._update(tcps=tcps)  # redraws the preview
        self.refresh()

    def set_whole(self, on: bool, part: str | None = None) -> None:
        t, name = self._tcp(), part or (self._part().name if self._part() else None)
        if t is None or name is None:
            return
        names = [n for n in t.contact_parts if n != name] + ([name] if on else [])
        self._set_tcp(contact_parts=names)

    def faces(self, part: str) -> np.ndarray:
        t = self._tcp()
        return np.asarray(t.contact_faces.get(part, []) if t is not None else [], int)

    def set_faces(self, part: str, ids) -> None:
        t = self._tcp()
        if t is None:
            return
        faces = {n: v for n, v in t.contact_faces.items() if n != part}
        ids = sorted({int(i) for i in ids})
        if ids:
            faces[part] = ids
        self._set_tcp(contact_faces=faces)

    def change_faces(self, part: str, ids, add: bool) -> None:
        cur = self.faces(part)
        self.set_faces(part, np.union1d(cur, ids) if add else np.setdiff1d(cur, ids))

    def is_contact(self, part: str) -> bool:
        t = self._tcp()
        return t is not None and part in t.contact_parts

    # --- widgets -------------------------------------------------------------------------------

    def refresh(self) -> None:
        b, t, x = self.b, self._tcp(), self._part()
        tool = b.pkg.kind == "tool"
        self.folder.visible = tool
        if not tool:
            self._rect(False)
            return
        mesh = x is not None and x.shape == "mesh"
        b._busy, busy = True, b._busy
        try:
            self.whole.value = x is not None and self.is_contact(x.name)
            if t is not None:
                from ui.radius_ui import LABELS

                self.min_r.value = float(t.min_radius_mm or 0.0)
                self.applies.value = LABELS[t.radius_applies]
                from ui.travel_ui import LABELS as TRAVEL

                self.travel.value = TRAVEL[t.travel]
                self.scan_w.value = float(t.scan_width_mm or 0.0)
                self.overrun.value = float(t.overrun_mm)
        finally:
            b._busy = busy
        self.whole.visible = x is not None
        for w in (self.mode, self.clear):
            w.visible = mesh
        m = self.mode.value
        self.angle.visible = mesh and m.startswith("Click")
        self.radius.visible = mesh and m.startswith("Brush")
        self._rect(mesh and m.startswith("Box"))
        self.info.content = summary(t) if t is not None else ""

    def _radius(self) -> None:
        from ui.radius_ui import LABELS

        r = float(self.min_r.value)
        kind = next(a for a, lab in LABELS.items() if lab == self.applies.value)
        self._set_tcp(min_radius_mm=r if r > 0 else None, radius_applies=kind)

    def _travel(self) -> None:
        from ui.travel_ui import LABELS as TRAVEL

        kind = next(k for k, lab in TRAVEL.items() if lab == self.travel.value)
        w = float(self.scan_w.value)
        self._set_tcp(travel=kind, scan_width_mm=w if w > 0 else None, overrun_mm=float(self.overrun.value))

    def _mode_changed(self) -> None:
        self.refresh()
        self.b.draw()  # clicks are registered on the selected part's mesh while picking

    def _rect(self, on: bool) -> None:
        if on and self._rect_cb is None:
            self._rect_cb = self.server.scene.on_rect_select()(self.on_rect)  # a decorator factory
        elif not on and self._rect_cb is not None:
            self.server.scene.remove_rect_select_callback(self._rect_cb)
            self._rect_cb = None

    # --- picking -------------------------------------------------------------------------------

    def picking(self) -> bool:
        x = self._part()
        return self.b.pkg.kind == "tool" and x is not None and x.shape == "mesh" and self.mode.value != "Off"

    def _T_world_part(self) -> np.ndarray:
        return self.b.root_pose() @ pose_to_matrix(self._part().pose)

    def click(self, mesh: trimesh.Trimesh, origin, direction) -> int:
        """A click ray (world) on the selected part: pick by the current mode; faces changed."""
        from core.mesh_select import grow_region, pick_face

        T = np.linalg.inv(self._T_world_part())
        o = T[:3, :3] @ np.asarray(origin, float) + T[:3, 3]
        d = T[:3, :3] @ np.asarray(direction, float)
        f = pick_face(mesh, o, d)
        if f is None:
            return 0
        m = self.mode.value
        if m.startswith("Brush"):
            from core.target_tools import brush_faces

            hit = o + d / np.linalg.norm(d) * float(
                np.dot(mesh.triangles_center[f] - o, d / np.linalg.norm(d))
            )
            ids = brush_faces(mesh, f, hit, float(self.radius.value) / 1000)
        else:
            ids = grow_region(mesh, f, float(self.angle.value))
        self.change_faces(self._part().name, ids, "+" in m)
        return len(ids)

    def on_rect(self, event) -> None:
        if not self.picking() or not self.mode.value.startswith("Box"):
            return
        from core import targets as tg
        from core.carrier import primitive_mesh

        mesh = primitive_mesh(self.b.primitive(self._part()))
        cam = event.client.camera
        T_cam_part = np.linalg.inv(make_T(wxyz_to_matrix(cam.wxyz), cam.position)) @ self._T_world_part()
        ids = tg.select_in_screen_rect(
            mesh, T_cam_part, cam.fov, cam.aspect, event.screen_min, event.screen_max
        )
        self.change_faces(self._part().name, ids, "+" in self.mode.value)

    # --- preview (ui.package_preview) ----------------------------------------------------------

    def colour(self, part, default):
        return GREEN if self.b.pkg.kind == "tool" and self.is_contact(part.name) else default

    def decorate(self, k: int, part, mesh: trimesh.Trimesh, handle, root: str) -> list:
        """Picked faces drawn green over part ``k``; the selected part takes clicks while picking."""
        out = []
        if self.b.pkg.kind != "tool" or part.shape != "mesh":
            return out
        T = pose_to_matrix(part.pose)
        ids = self.faces(part.name)
        ids = ids[ids < len(mesh.faces)]
        if len(ids):
            sub = mesh.submesh([ids], append=True)
            v = sub.vertices + sub.vertex_normals * LIFT
            out.append(
                self.server.scene.add_mesh_simple(
                    f"{root}/part_{k}_contact",
                    np.asarray(v, np.float32),
                    np.asarray(sub.faces, np.uint32),
                    color=GREEN,
                    side="double",
                    wxyz=matrix_to_wxyz(T[:3, :3]),
                    position=tuple(T[:3, 3]),
                )
            )
        if k == self.b.sel and self.picking() and not self.mode.value.startswith("Box"):
            handle.on_click(lambda e: self.click(mesh, e.ray_origin, e.ray_direction))
        return out


def summary(t) -> str:
    """'TCP **pad**: may touch the parts pad; 312 faces of body' (or nothing picked)."""
    bits = []
    if t.contact_parts:
        bits.append(
            "the part"
            + ("s " if len(t.contact_parts) > 1 else " ")
            + ", ".join(f"**{n}**" for n in t.contact_parts)
        )
    bits += [f"{len(v)} faces of **{n}**" for n, v in t.contact_faces.items()]
    if not bits:
        return f"_TCP **{t.name}**: nothing may touch the part (every tool surface collides)._"
    return f"TCP **{t.name}** may touch with: " + "; ".join(bits) + "."
