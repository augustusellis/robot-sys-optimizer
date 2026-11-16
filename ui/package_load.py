"""Package builder -> Parts -> *Load relative to* (v12 #27): which coordinate system of an imported
end-effector file goes on the robot flange.

A tool package's frame is the robot flange (``tool0`` / ``fanuc_flange``: z out of the flange face).

  Robot flange            (default) the file's own origin and axes are the flange
  STEP coordinate system  a named coordinate system of the STEP file (core.step_datums) is the
                          mounting face: the part is placed so that it sits on the flange.
                          One in the file: it is used; several: a dialog asks which (or the
                          file origin); none: the flange, with a note. STL / OBJ / PLY files
                          have none (the flange is used).

Other package kinds (parts, carriers) keep the package frame and do not show the choice. The
coordinate systems stay listed under *Frames and travel -> Frame from STEP*.
"""

from __future__ import annotations

import numpy as np

from core.transforms import matrix_to_pose, pose_to_matrix

FLANGE, STEP_CSYS = "Robot flange", "STEP coordinate system"
CHOICES = (FLANGE, STEP_CSYS)
FILE_ORIGIN = "None: the file origin is the flange"


def dropdown(gui):
    return gui.add_dropdown(
        "Load relative to",
        CHOICES,
        hint="end effectors: which coordinate system of the file goes on the robot flange",
    )


def place(b, part_name: str, datum: str) -> str:
    """Put STEP coordinate system ``datum`` of part ``part_name`` on the flange (package frame)."""
    parts = list(b.pkg.parts)
    k = next(i for i, x in enumerate(parts) if x.name == part_name)
    D = pose_to_matrix(parts[k].datums[datum])
    parts[k] = parts[k].model_copy(update={"pose": matrix_to_pose(np.linalg.inv(D))})
    b.pkg = b.pkg.model_copy(update={"parts": parts})
    b.load_widgets()
    b.draw()
    return f"STEP coordinate system **{datum}** on the robot flange"


def after_import(b, e, part, step: bool) -> str:
    """Apply the *Load relative to* choice to a just-added part; a note for the status line."""
    if b.pkg.kind != "tool" or b.load_from.value != STEP_CSYS:
        return "file origin on the robot flange" if b.pkg.kind == "tool" else ""
    if not step:
        return "an STL / OBJ / PLY file has no coordinate systems: its origin is on the robot flange"
    names = sorted(part.datums)
    if not names:
        return "no named coordinate systems in the STEP file: its origin is on the robot flange"
    if len(names) == 1:
        return place(b, part.name, names[0])
    ask(b, e, part.name, names)
    return f"{len(names)} STEP coordinate systems: pick the mounting face in the dialog"


def ask(b, e, part_name: str, names: list[str]) -> None:
    """Dialog: which coordinate system is the mounting face (no browser client: file origin)."""
    client = getattr(e, "client", None)
    if client is None:
        return
    with client.gui.add_modal(f"Mounting face of {part_name}") as modal:
        client.gui.add_markdown("_Which coordinate system of the STEP file goes on the robot flange?_")
        for name in [*names, FILE_ORIGIN]:
            btn = client.gui.add_button(name)

            def pick(_e, name=name) -> None:
                modal.close()
                if name != FILE_ORIGIN:
                    b.import_info.content = f"✔ {place(b, part_name, name)}."

            btn.on_click(pick)
