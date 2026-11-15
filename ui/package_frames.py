"""Package builder -> *Frames from STEP* (v11 task 10): named coordinate systems of imported STEP
parts (core.step_datums: the CAD datum CSYS features) as frames.

A STEP part keeps its datums (``urdf_package.Part.datums``: name -> pose in the part's own frame,
saved in package.yaml). Pick one under **Frame from STEP** and what it defines under **Assign to**:

  TCP (selected)          the selected TCP's pose
  Robot mount             where the robot's base sits (carriers)
  Vertical stage mount    the lift stage's parts move so the first one's origin is on the datum
                          (the robot mount moves along: it rides the stage)
  Package origin          the datum becomes the package frame (the tool's flange, a carrier's
                          floor frame, a part's origin): every part, the mount and the TCPs are
                          re-expressed in it

Limit (core.step_datums): only placements named in the file are read, and in an assembly a
sub-part's own placement is not composed onto its datums.
"""

from __future__ import annotations

import numpy as np

from core.transforms import matrix_to_pose, pose_to_matrix

TARGETS = ("TCP (selected)", "Robot mount", "Vertical stage mount", "Package origin")
NONE = "(no STEP frames)"


def frames(pkg) -> dict[str, np.ndarray]:
    """'part / datum' -> pose in the package frame, for every datum of every part."""
    out = {}
    for x in pkg.parts:
        T_part = pose_to_matrix(x.pose)
        for name, pose in x.datums.items():
            out[f"{x.name} / {name}"] = T_part @ pose_to_matrix(pose)
    return out


def assign(b, label: str, target: str) -> str:
    """Apply frame ``label`` to ``target`` (TARGETS) on builder ``b``'s package; a status line."""
    T = frames(b.pkg).get(label)
    if T is None:
        raise ValueError(f"no STEP frame '{label}'")
    p = b.pkg
    if target == TARGETS[0]:
        if not p.tcps:
            raise ValueError("this package has no TCP")
        k = min(b.sel_tcp, len(p.tcps) - 1)
        tcps = list(p.tcps)
        tcps[k] = tcps[k].model_copy(update={"pose": matrix_to_pose(T)})
        b.pkg = p.model_copy(update={"tcps": tcps})
        what = f"TCP {tcps[k].name}"
    elif target == TARGETS[1]:
        b.pkg = p.model_copy(update={"mount": matrix_to_pose(T)})
        what = "the robot mount"
    elif target == TARGETS[2]:
        lift = [x for x in p.parts if x.stage == "lift"]
        if not lift:
            raise ValueError("no part on the lift stage (Add vertical stage, or set a part's stage to lift)")
        D = T @ np.linalg.inv(pose_to_matrix(lift[0].pose))  # moves the first lift part onto the datum
        parts = [
            x.model_copy(update={"pose": matrix_to_pose(D @ pose_to_matrix(x.pose))})
            if x.stage == "lift"
            else x
            for x in p.parts
        ]
        b.pkg = p.model_copy(update={"parts": parts, "mount": matrix_to_pose(D @ pose_to_matrix(p.mount))})
        what = "the vertical stage (and the robot mount on it)"
    elif target == TARGETS[3]:
        inv = np.linalg.inv(T)

        def re(pose):
            return matrix_to_pose(inv @ pose_to_matrix(pose))

        b.pkg = p.model_copy(
            update={
                "parts": [x.model_copy(update={"pose": re(x.pose)}) for x in p.parts],
                "tcps": [t.model_copy(update={"pose": re(t.pose)}) for t in p.tcps],
                "mount": re(p.mount),
            }
        )
        what = "the package origin"
    else:
        raise ValueError(f"unknown target {target!r}")
    b.load_widgets()
    return f"✔ **{label}** is now {what}."


class FramePicker:
    """The dropdowns and the button (built in the builder's Frames and travel folder)."""

    def __init__(self, gui, b):
        from ui.widgets import notify

        self.b = b
        self.frame = gui.add_dropdown(
            "Frame from STEP", (NONE,), hint="named coordinate systems of STEP parts"
        )
        self.target = gui.add_dropdown("Assign to", TARGETS)
        self.go = gui.add_button("Assign")
        self.info = gui.add_markdown("")

        def go(e) -> None:
            try:
                self.info.content = assign(b, self.frame.value, self.target.value)
            except ValueError as err:
                notify(e, "Frame not assigned", str(err))

        self.go.on_click(go)

    def refresh(self) -> None:
        names = tuple(frames(self.b.pkg)) or (NONE,)
        if tuple(self.frame.options) != names:
            keep = self.frame.value if self.frame.value in names else names[0]
            self.frame.options = names
            self.frame.value = keep
        has = names != (NONE,)
        for w in (self.target, self.go):
            w.visible = has
