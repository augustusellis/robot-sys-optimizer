"""Robot display in Viser: URDF meshes under a base frame, driven by joint values.

Scene tree:  <parent> (base_link pose in world)
               └─ <parent>/urdf_root (URDF root link, offset so base_link lands on <parent>)
                    └─ ViserUrdf link frames ...
"""

from __future__ import annotations

import os
import warnings
from functools import partial
from pathlib import Path

import numpy as np
import pinocchio as pin
import viser
import yourdfpy
from PIL import Image
from viser.extras import ViserUrdf

from core.robot import Robot
from core.transforms import make_T, matrix_to_wxyz, wxyz_to_matrix


def _handle_T(h) -> np.ndarray:
    return make_T(wxyz_to_matrix(h.wxyz), h.position)


# v13 #2: largest texture side sent to the browser (the tests set 128: they never look at it)
TEXTURE_PX = int(os.environ.get("RSO_TEXTURE_PX", "1024"))
_SMALL: dict[tuple, Image.Image] = {}  # (URDF file, mtime, mesh, slot) -> downscaled texture


def light_urdf(path: Path) -> yourdfpy.URDF:
    """The URDF with its visual meshes, textures above ``TEXTURE_PX`` downscaled. Viser sends each
    mesh as a GLB and PNG-encodes its texture every time: the UR meshes each carry a 2048 x 2048
    baked atlas (logo, shading), which made an app start ~10 s of PNG encoding (v13 #2). At
    1024 px it is a quarter of that and still sharp at cell scale; the small copies are made once
    per file. Vertex colours were tried and dropped: the atlas has detail that vertices lose."""
    path = Path(path)
    urdf = yourdfpy.URDF.load(
        path,
        build_scene_graph=True,
        load_meshes=True,
        filename_handler=partial(yourdfpy.filename_handler_magic, dir=path.parent),
    )
    if urdf.scene is None:
        return urdf
    key = (str(path.resolve()), path.stat().st_mtime)
    for name, g in urdf.scene.geometry.items():
        mat = getattr(g.visual, "material", None) if getattr(g.visual, "kind", None) == "texture" else None
        for slot in ("baseColorTexture", "image"):
            im = getattr(mat, slot, None)
            if not isinstance(im, Image.Image) or max(im.size) <= TEXTURE_PX:
                continue
            k = (*key, name, slot)
            if k not in _SMALL:
                f = TEXTURE_PX / max(im.size)
                size = (max(1, round(im.width * f)), max(1, round(im.height * f)))
                _SMALL[k] = im.resize(size, Image.LANCZOS)
            setattr(mat, slot, _SMALL[k])
    return urdf


class RobotView:
    def __init__(self, server: viser.ViserServer, robot: Robot, urdf_path: Path, parent: str):
        self.robot = robot
        self.parent = parent
        # URDF root -> base_link offset (identity for the UR files)
        pin.framesForwardKinematics(robot.model, robot.data, pin.neutral(robot.model))
        T_root_base = np.asarray(robot.data.oMf[robot.base_frame_id].homogeneous)
        T_base_root = np.linalg.inv(T_root_base)
        self.root = server.scene.add_frame(
            f"{parent}/urdf_root",
            show_axes=False,
            wxyz=matrix_to_wxyz(T_base_root[:3, :3]),
            position=T_base_root[:3, 3],
        )
        self.urdf = ViserUrdf(server, light_urdf(urdf_path), root_node_name=f"{parent}/urdf_root")
        names = list(self.urdf.get_actuated_joint_names())
        missing = set(robot.joint_names) - set(names)
        if missing:
            raise ValueError(f"viewer and model disagree on joints: {sorted(missing)}")
        self._order = [names.index(n) for n in robot.joint_names]  # pinocchio -> viser order
        self._viser_names = names
        n_joints = len(self.urdf._urdf.joint_map)
        self._link_frame = {
            j.child: (j.parent, h)
            for j, h in zip(
                self.urdf._joint_map_values[:n_joints], self.urdf._joint_frames[:n_joints], strict=True
            )
        }

    def joint_frame(self, joint: str):
        """The scene frame of URDF joint ``joint`` (its child link's frame, moves with q)."""
        n_joints = len(self.urdf._urdf.joint_map)
        for j, h in zip(
            self.urdf._joint_map_values[:n_joints], self.urdf._joint_frames[:n_joints], strict=True
        ):
            if j.name == joint:
                return h
        return None

    def set_q(self, q: np.ndarray) -> None:
        cfg = np.zeros(len(self._viser_names))
        cfg[self._order] = q
        self.urdf.update_cfg(cfg)

    def link_pose_in_parent(self, link: str) -> np.ndarray:
        """Pose of ``link`` in the <parent> frame, composed from the scene handles."""
        T = np.eye(4)
        while link in self._link_frame:
            parent, h = self._link_frame[link]
            T = _handle_T(h) @ T
            link = parent
        return _handle_T(self.root) @ T

    def remove(self) -> None:
        quiet_remove(self.urdf)
        quiet_remove(self.root)


def quiet_remove(handle) -> None:
    """Remove a scene handle without Viser's "Attempted to remove already removed node"
    warnings: ViserUrdf removes child nodes after their parent frame is already gone,
    which is harmless but floods the console on every project load."""
    with warnings.catch_warnings():
        warnings.filterwarnings("ignore", message="Attempted to remove already removed node")
        handle.remove()
