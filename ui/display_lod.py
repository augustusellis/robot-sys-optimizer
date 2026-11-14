"""Display level of detail and a small cache for the part mesh (R1c).

A part of several hundred thousand triangles is one 9 MB message and stalls the browser. Above
``DISPLAY_FACES`` the viewer draws a simplified copy (``TARGET_FACES``, quadric decimation from
``core.collision_mesh``). Only the drawing changes: planning, collision and surface picking use
the full mesh. Picking casts the click ray (sent by the browser as origin and direction, not as
a face index) against the full mesh, so face ids always refer to the full mesh.
"""

from __future__ import annotations

from collections import OrderedDict
from pathlib import Path

import trimesh

DISPLAY_FACES = 300_000  # above this many triangles the drawing is simplified
TARGET_FACES = 200_000  # about this many triangles in the simplified copy
KEEP = 2  # meshes kept (the current part and the one before it)


def file_key(path: Path, units: str) -> tuple:
    """Cache key: changes when the file is replaced or edited, or the units change."""
    p = Path(path)
    st = p.stat()  # a missing file raises FileNotFoundError, as loading would
    return (str(p.resolve()), units, st.st_size, st.st_mtime_ns)


class MeshCache:
    """The last ``keep`` entries by key (least recently used goes first)."""

    def __init__(self, keep: int = KEEP):
        self.keep = keep
        self._d: OrderedDict = OrderedDict()

    def get(self, key, make):
        if key in self._d:
            self._d.move_to_end(key)
            return self._d[key]
        value = make()
        self._d[key] = value
        while len(self._d) > self.keep:
            self._d.popitem(last=False)
        return value

    def clear(self) -> None:
        self._d.clear()


def display_copy(mesh: trimesh.Trimesh, limit: int = DISPLAY_FACES, target: int = TARGET_FACES):
    """(mesh to draw, True when it is simplified). The mesh itself when it has at most ``limit``
    triangles, or when the simplifier is missing (a convex hull would hide the part)."""
    if len(mesh.faces) <= limit:
        return mesh, False
    from core.collision_mesh import _reduce

    out, how = _reduce(mesh, target)
    if how != "reduce":
        return mesh, False
    return out, True


def note(full_faces: int, shown_faces: int) -> str:
    return (
        f"Display simplified: {full_faces / 1000:,.0f}k → {shown_faces / 1000:,.0f}k triangles "
        "(planning, collision and picking use the full mesh)"
    )
