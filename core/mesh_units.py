"""Importing meshes (v10 T20): guess a mesh's units from its size, and mesh STEP files in a separate
process so the import can be cancelled.

``guess_units(extent)``: STL / OBJ / PLY carry no units. The largest bounding-box side in file units
tells: over ``MM_ABOVE`` it is millimetres (a 20 m end effector or cell part is not plausible), from
``M_FROM`` to ``M_UP_TO`` it is metres; in between (5 to 20) it could be a 5-20 m frame in metres
or a 5-20 mm piece in millimetres: unsure, the caller asks. Below ``M_FROM`` (5 mm in metres or
0.005 mm in mm): unsure too.

``step_in_process``: CadQuery meshes a STEP file in seconds to minutes and cannot be interrupted in a
thread; a spawned process can be ended. The result comes back as an STL file in metres.
"""

from __future__ import annotations

import multiprocessing as mp
import time
from collections.abc import Callable
from pathlib import Path

MM_ABOVE = 20.0
M_FROM, M_UP_TO = 0.005, 5.0
SCALE = {"m": 1.0, "mm": 1e-3, "in": 0.0254}


def guess_units(extent: float) -> tuple[str, bool]:
    """(units, sure) for a mesh whose largest side is ``extent`` in file units."""
    e = float(extent)
    if e > MM_ABOVE:
        return "mm", True
    if M_FROM <= e <= M_UP_TO:
        return "m", True
    return ("m" if e > M_UP_TO else "mm"), False


def describe(extent: float, units: str) -> str:
    """'350 mm long' (the mesh's largest side in the chosen units, shown in mm)."""
    return f"{extent * SCALE[units] * 1000:.0f} mm long"


def _step_worker(src: str, dst: str) -> None:  # pragma: no cover - runs in the spawned process
    try:
        from core.tool_import import step_mesh

        step_mesh(Path(src)).export(dst)
    except Exception as err:
        Path(dst + ".err").write_text(f"{type(err).__name__}: {err}", encoding="utf-8")
        raise SystemExit(1) from None


class ImportCancelledError(Exception):
    pass


def step_in_process(src: Path, dst: Path, cancelled: Callable[[], bool], poll: float = 0.2) -> None:
    """Mesh STEP ``src`` into the STL ``dst`` (metres) in a spawned process; ``cancelled()`` ends it
    (ImportCancelledError). A failure in the process raises RuntimeError with its message."""
    err = Path(str(dst) + ".err")
    err.unlink(missing_ok=True)
    p = mp.get_context("spawn").Process(target=_step_worker, args=(str(src), str(dst)), daemon=True)
    p.start()
    try:
        while p.is_alive():
            if cancelled():
                p.terminate()
                p.join(5)
                raise ImportCancelledError()
            p.join(poll)
            time.sleep(0)
    finally:
        if p.is_alive():
            p.kill()
    if p.exitcode != 0 or not dst.is_file():
        msg = err.read_text(encoding="utf-8") if err.is_file() else f"the STEP process ended ({p.exitcode})"
        raise RuntimeError(msg)
