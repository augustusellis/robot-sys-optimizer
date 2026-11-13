"""Parallel exact checks (v8 T2).

A ``CheckPool`` is a process pool whose workers each build their own ``FeasibilityChecker``
once from a picklable ``CheckerSpec`` (Pinocchio / Coal objects don't pickle). The planner
code stays serial: before a loop of checks it calls ``prefetch(checker, calls)``, which runs
the calls the plan's ``CheckMemo`` doesn't have yet in the pool and stores them in the memo;
the serial loop then reads them from the memo. Results are identical to the serial path
(same inputs, same deterministic checker).

``calls`` are ``(T_world_base, kind, pid, p, n, group, max_poses, clearance)`` with kind
"point" (``check_point``) or "configs" (``check_point_configs``; clearance ignored), and an
optional 9th entry, the part pose for that call (tolerance samples, ``opt.tolerance``).

Workers are kept between runs (v10 #7): one process pool (``_executor``, per worker count) lives
as long as the process that made it, and a ``CheckPool`` is a light handle for one setup. Its spec
is pickled once to a file; each task carries the setup's key and that file, and a worker whose
current setup has another key re-targets (loads the spec, builds that checker) before it runs the
calls. Compare (one setup per robot) and Gantry placement (one per frame) therefore start the
processes once instead of once per robot / frame (a Windows spawn is ~3 s per worker). Workers end
themselves when the process that made the pool ends (watchdog on the parent).
"""

from __future__ import annotations

import hashlib
import multiprocessing as mp
import os
import pickle
import tempfile
import threading
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

MIN_CALLS = 24  # fewer calls than this: not worth the round trip
CHUNK = 8  # calls per task


def default_workers() -> int:
    """Default pool size: CPU count - 1, at most 4 (each worker holds a robot + scene)."""
    return max(1, min(4, (os.cpu_count() or 2) - 1))


@dataclass
class CheckerSpec:
    """Everything a worker needs to build the planner's checker."""

    robot_spec: object  # core.robot.RobotSpec
    tool: object  # core.tool.ToolModel
    mesh: object  # trimesh.Trimesh (part)
    T_world_base: np.ndarray
    T_world_part: np.ndarray
    base_bodies: list = field(default_factory=list)
    world_bodies: list = field(default_factory=list)
    obstacles: list = field(default_factory=list)
    dress: float = 0.0  # v8 T18: core.collision.DRESS_M of the parent
    hose: object = None  # core.collision.HOSE of the parent (core.hose.Hose)
    geom: object = None  # core.collision.geom_state() of the parent (collision geometry)

    def build(self):
        from core import collision
        from core.collision import CollisionScene
        from core.feasibility import FeasibilityChecker
        from core.ik import make_ik
        from core.robot import Robot

        collision.DRESS_M = self.dress
        collision.HOSE = self.hose
        collision.set_geom_state(self.geom)
        robot = Robot(self.robot_spec)
        scene = CollisionScene(
            robot,
            self.tool,
            self.mesh,
            self.T_world_base,
            self.T_world_part,
            base_bodies=self.base_bodies,
            world_bodies=self.world_bodies,
            obstacles=self.obstacles,
        )
        return FeasibilityChecker(
            robot, make_ik(robot), self.tool, scene, self.T_world_base, self.T_world_part
        )

    def key(self) -> str:
        return hashlib.sha1(pickle.dumps(self, protocol=5)).hexdigest()


_CHECKER = None  # in a pool worker: the checker of the current setup
_KEY = None  # its key


def _watch_parent() -> None:  # pragma: no cover - runs in pool workers
    parent = mp.parent_process()
    if parent is not None:
        parent.join()
        os._exit(0)


def _init() -> None:  # pragma: no cover - runs in pool workers
    threading.Thread(target=_watch_parent, daemon=True).start()
    import core.collision  # noqa: F401  (warm imports: the first re-target is faster)
    import core.feasibility  # noqa: F401


def _target(key: str, path: str):  # pragma: no cover - runs in pool workers
    """This worker's checker for setup ``key`` (built from the spec file when it changed)."""
    global _CHECKER, _KEY
    if key != _KEY:
        with open(path, "rb") as f:
            spec = pickle.load(f)
        _CHECKER, _KEY = None, None  # the old setup's robot / scene go first (memory)
        _CHECKER, _KEY = spec.build(), key
    return _CHECKER


def _run(key: str, path: str, settings, calls: list) -> list:  # pragma: no cover - in pool workers
    from opt.tolerance import set_part

    c = _target(key, path)
    c.settings = settings
    T_part0 = getattr(c, "_T_part0", None)
    if T_part0 is None:
        T_part0 = c._T_part0 = c.T_world_part.copy()
    out = []
    for call in calls:  # an optional 9th entry moves the part (v8 T5 tolerance samples)
        T, kind, pid, p, n, group, max_poses, clearance = call[:8]
        set_part(c, call[8] if len(call) > 8 and call[8] is not None else T_part0)
        c.set_base(T)
        if kind == "point":
            out.append(c._check_point(pid, p, n, group, max_poses, clearance))
        else:
            out.append(c._check_point_configs(pid, p, n, group, max_poses))
    return out


def _noop() -> None:  # pragma: no cover - runs in pool workers
    return None


def warm(workers: int) -> None:
    """Start the pool's processes in the background (a spawn blocks ~3 s per worker): the caller
    goes on with work that needs no pool (set cover, frame scoring) meanwhile."""
    if workers > 1:
        threading.Thread(target=lambda: _executor(workers).submit(_noop), daemon=True).start()


_EXECUTORS: dict[int, ProcessPoolExecutor] = {}
_LOCK = threading.Lock()
SPEC_DIR = Path(tempfile.gettempdir()) / f"rso_check_specs_{os.getpid()}"


def _executor(workers: int) -> ProcessPoolExecutor:
    """The persistent process pool with ``workers`` processes (one at a time: each worker holds a
    robot + scene). A pool that broke (a worker died) is replaced."""
    with _LOCK:
        ex = _EXECUTORS.get(workers)
        if ex is not None and not getattr(ex, "_broken", False):
            return ex
        for old in _EXECUTORS.values():
            old.shutdown(wait=False, cancel_futures=True)
        _EXECUTORS.clear()
        ex = _EXECUTORS[workers] = ProcessPoolExecutor(
            max_workers=workers, mp_context=mp.get_context("spawn"), initializer=_init
        )
        return ex


class CheckPool:
    """The check pool for one setup (a handle on the shared processes, see the module docstring)."""

    def __init__(self, spec: CheckerSpec, workers: int):
        self.workers = int(workers)
        self.key = spec.key()
        SPEC_DIR.mkdir(parents=True, exist_ok=True)
        self.path = str(SPEC_DIR / f"{self.key}.pkl")
        if not os.path.exists(self.path):
            tmp = f"{self.path}.{threading.get_ident()}.tmp"
            with open(tmp, "wb") as f:
                pickle.dump(spec, f, protocol=5)
            os.replace(tmp, self.path)
        self._ex = _executor(self.workers)

    def run(self, settings, calls: list, cancelled=None) -> list:
        """Results of ``calls`` in order (``cancelled`` is polled between chunks)."""
        if getattr(self._ex, "_broken", False):
            self._ex = _executor(self.workers)
        chunks = [calls[i : i + CHUNK] for i in range(0, len(calls), CHUNK)]
        futs = [self._ex.submit(_run, self.key, self.path, settings, ch) for ch in chunks]
        out = []
        for f in futs:
            if cancelled is not None and cancelled():
                for g in futs:
                    g.cancel()
                from core.feasibility import CancelledError

                raise CancelledError()
            out += f.result()
        return out

    def shutdown(self) -> None:
        """Forget this setup (the processes stay for the next one: ``shutdown_all`` ends them)."""
        try:
            os.remove(self.path)
        except OSError:
            pass


_POOLS: dict[str, CheckPool] = {}


def pool_for(spec: CheckerSpec, workers: int) -> CheckPool | None:
    """The pool for this setup (re-targets the shared workers), or None for ``workers <= 1``."""
    if workers <= 1:
        return None
    key = spec.key()
    p = _POOLS.get(key)
    if p is not None and p.workers == workers and os.path.exists(p.path):
        return p
    for old in _POOLS.values():  # one setup at a time (its spec file)
        old.shutdown()
    _POOLS.clear()
    p = _POOLS[key] = CheckPool(spec, workers)
    warm(workers)  # v10 #7: the processes start while the planner's first steps run
    return p


def shutdown_all() -> None:
    for p in _POOLS.values():
        p.shutdown()
    _POOLS.clear()
    with _LOCK:
        for ex in _EXECUTORS.values():
            ex.shutdown(wait=False, cancel_futures=True)
        _EXECUTORS.clear()


def prefetch(checker, calls: list) -> None:
    """Fill the checker's memo with ``calls`` computed in its pool (no-op without a pool or
    a memo, or for a handful of calls)."""
    pool, memo = getattr(checker, "pool", None), checker.memo
    if pool is None or memo is None or len(calls) < MIN_CALLS:
        return
    todo, keys, seen = [], [], set()
    for c in calls:
        T, kind, pid, _p, _n, _g, max_poses, clearance = c
        call = ("point", pid, max_poses, clearance) if kind == "point" else ("configs", pid, max_poses)
        k = (memo.key(T), *call)
        if k in memo.data or k in seen:
            continue
        seen.add(k)
        todo.append(c)
        keys.append(k)
    if len(todo) < MIN_CALLS:
        return
    for k, r in zip(keys, pool.run(checker.settings, todo, checker.cancelled), strict=True):
        memo.data[k] = r
