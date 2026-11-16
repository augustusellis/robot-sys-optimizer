"""Dense reach check where a sparse plan can hide gaps (v10 task 10).

The planner checks the target points exactly, but between them it assumes the arm reaches too. That
fails first where something changes: between two stops' areas, between two joint-configuration
zones, at the part's edges and creases, and around points that were only just reached. After
planning, ``edge_points`` puts verification points there (part frame):

  stops    along the boundary between two stops' areas (neighbouring targets of different stops)
  zones    along the boundary between two configuration zones of one stop
  edge     along the edges of the targeted surface and the creases inside it (one point per face
           side of a crease), within reach of the target points
  margin   a ring around each low-margin target (marginal, or a joint within ``LOW_MARGIN_DEG``)

``check_edges`` checks each one exactly from the stops of its nearest targets (zone points: in
either zone's configuration) and marks what no stop reaches: the gaps. Work points inside zones stay
sparse; *Densify edges* (ui) adds the gaps as targets. Pure geometry + core.feasibility: no viser.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field

import numpy as np
import trimesh
from scipy.spatial import cKDTree

KINDS = {"stops": "between stops", "zones": "between zones", "edge": "part edge", "margin": "low margin"}
CREASE_DEG = 30.0  # dihedral angle of a crease inside the targeted surface
LOW_MARGIN_DEG = 15.0
SAME_SIDE = 0.5  # neighbours whose normals differ by more than 60 deg get no points between them
MAX_POINTS = 3000


@dataclass
class EdgePoint:
    p: np.ndarray  # part frame
    n: np.ndarray
    kind: str  # KINDS
    near: int  # nearest target point id (its group and stop)
    stops: list[int]  # stops it is checked from
    configs: tuple[str, ...] = ()  # zone points: the configurations of the two zones
    reached: bool = False
    reason: str = ""
    stop: int | None = None  # the stop that reaches it
    notes: list[str] = field(default_factory=list)


def spacing_of(P: np.ndarray) -> float:
    """Median distance from each target to its nearest neighbour."""
    if len(P) < 2:
        return 0.05
    d, _ = cKDTree(P).query(P, k=2)
    return float(np.median(d[:, 1]))


def _on_mesh(mesh: trimesh.Trimesh, p: np.ndarray) -> np.ndarray:
    on, _, _ = trimesh.proximity.closest_point(mesh, np.atleast_2d(p))
    return on


def _unit(v: np.ndarray) -> np.ndarray:
    return v / np.maximum(np.linalg.norm(v, axis=-1, keepdims=True), 1e-12)


def edge_points(
    mesh: trimesh.Trimesh,
    ids: np.ndarray,
    P: np.ndarray,
    N: np.ndarray,
    stop_of: dict[int, int],
    zone_of: dict[int, tuple[int, str]] | None = None,
    low: set[int] | None = None,
    spacing: float | None = None,
) -> list[EdgePoint]:
    """Verification points of a plan (see the module docstring). ``stop_of``: point id -> stop;
    ``zone_of``: point id -> (zone index, configuration); ``low``: low-margin point ids."""
    ids = np.asarray(ids)
    P, N = np.asarray(P, float), _unit(np.asarray(N, float))
    h = spacing or spacing_of(P)
    tree = cKDTree(P)
    idx = {int(pid): i for i, pid in enumerate(ids)}
    out: list[EdgePoint] = []
    zone_of = zone_of or {}

    def stops_near(p: np.ndarray, k: int = 3) -> tuple[int, list[int]]:
        _, nn = tree.query(p, k=min(k, len(P)))
        nn = np.atleast_1d(nn)
        sts = sorted({stop_of[int(ids[j])] for j in nn if int(ids[j]) in stop_of})
        return int(ids[nn[0]]), sts

    # boundaries between stops' areas and between zones: samples between neighbouring targets
    pairs = tree.query_pairs(1.6 * h, output_type="ndarray")
    for i, j in pairs:
        a, b = int(ids[i]), int(ids[j])
        if a not in stop_of or b not in stop_of:
            continue
        if float(N[i] @ N[j]) < SAME_SIDE:  # e.g. two faces of a thin wall: no surface between them
            continue
        sa, sb = stop_of[a], stop_of[b]
        za, zb = zone_of.get(a), zone_of.get(b)
        if sa != sb:
            kind, sts, cfg = "stops", sorted({sa, sb}), ()
        elif za is not None and zb is not None and za[0] != zb[0]:
            kind, sts, cfg = "zones", [sa], (za[1], zb[1])
        else:
            continue
        d = float(np.linalg.norm(P[j] - P[i]))
        ts = (0.5,) if d <= 1.1 * h else (0.25, 0.5, 0.75)
        for t in ts:
            p = _on_mesh(mesh, P[i] + t * (P[j] - P[i]))[0]
            n = _unit(N[i] + t * (N[j] - N[i]))
            out.append(EdgePoint(p, n, kind, a if t <= 0.5 else b, sts, cfg))
    # edges and creases of the targeted surface
    cen = mesh.triangles_center
    dist, near = tree.query(cen)
    fn = mesh.face_normals
    on = (dist < 0.75 * h) & (np.einsum("ij,ij->i", fn, N[near]) > 0.5)
    if on.any():
        adj, ang = mesh.face_adjacency, mesh.face_adjacency_angles
        edges_v = mesh.face_adjacency_edges
        both = on[adj[:, 0]] & on[adj[:, 1]]
        crease = both & (ang > np.radians(CREASE_DEG))
        lines: list[tuple[np.ndarray, np.ndarray, list[int]]] = []
        for e in np.flatnonzero(crease):
            lines.append((mesh.vertices[edges_v[e, 0]], mesh.vertices[edges_v[e, 1]], list(adj[e])))
        sel = np.flatnonzero(on)
        sub = mesh.faces[sel]
        es = np.sort(np.concatenate([sub[:, [0, 1]], sub[:, [1, 2]], sub[:, [2, 0]]]), axis=1)
        face_of = np.tile(sel, 3)
        uniq, inv, cnt = np.unique(es, axis=0, return_inverse=True, return_counts=True)
        for u in np.flatnonzero(cnt == 1):  # boundary of the targeted surface
            f = int(face_of[np.flatnonzero(inv.ravel() == u)[0]])
            lines.append((mesh.vertices[uniq[u, 0]], mesh.vertices[uniq[u, 1]], [f]))
        for v0, v1, faces in lines:
            L = float(np.linalg.norm(v1 - v0))
            m = max(1, int(np.ceil(L / (0.5 * h))))
            for s in (np.arange(m) + 0.5) / m:
                p = v0 + s * (v1 - v0)
                if tree.query(p)[0] > 0.75 * h:
                    continue
                pid, sts = stops_near(p)
                for f in faces:
                    out.append(EdgePoint(p.copy(), fn[f].copy(), "edge", pid, sts))
    # around low-margin points
    for pid in sorted(low or ()):
        if pid not in idx or pid not in stop_of:
            continue
        p, n = P[idx[pid]], N[idx[pid]]
        u = _unit(np.cross(n, [1.0, 0, 0] if abs(n[0]) < 0.9 else [0, 1.0, 0]))
        v = np.cross(n, u)
        for a in np.radians(np.arange(0, 360, 60)):
            q = _on_mesh(mesh, p + 0.5 * h * (np.cos(a) * u + np.sin(a) * v))[0]
            out.append(EdgePoint(q, n.copy(), "margin", pid, [stop_of[pid]]))
    return _thin(out, 0.25 * h)[:MAX_POINTS]


def two_sided(
    ids: np.ndarray, P: np.ndarray, N: np.ndarray, group_of: dict[int, str], spacing: float | None = None
) -> list[str]:
    """Target groups with neighbouring targets on opposite sides of a sheet (v12 #5): pairs within
    the edge check's 1.6 x spacing whose normals differ by more than ``SAME_SIDE`` allows. The edge
    check puts no points between those (by design), so it says so instead of passing silently."""
    if len(P) < 2:
        return []
    P, N = np.asarray(P, float), _unit(np.asarray(N, float))
    h = spacing or spacing_of(P)
    out: set[str] = set()
    for i, j in cKDTree(P).query_pairs(1.6 * h, output_type="ndarray"):
        ga, gb = group_of.get(int(ids[i])), group_of.get(int(ids[j]))
        if ga is not None and ga == gb and float(N[i] @ N[j]) < SAME_SIDE:
            out.add(ga)
    return sorted(out)


def _thin(pts: list[EdgePoint], tol: float) -> list[EdgePoint]:
    """Drop near-duplicates (same place, same normal) - crease ends, pairs sharing a midpoint."""
    if not pts:
        return pts
    X = np.array([np.concatenate([e.p, 0.2 * e.n]) for e in pts])  # crease sides stay apart
    keep, seen = [], cKDTree(X)
    gone = np.zeros(len(pts), bool)
    for i in range(len(pts)):
        if gone[i]:
            continue
        keep.append(pts[i])
        for j in seen.query_ball_point(X[i], tol):
            if j != i:
                gone[j] = True
    return keep


def check_edges(
    checker,
    pts: list[EdgePoint],
    base_poses: list[np.ndarray],
    group_of: Callable[[int], object],
    cancelled: Callable[[], bool] | None = None,
    progress: Callable[[float], None] | None = None,
    pool=None,
    chunk: int = 200,
) -> list[EdgePoint]:
    """Exact checks of the verification points, from their stops (round r = each point's r-th
    stop, only for points not reached yet); in ``pool`` when given. Returns the gaps (unreached).
    ``group_of(point id)``: the TargetGroup whose tolerances apply (the nearest target's)."""
    from core.feasibility import CancelledError
    from opt.tolerance import run_calls

    rounds = max((len(e.stops) for e in pts), default=0)
    total = sum(len(e.stops) for e in pts) or 1
    done = 0
    for r in range(rounds):
        todo = [i for i, e in enumerate(pts) if not e.reached and len(e.stops) > r]
        for c0 in range(0, len(todo), chunk):
            if cancelled is not None and cancelled():
                raise CancelledError()
            part = todo[c0 : c0 + chunk]
            calls = []
            for i in part:
                e = pts[i]
                kind = "configs" if e.kind == "zones" else "point"
                mp = 40 if kind == "configs" else None
                calls.append((base_poses[e.stops[r]], kind, -(i + 1), e.p, e.n, group_of(e.near), mp, False))
            for i, res in zip(part, run_calls(checker, calls, pool, cancelled), strict=True):
                e, k = pts[i], pts[i].stops[r]
                if e.kind == "zones":
                    if any(c in res for c in e.configs):
                        e.reached, e.stop = True, k
                    elif not e.reason:
                        e.reason = "other configuration" if res else "no_ik"
                elif res.status != "unreachable":
                    e.reached, e.stop = True, k
                else:
                    e.reason, e.notes = res.reason, list(res.notes)
            done += len(part)
            if progress is not None:
                progress(min(1.0, done / total))
    for e in pts:
        if e.reached:
            e.reason, e.notes = "", []
    return [e for e in pts if not e.reached]


def plan_maps(sol, results: dict) -> tuple[dict[int, int], dict[int, tuple[int, str]], set[int]]:
    """(stop_of, zone_of, low) of a plan: reached points' stops, their zones (index, configuration)
    and the low-margin ones (status marginal or a joint within ``LOW_MARGIN_DEG`` of its limit)."""
    ok = {int(p): r for p, r in results.items() if r.status != "unreachable"}
    stop_of = {int(p): int(k) for p, k in sol.assignment.items() if int(p) in ok}
    zone_of = {int(p): (zi, z.config) for zi, z in enumerate(sol.zones) for p in z.points if int(p) in ok}
    low = {p for p, r in ok.items() if r.status == "marginal" or r.limit_margin_deg < LOW_MARGIN_DEG}
    return stop_of, zone_of, low


@dataclass
class EdgeResult:
    """One edge check of a plan (``Session.edge_check``, not saved)."""

    label: str
    points: list[EdgePoint]
    gaps: list[EdgePoint]
    spacing: float

    def summary(self) -> str:
        return summary(self.points, self.gaps)


def summary(pts: list[EdgePoint], gaps: list[EdgePoint]) -> str:
    """'12 of 240 edge points not reached: 8 between stops, 4 part edge' (or all reached)."""
    if not gaps:
        return f"all {len(pts)} edge points reached"
    by: dict[str, int] = {}
    for e in gaps:
        by[KINDS[e.kind]] = by.get(KINDS[e.kind], 0) + 1
    parts = ", ".join(f"{v} {k}" for k, v in sorted(by.items(), key=lambda kv: -kv[1]))
    return f"{len(gaps)} of {len(pts)} edge points not reached: {parts}"
