"""Coverage patterns that follow the surface (v10 T15), for curved faces where the projected
patterns of core.patterns (concentric / spiral in the plane across the mean normal) mix up rings
on faces that wrap around (a cylinder side, a fillet, a bent panel).

- **Contour-parallel**: passes at equal distance from the group's boundary, measured along the
  surface (Dijkstra on the 3D neighbour graph of the points), one pass per point spacing; each pass
  is a closed loop walked in one direction, the next pass starts next to where the last one began.
- **Spiral (along the surface)**: the same passes joined into one spiral: each pass starts next to
  where the previous one ended.

The boundary is found on the surface too: a point is on it when its neighbours, seen in its own
tangent plane, leave an angular gap over 120 degrees. ``turn_lifts`` (raster patterns): a move from
one row to the next lifts off when it is longer than the tool's footprint diameter (the pad would
drag across surface between rows it does not cover).
"""

from __future__ import annotations

import heapq

import numpy as np

GAP = 2 * np.pi / 3


def _tangent(n: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    a = np.array([1.0, 0, 0]) if abs(n[0]) < 0.9 else np.array([0, 1.0, 0])
    u = np.cross(n, a)
    u /= np.linalg.norm(u)
    return u, np.cross(n, u)


def boundary(P: np.ndarray, N: np.ndarray, nb: list[list[int]]) -> np.ndarray:
    """Points on the boundary of the point set, judged in each point's tangent plane."""
    out = np.zeros(len(P), bool)
    for i, js in enumerate(nb):
        if len(js) < 3:
            out[i] = True
            continue
        u, v = _tangent(N[i] / max(np.linalg.norm(N[i]), 1e-12))
        d = P[js] - P[i]
        ang = np.sort(np.arctan2(d @ v, d @ u))
        gap = max(ang[0] + 2 * np.pi - ang[-1], float(np.diff(ang).max(initial=0.0)))
        out[i] = gap > GAP
    return out


def surface_distance(P: np.ndarray, nb: list[list[int]], sources: np.ndarray) -> np.ndarray:
    """Distance along the neighbour graph (3D edge lengths) from the nearest source point."""
    d = np.full(len(P), np.inf)
    heap = []
    for s in np.flatnonzero(sources):
        d[s] = 0.0
        heap.append((0.0, int(s)))
    heapq.heapify(heap)
    while heap:
        di, i = heapq.heappop(heap)
        if di > d[i]:
            continue
        for j in nb[i]:
            nd = di + float(np.linalg.norm(P[j] - P[i]))
            if nd < d[j]:
                d[j] = nd
                heapq.heappush(heap, (nd, j))
    d[~np.isfinite(d)] = 0.0  # a piece without a boundary point (cannot happen on open sets)
    return d


def _walk(P: np.ndarray, N: np.ndarray, loop: list[int], start: int) -> list[int]:
    """A pass: nearest unvisited point next, ties counter-clockwise about the pass's own normal."""
    n = N[loop].sum(0)
    if np.linalg.norm(n) < 1e-6 * len(loop):  # a pass all around (a cylinder): its own plane
        c = P[loop] - P[loop].mean(0)
        n = np.linalg.svd(c, full_matrices=False)[2][-1]
    u, v = _tangent(n / np.linalg.norm(n))
    c = P[loop].mean(0)
    ang = np.arctan2((P - c) @ v, (P - c) @ u)
    left = [j for j in loop if j != start]
    seq, cur = [start], start
    while left:
        L = np.array(left)
        d = np.linalg.norm(P[L] - P[cur], axis=1)
        near = L[d <= d.min() * 1.05 + 1e-12]
        turn = np.mod(ang[near] - ang[cur] - 1e-9, 2 * np.pi) + 1e-9
        nxt = int(near[np.lexsort((near, turn))[0]])
        left.remove(nxt)
        seq.append(nxt)
        cur = nxt
    return seq


def contour_passes(P, N, nb, spacing: float, inward: bool, start: int, spiral: bool):
    """(order, pass number per visit) for Contour-parallel / Spiral (along the surface)."""
    from core.patterns import split_components, two_opt

    P, N = np.asarray(P, float), np.asarray(N, float)
    d = surface_distance(P, nb, boundary(P, N, nb))
    band = np.floor(d / max(spacing, 1e-9) + 0.5).astype(int)
    levels = sorted(set(band.tolist()), reverse=not inward)
    order, rid = [], []
    anchor = P[start]
    first = None  # where the last pass began (contour-parallel steps in from there)
    for g, lev in enumerate(levels):
        loops = split_components(nb, np.flatnonzero(band == lev).tolist())
        while loops:
            ref = anchor if spiral or first is None else first
            dist = [np.linalg.norm(P[lp] - ref, axis=1).min() for lp in loops]
            loop = loops.pop(int(np.argmin(dist)))
            s0 = loop[int(np.argmin(np.linalg.norm(P[loop] - ref, axis=1)))]
            seq = two_opt(P, _walk(P, N, loop, s0), budget=200_000)
            if seq[0] != s0:  # 2-opt keeps the first point; guard anyway
                seq = seq[seq.index(s0) :] + seq[: seq.index(s0)]
            order += seq
            rid += [g] * len(seq)
            first, anchor = P[seq[0]], P[seq[-1]]
    return order, rid


def turn_lifts(P_ordered: np.ndarray, rows: np.ndarray, footprint: float) -> np.ndarray:
    """Per move: True where the move goes to another row and is longer than the footprint diameter."""
    if len(P_ordered) < 2 or footprint <= 0:
        return np.zeros(max(len(P_ordered) - 1, 0), bool)
    step = np.linalg.norm(np.diff(P_ordered, axis=0), axis=1)
    return (np.diff(rows) != 0) & (step > 2 * footprint)
