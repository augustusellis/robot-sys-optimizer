"""Trajectory optimisation of the joint moves between points (v10 T13, CHOMP style, numpy).

The moves to / from home, between groups and through configuration changes come from
core.joint_plan (straight, one joint group at a time, or RRT-connect + shortcut): collision-free
but often a zig-zag of straight joint segments close to the part. ``smooth_path`` takes such a
path as the seed, resamples it to ``n`` waypoints and lowers

    J = (n - 1) sum |W (q[i+1] - q[i])|^2   (the CHOMP smoothness term: joint speed squared over
                                            the move, wrist joints weighted less; also shortens it)
      + w_clear * sum_i max(0, d_safe - d(q[i]))^2   (d = clearance to the part, core.collision)

by covariant gradient steps (the CHOMP update: the smoothness metric A^-1 spreads each waypoint's
obstacle gradient along the path), the ends fixed and every waypoint inside the joint limits. The
clearance gradient (only at waypoints closer than ``d_safe``) is analytic when ``clearance_grad`` is
given (v12 #21: core.collision.part_clearance_grad, the nearest points and the link Jacobian, one
distance pass instead of seven), else a forward difference per joint.
The result is kept only if it is collision-free when checked densely (``step``) and better than
the seed (lower cost); otherwise the seed is returned: RRT-connect + shortcut stay the fallback.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

import numpy as np

TOL = 0.005  # relative cost gain per step below which the optimisation counts as converged
WEIGHTS = np.array([1.0, 1.0, 1.0, 0.5, 0.5, 0.5])  # wrist joints move cheaply


@dataclass
class SmoothStats:
    """What one ``smooth_path`` call did (for logs and tests)."""

    used: bool = False
    iters: int = 0
    cost_seed: float = 0.0
    cost: float = 0.0
    length_seed: float = 0.0  # weighted joint path length (rad)
    length: float = 0.0
    clear_seed: float = float("inf")  # smallest clearance (m) over the waypoints
    clear: float = float("inf")


def resample(path: list[np.ndarray], n: int) -> np.ndarray:
    """(n, dof) waypoints evenly spaced by joint-space arc length along ``path``."""
    Q = np.asarray(path, float)
    seg = np.linalg.norm(np.diff(Q, axis=0), axis=1)
    s = np.r_[0.0, np.cumsum(seg)]
    if s[-1] <= 1e-12:
        return np.repeat(Q[:1], n, axis=0)
    t = np.linspace(0.0, s[-1], n)
    return np.stack([np.interp(t, s, Q[:, j]) for j in range(Q.shape[1])], axis=1)


def path_length(Q: np.ndarray, w: np.ndarray | None = None) -> float:
    w = WEIGHTS[: Q.shape[1]] if w is None else w
    return float(np.sum(np.linalg.norm(np.diff(Q, axis=0) * w, axis=1)))


def _metric(n: int) -> np.ndarray:
    """The finite-difference velocity metric A = K^T K of the interior waypoints."""
    m = n - 2
    K = np.zeros((m + 1, m))
    for i in range(m + 1):
        if i < m:
            K[i, i] = 1.0
        if i > 0:
            K[i, i - 1] = -1.0
    return K.T @ K


def smooth_path(
    collides: Callable[[np.ndarray], bool],
    clearance: Callable[[np.ndarray], float],
    path: list[np.ndarray],
    lo,
    hi,
    step: float,
    n: int = 20,
    d_safe: float = 0.05,
    w_clear: float = 40.0,
    iters: int = 40,
    eta: float = 0.08,
    eps: float = 1e-3,
    clearance_grad: Callable[[np.ndarray], tuple[float, np.ndarray]] | None = None,
) -> tuple[list[np.ndarray], SmoothStats]:
    """(waypoints, stats): the optimised path, or ``path`` when that does no better."""
    from core.joint_plan import _edge_ok

    st = SmoothStats()
    seed = np.asarray(path, float)
    if len(seed) < 2:
        return list(path), st
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    w = WEIGHTS[: seed.shape[1]] if seed.shape[1] <= len(WEIGHTS) else np.ones(seed.shape[1])
    Q = resample(list(seed), n)
    A = _metric(n)
    Ainv = np.linalg.inv(A)
    cache: dict[bytes, float] = {}
    grads: dict[bytes, np.ndarray] = {}

    def dist(q: np.ndarray) -> float:
        k = np.round(q, 6).tobytes()
        if k not in cache:
            if clearance_grad is not None:
                d, g = clearance_grad(q)
                cache[k], grads[k] = float(d), np.asarray(g, float)
            else:
                cache[k] = float(clearance(q))
        return cache[k]

    def cost(Q: np.ndarray) -> tuple[float, np.ndarray]:
        vel = np.diff(Q, axis=0) * w
        d = np.array([dist(q) for q in Q[1:-1]])
        pen = np.maximum(0.0, d_safe - np.minimum(d, d_safe))
        return float((n - 1) * np.sum(vel**2) + w_clear * np.sum(pen**2)), d

    def total_cost(Q: np.ndarray) -> float:
        return cost(Q)[0]

    st.cost_seed = total_cost(Q)
    st.length_seed = path_length(seed, w)
    st.clear_seed = min((dist(q) for q in Q), default=float("inf"))
    best, best_c = Q.copy(), st.cost_seed
    slow = 0
    for it in range(iters):
        c, d = cost(Q)
        X = Q[1:-1]
        V = np.diff(Q, axis=0) * w  # smoothness gradient (the ends fixed)
        g_s = 2 * (n - 1) * (V[:-1] - V[1:]) * w
        g_o = np.zeros_like(X)
        for i in np.flatnonzero(d < d_safe):
            base = d[i]
            gk = grads.get(np.round(X[i], 6).tobytes())
            if gk is not None:  # v12 #21: analytic
                g_o[i] = -2 * w_clear * (d_safe - base) * gk[: X.shape[1]]
                continue
            for j in range(X.shape[1]):
                qe = X[i].copy()
                qe[j] += eps
                g_o[i, j] = -2 * w_clear * (d_safe - base) * (dist(qe) - base) / eps
        G = g_s + g_o
        step_x = Ainv @ G
        scale = np.max(np.abs(step_x))
        if scale < 1e-6:
            break
        X_new = np.clip(X - eta * step_x / max(scale, 1.0), lo, hi)
        Q_new = np.vstack([Q[:1], X_new, Q[-1:]])
        c_new = total_cost(Q_new)
        st.iters = it + 1
        if c_new < c:
            Q = Q_new
            if c_new < best_c:
                best, best_c = Q.copy(), c_new
            slow = slow + 1 if c - c_new < TOL * c else 0
            if slow >= 3:  # three steps that each gained less than TOL: converged
                break
        else:
            eta *= 0.5
            if eta < 1e-4:
                break
    # never closer to the part than the seed (d_safe is a soft target: a little inside it is fine
    # when the seed was further out), and clear when checked densely
    ok = (
        best_c < st.cost_seed - 1e-9
        and min(dist(q) for q in best) >= min(st.clear_seed, 0.9 * d_safe) - 1e-4
        and all(
            _edge_ok(collides, best[i], best[i + 1], step, i < len(best) - 2) for i in range(len(best) - 1)
        )
    )
    if not ok:
        st.cost = st.cost_seed
        st.length, st.clear = st.length_seed, st.clear_seed
        return list(path), st
    st.used, st.cost = True, best_c
    st.length = path_length(best, w)
    st.clear = min(dist(q) for q in best)
    return list(best), st


def smooth_move(tracer, hit, way: list[np.ndarray], tcp: str, step: float) -> list[np.ndarray]:
    """core.paths._Tracer.joint_move: ``way`` optimised with the part clearance of ``tcp``'s
    tool; the stats go to ``tracer.smooth_stats``."""
    ch = tracer.ch
    out, st = smooth_path(
        hit,
        lambda q: ch.scene.part_clearance(q, tcp),
        way,
        ch.lo,
        ch.hi,
        step,
        clearance_grad=lambda q: ch.scene.part_clearance_grad(q, tcp),  # v12 #21
    )
    tracer.smooth_stats.append(st)
    return out
