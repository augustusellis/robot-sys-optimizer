"""Time-parameterised trajectories along built paths (v10 T14, TOPP style, numpy).

``time_path(Q, ...)`` times a joint path given as waypoints (the path samples of core.paths) so that
it is as fast as the limits allow: per-joint velocity, acceleration and jerk, plus optional caps on
the path speed per segment (a TCP speed along the surface, core.cycle_time). The path parameter s is
the joint-space arc length; with u = s_dot^2 the limits become, per segment k (tangent q'_k):

  u <= min_j (v_j / |q'_kj|)^2            joint speed (and the segment's own cap, e.g. TCP speed)
  |s_ddot| <= min_j a_j / |q'_kj|         joint acceleration along the segment
  u_i <= min_j a_j / |q''_ij|             at waypoint i, the turn of the path (q'' between segments)

and the time-optimal profile is the classic forward / backward pass (TOPP by numerical integration,
zero-order: constant s_ddot per segment): u_{i+1} <= u_i + 2 s_ddot_k ds_k and back, starting and
ending at rest. A straight move then gives exactly the trapezoid of core.cycle_time.

Jerk is limited afterwards: where the joint acceleration changes faster than j between two
segments, the time of those segments is stretched by (jerk / j)^(1/3) (acceleration falls with the
square of the stretch, jerk with the cube), repeated until every jerk is within the limit. Returns
the time at every waypoint.
"""

from __future__ import annotations

import numpy as np

MAX_ROUNDS = 40  # jerk passes


def time_path(
    Q,
    v_max,
    a_max,
    j_max=None,
    ds_cap: np.ndarray | None = None,
    acc_cap: np.ndarray | None = None,
) -> np.ndarray:
    """Times (s) at each waypoint of ``Q`` (m, dof), from rest to rest. ``v_max`` / ``a_max`` /
    ``j_max``: per joint (scalars broadcast; j_max None = no jerk limit). ``ds_cap`` / ``acc_cap``:
    per segment (m - 1) upper bounds on s_dot and |s_ddot| (path parameter = joint arc length)."""
    Q = np.asarray(Q, float)
    m = len(Q)
    if m < 2:
        return np.zeros(m)
    dof = Q.shape[1]
    v = np.broadcast_to(np.asarray(v_max, float), (dof,))
    a = np.broadcast_to(np.asarray(a_max, float), (dof,))
    dQ = np.diff(Q, axis=0)
    ds = np.linalg.norm(dQ, axis=1)
    moving = ds > 1e-12
    if not moving.any():
        return np.zeros(m)
    T = np.zeros_like(dQ)
    T[moving] = dQ[moving] / ds[moving, None]  # unit tangents
    absT = np.maximum(np.abs(T), 1e-12)
    cap = np.min(v / absT, axis=1)  # s_dot per segment
    acc = np.min(a / absT, axis=1)  # |s_ddot| per segment
    if ds_cap is not None:
        cap = np.minimum(cap, np.asarray(ds_cap, float))
    if acc_cap is not None:
        acc = np.minimum(acc, np.asarray(acc_cap, float))
    u = np.full(m, np.inf)
    u[0] = u[-1] = 0.0
    u[1:] = np.minimum(u[1:], cap**2)
    u[:-1] = np.minimum(u[:-1], cap**2)
    for i in range(1, m - 1):  # the path's turn at waypoint i
        if moving[i - 1] and moving[i]:
            dd = (T[i] - T[i - 1]) / (0.5 * (ds[i - 1] + ds[i]))
            big = np.abs(dd) > 1e-12
            if big.any():
                u[i] = min(u[i], float(np.min(a[big] / np.abs(dd[big]))))
    for k in range(m - 1):  # forward: accelerate at most acc
        u[k + 1] = min(u[k + 1], u[k] + 2 * acc[k] * ds[k])
    for k in range(m - 2, -1, -1):  # backward: brake at most acc
        u[k] = min(u[k], u[k + 1] + 2 * acc[k] * ds[k])
    sd = np.sqrt(np.maximum(u, 0.0))
    dt = np.zeros(m - 1)
    for k in range(m - 1):
        if not moving[k]:
            continue
        s = sd[k] + sd[k + 1]
        dt[k] = 2 * ds[k] / s if s > 1e-12 else 2 * np.sqrt(ds[k] / max(acc[k], 1e-9))
    if j_max is not None:
        dt = _limit_jerk(dQ, dt, np.broadcast_to(np.asarray(j_max, float), (dof,)))
    return np.r_[0.0, np.cumsum(dt)]


def joint_rates(Q, t) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(velocity per segment, acceleration between segments, jerk) of waypoints ``Q`` at times ``t``."""
    Q, t = np.asarray(Q, float), np.asarray(t, float)
    dt = np.diff(t)
    ok = dt > 1e-12
    V = np.zeros((len(dt), Q.shape[1]))
    V[ok] = np.diff(Q, axis=0)[ok] / dt[ok, None]
    V = np.vstack([np.zeros(Q.shape[1]), V, np.zeros(Q.shape[1])])  # at rest before and after
    h = np.r_[dt[:1], 0.5 * (dt[:-1] + dt[1:]), dt[-1:]] if len(dt) else np.zeros(1)
    A = np.diff(V, axis=0) / np.maximum(h, 1e-12)[:, None]
    hj = 0.5 * (h[:-1] + h[1:])
    J = np.diff(A, axis=0) / np.maximum(hj, 1e-12)[:, None]
    return V[1:-1], A, J


def _limit_jerk(dQ: np.ndarray, dt: np.ndarray, j: np.ndarray) -> np.ndarray:
    """Segment times with the jerk limited: the path speed over time (rest to rest) is averaged over
    a moving window of tau = peak joint acceleration / jerk limit (the joint with the tightest
    ratio). A trapezoid becomes the S-curve with jerk a / tau and the move takes tau longer; the
    distance along the path is unchanged and speeds and accelerations do not grow. Widened (up to
    4 x) while any joint's jerk is still above its limit."""
    T = float(np.sum(dt))
    if T <= 0:
        return dt
    Q = np.vstack([np.zeros(dQ.shape[1]), np.cumsum(dQ, axis=0)])
    t0 = np.r_[0.0, np.cumsum(dt)]
    _, A, _ = joint_rates(Q, t0)
    tau0 = float(np.max(np.max(np.abs(A), axis=0) / j))
    if tau0 <= 1e-9:
        return dt
    s = np.r_[0.0, np.cumsum(np.linalg.norm(dQ, axis=1))]
    out = dt
    for widen in (1.0, 1.5, 2.0, 3.0, 4.0):
        tau = tau0 * widen
        h = min(T / 4000.0, tau / 40.0)
        g = np.arange(0.0, T + h, h)
        sg = np.interp(g, t0, s)
        v = np.r_[np.diff(sg) / h, 0.0]
        w = max(1, int(round(tau / h)))
        vf = np.convolve(np.r_[v, np.zeros(w)], np.ones(w) / w)[: len(v) + w]
        sf = np.r_[0.0, np.cumsum(vf[:-1]) * h]
        sf *= s[-1] / max(sf[-1], 1e-12)
        tf = np.arange(len(sf)) * h
        mono = sf + np.arange(len(sf)) * 1e-15  # np.interp needs increasing x
        ti = np.interp(s + np.arange(len(s)) * 1e-15, mono, tf)
        ti[0], ti[-1] = 0.0, tf[-1]
        out = np.maximum(np.diff(ti), 0.0)
        # the jerk of each joint on the filtered profile, at tau / 2 (finer steps only see the
        # staircase of the waypoints' segment speeds): within the limit?
        c = max(1, w // 2)
        qg = np.stack([np.interp(sf[::c], s, Q[:, k]) for k in range(Q.shape[1])], axis=1)
        jg = np.diff(qg, n=3, axis=0) / (c * h) ** 3
        if not len(jg) or np.all(np.max(np.abs(jg), axis=0) <= j * 1.05):
            break
    return out
