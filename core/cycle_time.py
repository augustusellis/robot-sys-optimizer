"""Cycle time (v8 T6): how long one part takes, from the planned motion.

Every move is a trapezoidal profile (accelerate, cruise, brake; a triangle when too short to
reach cruise speed). Joint moves are synchronized: all joints start and stop together, so the
move takes as long as its slowest joint. Speeds are the robot program's (``CycleSettings``),
capped by the robot's joint speed limits (URDF velocity limits).

- ``plan_estimate``: from a station plan without paths (the planner's cost term and the
  Results card): joint moves between neighbouring points of each stop's sequences, one joint
  move between sequences, carrier moves between stops (the carrier's travel metric at its
  speed) and a settle time per stop.
- ``path_time``: from built paths (core.paths): surface runs at the process speed, lifts /
  retreats at the approach speed, joint moves (home, reconfigure, planned moves) as joint
  runs, carrier runs per axis, plus the settle time per stop. Returns the parts too. v10 T14:
  arm runs are timed along the path (core.trajectory) with the joints' speed, acceleration and
  jerk limits (``CycleSettings.timing`` = "topp"), and the time of every sample is available.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class CycleSettings:
    joint_speed: float = 1.05  # rad/s (UR movej default), capped per joint by the robot's limits
    joint_acc: float = 1.4  # rad/s^2 (UR movej default)
    process_speed: float = 0.1  # m/s along the surface
    approach_speed: float = 0.25  # m/s lifts, approach / retract (UR movel default)
    linear_acc: float = 1.2  # m/s^2
    carrier_speed: float = 0.5  # m/s (rail, gantry axes, mobile base); rad/s for a mobile yaw
    carrier_acc: float = 0.5  # m/s^2
    lift_speed: float = 0.1  # m/s
    settle_s: float = 1.0  # per stop: carrier settles / localizes before the arm starts
    # v10 T14: built paths are timed "topp" (time-optimal along the path, core.trajectory, with
    # joint speed / acceleration / jerk limits) or "trapezoid" (one profile per run, pre-v10)
    timing: str = "topp"
    joint_jerk: float = 10.0  # rad/s^3 (0 = no jerk limit)


def trapezoid(d: float, v: float, a: float) -> float:
    """Time for distance ``d`` from rest to rest at max speed ``v`` and acceleration ``a``."""
    d = abs(float(d))
    if d <= 0 or v <= 0:
        return 0.0
    if a <= 0:
        return d / v
    if d <= v * v / a:  # never reaches v: accelerate half way, brake half way
        return 2.0 * np.sqrt(d / a)
    return d / v + v / a


def joint_move_time(dq, cs: CycleSettings, v_limits=None) -> float:
    """Synchronized joint move: the slowest joint's trapezoid (2*pi wraps not removed:
    pass the actual joint path)."""
    dq = np.abs(np.asarray(dq, float))
    v = np.full(len(dq), cs.joint_speed)
    if v_limits is not None:
        v = np.minimum(v, np.asarray(v_limits, float)[: len(dq)])
    return max((trapezoid(d, vi, cs.joint_acc) for d, vi in zip(dq, v, strict=True)), default=0.0)


def plan_estimate(
    seqs: dict[int, list[list[int]]],
    results: dict,
    stop_moves: list[float],
    cs: CycleSettings,
    v_limits=None,
    share: dict[int, float] | None = None,
) -> dict[str, float]:
    """Estimate for a plan: ``seqs`` = {stop: [[point ids in order], ...]} (opt.zoning
    .stop_sequences), ``stop_moves`` = carrier travel between consecutive stops (m, the
    carrier's metric). ``share`` (point id -> speed share, core.speed_zones): a move to a
    point inside a reduced-speed zone takes 1 / share as long (the extra time is "reduced").
    Returns {"joint", "carrier", "settle", ["reduced",] "total"} in seconds."""
    joint = extra = 0.0
    for groups in seqs.values():
        prev = None
        for members in groups:
            for p in members:
                q = np.asarray(results[p].q, float)
                if prev is not None:
                    t = joint_move_time(np.mod(q - prev + np.pi, 2 * np.pi) - np.pi, cs, v_limits)
                    joint += t
                    f = 1.0 if share is None else float(share.get(p, 1.0))
                    if f < 1.0:
                        extra += t * (1.0 / max(f, 1e-3) - 1.0)
                prev = q
    carrier = sum(trapezoid(d, cs.carrier_speed, cs.carrier_acc) for d in stop_moves)
    settle = cs.settle_s * len(seqs)
    out = {"joint": joint, "carrier": carrier, "settle": settle}
    if extra > 0:
        out["reduced"] = extra
    out["total"] = joint + carrier + settle + extra
    return out


def _kind(a, b) -> str:
    """Motion kind of the move from sample ``a`` to ``b``."""
    ca, cb = a.carrier, b.carrier
    moving = ca is not None and cb is not None and ca != cb
    # v10 T16: the carrier moving during a surface / approach move = coordinated motion (arm run)
    if b.kind == "carrier" or a.station != b.station or (moving and b.kind not in ("", "lift", "retreat")):
        return "carrier"
    if b.kind in ("home",) or a.q is None or b.q is None or b.a == b.b and b.s == 0.0 and a.b != b.a:
        return "joint"
    if b.kind in ("lift", "retreat") or a.kind in ("lift", "retreat"):
        return "approach"
    return "process"


def runs(samples) -> list[tuple[str, int, int]]:
    """(kind, first, last) sample index of each run of moves of one kind (moves first-1 -> last)."""
    out: list[tuple[str, int, int]] = []
    for i in range(1, len(samples)):
        k = _kind(samples[i - 1], samples[i])
        if out and out[-1][0] == k and out[-1][2] == i - 1:
            out[-1] = (k, out[-1][1], i)
        else:
            out.append((k, i - 1, i))
    return out


def _run_times(kind: str, S: list, cs: CycleSettings, v_limits) -> np.ndarray:
    """Times (from 0) at each sample of one run. Carrier runs: the trapezoid per axis (slowest).
    Arm runs: ``cs.timing`` "trapezoid" (one profile per run, spread by distance) or "topp"
    (v10 T14, core.trajectory: joint speed / acceleration / jerk limits; surface and approach runs
    also at most the TCP speed and linear acceleration along the path)."""
    n = len(S)
    if kind == "carrier":
        d = {}
        steps = np.zeros(n)
        for i in range(1, n):
            ca, cb = S[i - 1].carrier or {}, S[i].carrier or {}
            for k in set(ca) | set(cb):
                x = abs(float(cb.get(k, 0.0)) - float(ca.get(k, 0.0)))
                d[k] = d.get(k, 0.0) + x
                steps[i] = max(steps[i], x)
        T = max(
            (
                trapezoid(x, cs.lift_speed if k == "lift" else cs.carrier_speed, cs.carrier_acc)
                for k, x in d.items()
            ),
            default=0.0,
        )
        return _spread(steps, T)
    qs = [s.q for s in S]
    lin = kind in ("process", "approach")
    P = np.array([s.T_part_tcp[:3, 3] for s in S])
    dp = np.linalg.norm(np.diff(P, axis=0), axis=1)
    v_lin = cs.process_speed if kind == "process" else cs.approach_speed
    if cs.timing == "topp" and all(q is not None for q in qs):
        from core.trajectory import time_path

        Q = np.asarray(qs, float)
        v = np.full(Q.shape[1], cs.joint_speed)
        if v_limits is not None:
            v = np.minimum(v, np.asarray(v_limits, float)[: Q.shape[1]])
        a_max, j_max = np.full(Q.shape[1], cs.joint_acc), np.full(Q.shape[1], cs.joint_jerk or np.inf)
        C, keys = _carrier_cols(S)
        if C is not None:  # v10 T16 coordinated: the carrier axes are timed as more joints
            Q, v = np.c_[Q, C], np.r_[v, [cs.lift_speed if k == "lift" else cs.carrier_speed for k in keys]]
            a_max, j_max = np.r_[a_max, [cs.carrier_acc] * len(keys)], np.r_[j_max, [np.inf] * len(keys)]
        ds = np.linalg.norm(np.diff(Q, axis=0), axis=1)
        cap = acc = None
        if lin:  # the TCP's own speed along the path: s_dot <= v_lin * ds / dp
            rate = np.where(ds > 1e-12, dp / np.maximum(ds, 1e-12), 0.0)
            cap = np.where(rate > 1e-12, v_lin / np.maximum(rate, 1e-12), np.inf)
            acc = np.where(rate > 1e-12, cs.linear_acc / np.maximum(rate, 1e-12), np.inf)
        if ds.sum() > 1e-12:
            jm = None if not np.isfinite(j_max).any() else np.where(np.isfinite(j_max), j_max, 1e9)
            return time_path(Q, v, a_max, jm, cap, acc)
        if not lin or dp.sum() <= 0:
            return np.zeros(n)
    if lin:
        T = trapezoid(dp.sum(), v_lin, cs.linear_acc)
        C, keys = _carrier_cols(S)
        if C is not None:  # v10 T16: no faster than the carrier can follow
            d = np.abs(np.diff(C, axis=0)).sum(axis=0)
            T = max(T, *(trapezoid(x, cs.carrier_speed, cs.carrier_acc) for x in d))
        return _spread(np.r_[0.0, dp], T)
    steps = np.zeros(n)
    tot = np.zeros(len(qs[0])) if qs and qs[0] is not None else None
    for i in range(1, n):
        if qs[i] is not None and qs[i - 1] is not None:
            dq = np.abs(np.asarray(qs[i], float) - np.asarray(qs[i - 1], float))
            steps[i] = float(dq.max())
            tot = dq if tot is None or len(tot) != len(dq) else tot + dq
    return _spread(steps, joint_move_time(tot, cs, v_limits) if tot is not None else 0.0)


def _carrier_cols(S: list) -> tuple[np.ndarray | None, list[str]]:
    """The carrier values of a run's samples as columns, when they change along it (coordinated)."""
    cv = [s.carrier for s in S]
    if any(c is None for c in cv) or all(c == cv[0] for c in cv):
        return None, []
    keys = sorted(cv[0])
    return np.array([[float(c.get(k, 0.0)) for k in keys] for c in cv]), keys


def _spread(steps: np.ndarray, T: float) -> np.ndarray:
    """Times at the samples of a run of total time ``T``, proportional to each step's size."""
    c = np.cumsum(steps)
    return c / c[-1] * T if c[-1] > 0 else np.linspace(0.0, T, len(steps))


def path_time(
    plan, cs: CycleSettings, v_limits=None, slow=None, times: list | None = None
) -> dict[str, float]:
    """Cycle time of built paths (core.paths.PathPlan). Returns seconds per kind of motion:
    {"process", "approach", "joint", "carrier", "settle", ["reduced",] "total"}. ``slow(sample)``
    (v8 T18) -> speed share (1 = full) for a move ending at that sample: inside a reduced-speed
    zone a move takes 1 / share as long (the extra time is reported as "reduced"). ``times``
    (v10 T14): filled with the time of every sample (s from the start; playback)."""
    out = dict.fromkeys(("process", "approach", "joint", "carrier", "settle"), 0.0)
    samples = plan.samples
    at = np.zeros(len(samples))
    starts = set()
    k = 0
    for st in plan.stations:
        if st.samples:
            out["settle"] += cs.settle_s
            starts.add(k)
        k += len(st.samples)
    extra = 0.0
    clock = 0.0
    done = 0
    for kind, i0, i1 in runs(samples):
        for j in range(done, i0 + 1):  # samples before this run (settle at stop starts)
            if j in starts and j > 0:
                clock += cs.settle_s
            at[j] = clock
        t = _run_times(kind, samples[i0 : i1 + 1], cs, v_limits)
        out[kind] += float(t[-1])
        stretch = np.ones(len(t) - 1)
        if slow is not None and kind != "carrier":
            for m in range(1, len(t)):
                f = float(slow(samples[i0 + m]))
                if f < 1.0:  # the move at full speed, then stretched by the zone's share
                    stretch[m - 1] = 1.0 / max(f, 1e-3)
            extra += float(np.sum(np.diff(t) * (stretch - 1.0)))
        at[i0 + 1 : i1 + 1] = clock + np.cumsum(np.diff(t) * stretch)
        clock = at[i1]
        done = i1 + 1
    if starts and 0 in starts:
        at += cs.settle_s
    if extra > 0:
        out["reduced"] = extra
    out["total"] = float(sum(out.values()))
    if times is not None:
        times[:] = list(at)
    return out
