"""Free tool spin along a path (v10 task 9).

Where a target group's spin about the surface normal is free (or a range), the spin at every path
sample is a choice. The planner fixed one spin per point (the first feasible pose), and the paths
slerped between those, so neighbouring points with unlucky spins made the wrist turn a long way, or
unwind at a joint limit. Here the spin is chosen per sample by dynamic programming (Viterbi) over
spin steps, for one sequence of a stop (one joint configuration):

* states of a sample: the tool pose turned about its own axis by each allowed offset (``STEP``
  apart; a range tolerance limits the offsets, the planned spin is offset 0), each IK solution in
  the sequence's configuration, in the 2*pi copy nearest the planned joints and with wrist joints
  4 / 6 also one turn either way (inside the limits);
* state cost: closeness to the joint limits (``LIMIT_BAND``), so the path keeps away from them;
* step cost: the weighted joint motion to the previous sample (wrist joints count ``W_WRIST``);
  a step over the jump limit costs ``JUMP_COST`` extra.

The caller keeps the planned-spin path of a sequence when it is no worse (``motion``, problem
samples): the spin grid cannot follow every smooth slerp, and the planned path may already be fine.
Collisions are checked lazily: the cheapest path is checked sample by sample, colliding states are
dropped and the search runs again (``ROUNDS``). A sample without any state (no IK in this
configuration at any allowed spin) makes the sequence fall back to the planned spins (None).
Pure geometry + core.feasibility: no viser.
"""

from __future__ import annotations

from itertools import product

import numpy as np
import trimesh
from scipy.spatial.transform import Rotation, Slerp

from core.configs import config_label
from core.path_types import PathSample

STEP = np.radians(15.0)  # spin offsets this far apart
LIMIT_BAND = np.radians(20.0)  # joints closer than this to a limit pay a cost
LIMIT_COST = 0.5  # rad of joint motion a joint at its limit is worth
W_WRIST = 2.0  # wrist joints' motion counts double (they flip and unwind)
JUMP_COST = 20.0  # extra cost of a step over the jump limit
ARRIVE = 1.0  # weight of the moves from the previous pose / back to ``leave`` (cycle time too)
ROUNDS = 8  # collision repair rounds


def offsets(allowed) -> np.ndarray:
    """Spin offsets (rad) for ``allowed`` = None (free) or (lo, hi) in rad around the planned spin."""
    if allowed is None:
        return np.arange(-np.pi, np.pi - 1e-9, STEP)
    lo, hi = allowed
    if hi - lo < 1e-9:
        return np.array([0.0])
    n = int(np.floor((hi - lo) / STEP + 1e-9))
    out = sorted({0.0, *(lo + STEP * np.arange(n + 1)).tolist(), hi})
    return np.array([v for v in out if lo - 1e-9 <= v <= hi + 1e-9])


def meet(a, b):
    """Offsets allowed at both ends of a segment (None = free)."""
    if a is None:
        return b
    if b is None:
        return a
    lo, hi = max(a[0], b[0]), min(a[1], b[1])
    return (lo, hi) if hi >= lo else (0.0, 0.0)


def _rz(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]])


def segment_poses(tr, ra, rb, Pa, Pb, Na, Nb, lift: float = 0.0):
    """(s, TCP poses in the part frame) of the samples a -> b, exactly as ``_Tracer.segment``."""
    Ta, Tb = tr.tcp_pose(ra), tr.tcp_pose(rb)
    n = max(1, int(np.ceil(np.linalg.norm(Pb - Pa) / tr.step)))
    s = np.arange(1, n + 1) / n
    chord = Pa + s[:, None] * (Pb - Pa)
    on, _, _ = trimesh.proximity.closest_point(tr.mesh, chord)
    on[-1] = Pb
    normal = Na + s[:, None] * (Nb - Na)
    normal /= np.linalg.norm(normal, axis=1, keepdims=True)
    so = ra.standoff + s * (rb.standoff - ra.standoff)
    if lift > 0:
        so = so + lift * np.minimum(1.0, 8 * s * (1 - s))
    rot = Slerp([0.0, 1.0], Rotation.from_matrix([Ta[:3, :3], Tb[:3, :3]]))(s).as_matrix()
    out = []
    for i in range(n):
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = rot[i], on[i] + so[i] * normal[i]
        out.append(T)
    return s, out


class _Sample:
    """One path sample: its pose, the allowed offsets, and (after ``solve``) its states."""

    def __init__(self, a, b, s, T, allowed, q_ref, kind=""):
        self.a, self.b, self.s, self.T, self.kind = a, b, float(s), T, kind
        self.off = offsets(allowed)
        self.q_ref = np.asarray(q_ref, float)
        self.Q = np.zeros((0, 6))
        self.which = np.zeros(0, int)  # offset index of each state
        self.cost = np.zeros(0)


def _copies(tr, q: np.ndarray, ref: np.ndarray) -> list[np.ndarray]:
    """q's copy nearest ``ref`` (inside the limits), plus wrist joints 4 / 6 one turn either way."""
    ch = tr.ch
    base = tr.nearest_copy(q, ref)
    out = []
    for d4, d6 in product((0, -1, 1), repeat=2):
        c = base.copy()
        if len(c) > 5:
            c[3] += 2 * np.pi * d4
            c[5] += 2 * np.pi * d6
        elif d4 or d6:
            continue
        if np.all(c >= ch.lo - 1e-9) and np.all(c <= ch.hi + 1e-9):
            out.append(c)
    return out


def _solve(tr, smp: _Sample, tcp: str, label: str) -> None:
    ch = tr.ch
    Ts = np.array([ch.T_base_part @ smp.T @ _rz(o) @ ch._T_tcp_flange[tcp] for o in smp.off])
    if tr.batch:
        Q, V, sing = ch.ik.solve_batch(Ts, return_singular=True)
        sols = [ch.ik.solve(Ts[j]) if sing[j] else Q[j][V[j]] for j in range(len(Ts))]
    else:
        sols = [ch.ik.solve(T) for T in Ts]
    qs, which = [], []
    for j, S in enumerate(sols):
        for q in np.atleast_2d(S) if len(S) else []:
            if config_label(ch.ik, q) != label:
                continue
            for c in _copies(tr, np.asarray(q, float), smp.q_ref):
                qs.append(c)
                which.append(j)
    if not qs:
        return
    Q = np.array(qs)
    near = np.minimum(Q - ch.lo, ch.hi - Q)
    smp.Q, smp.which = Q, np.array(which)
    smp.cost = LIMIT_COST * np.sum(np.clip(1.0 - near / LIMIT_BAND, 0.0, None) ** 2, axis=1)


def _weights(n: int) -> np.ndarray:
    w = np.ones(n)
    w[3:] = W_WRIST
    return w


def _viterbi(samples: list[_Sample], ref, jump: float, banned: list[set], leave=None) -> list[int] | None:
    w = _weights(samples[0].Q.shape[1])
    D = samples[0].cost.copy()
    if ref is not None:
        D = D + ARRIVE * np.abs(samples[0].Q - ref) @ w
    D[list(banned[0])] = np.inf
    back = []
    for i in range(1, len(samples)):
        P, C = samples[i - 1].Q, samples[i].Q
        d = np.abs(C[None, :, :] - P[:, None, :])  # prev x cur x joints
        step = d @ w + JUMP_COST * (d.max(axis=2) > jump)
        tot = D[:, None] + step
        arg = np.argmin(tot, axis=0)
        D = tot[arg, np.arange(len(C))] + samples[i].cost
        D[list(banned[i])] = np.inf
        back.append(arg)
    if leave is not None:  # the move away after the sequence (back home)
        D = D + ARRIVE * np.abs(samples[-1].Q - leave) @ w
    if not np.isfinite(D).any():
        return None
    path = [int(np.argmin(D))]
    for arg in reversed(back):
        path.append(int(arg[path[-1]]))
    return path[::-1]


def optimize_sequence(tr, k, zone, pids, results, points, label, ref, allowed, lifts, lift, leave=None):
    """PathSamples of one sequence (first point, then each segment) with the spin chosen per
    sample, or None (a sample has no state: use the planned spins). ``allowed(pid)``: None (free)
    or (lo, hi) offsets in rad; ``ref``: the joints the arm arrives from, ``leave``: where it goes
    after the sequence (None: anywhere)."""
    r0 = results[pids[0]]
    tcp = r0.tcp
    q_plan = {p: tr.nearest_copy(results[p].q, r0.q if ref is None else ref) for p in pids[:1]}
    samples = [_Sample(pids[0], pids[0], 0.0, tr.tcp_pose(r0), allowed(pids[0]), q_plan[pids[0]])]
    for m, (a, b) in enumerate(zip(pids[:-1], pids[1:], strict=True)):
        ra, rb = results[a], results[b]
        if rb.tcp != tcp or config_label(tr.ch.ik, np.asarray(rb.q, float)) != label:
            return None  # another TCP or configuration in the sequence: the tracer handles it
        qa, qb = q_plan[a], tr.nearest_copy(rb.q, q_plan[a])
        q_plan[b] = qb
        (Pa, Na), (Pb, Nb) = points[a], points[b]
        h = lift if lifts[m] else 0.0
        s, Ts = segment_poses(tr, ra, rb, Pa, Pb, Na, Nb, h)
        both = meet(allowed(a), allowed(b))
        for si, T in zip(s, Ts, strict=True):
            samples.append(_Sample(a, b, si, T, both, qa + si * (qb - qa), "lift" if h > 0 else ""))
    for smp in samples:
        _solve(tr, smp, tcp, label)
        if not len(smp.Q):
            return None
    banned = [set() for _ in samples]
    hit: dict[tuple[int, int], bool] = {}
    for _ in range(ROUNDS):
        path = _viterbi(samples, ref, tr.jump, banned, leave)
        if path is None:
            return None
        bad = False
        for i, j in enumerate(path):
            key = (i, j)
            if key not in hit:
                hit[key] = bool(tr.ch.scene.collides(samples[i].Q[j], tcp))
            if hit[key]:
                banned[i].add(j)
                bad = True
        if not bad:
            break
    out, prev = [], None
    for i, j in enumerate(path):
        smp = samples[i]
        q = smp.Q[j]
        T = smp.T @ _rz(smp.off[smp.which[j]])
        issue = "collision" if hit.get((i, j)) else ""
        if not issue and prev is not None and np.max(np.abs(q - prev)) > tr.jump:
            issue = "jump"
        out.append(PathSample(k, zone, smp.a, smp.b, smp.s, T, tcp, q, issue, kind=smp.kind))
        prev = q
    return out


def allowed_from(groups: dict, results: dict, group_of: dict):
    """``allowed(pid)`` for ``optimize_sequence`` from the target groups' spin tolerances: None
    (free) or (lo, hi) offsets (rad) around the planned spin."""

    def fn(pid: int):
        g = groups.get(group_of.get(pid))
        spin = "free" if g is None else g.spin_deg
        if spin == "free":
            return None
        lo, hi = (-float(spin), float(spin)) if not isinstance(spin, tuple) else spin
        s = float(results[pid].spin_deg)
        return (np.radians(lo - s), np.radians(hi - s))

    return fn


STATS = {"spin": 0, "planned": 0}  # sequences that took the optimized / planned spins (diagnostics)


def motion(samples, ref=None, leave=None) -> float:
    """Weighted joint motion along samples (wrist joints ``W_WRIST``), with the moves from ``ref``
    and to ``leave`` when given: the optimizer's measure."""
    qs = [np.asarray(p.q, float) for p in samples if p.q is not None]
    qs = ([np.asarray(ref, float)] if ref is not None else []) + qs
    qs += [np.asarray(leave, float)] if leave is not None else []
    if len(qs) < 2:
        return 0.0
    d = np.abs(np.diff(np.array(qs), axis=0))
    return float((d @ _weights(d.shape[1])).sum())
