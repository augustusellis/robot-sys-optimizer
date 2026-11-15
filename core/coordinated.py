"""Coordinated rail / gantry motion (v10 T16): the carrier moves while the arm works.

The plan's stops (stop-and-go, *Uncoordinated*) tell where the arm likes to stand relative to the
tool: for every planned point its TCP's position along the carrier axes minus its stop's carrier
value. *Coordinated* works the plan's points in **passes**, one per target group (face), each over
every reached point of that group from any stop (rows along the rail by default), in the face's most
common joint configuration; a face is split into one pass per configuration only when that has
fewer problem samples. In a pass the carrier is at the tool's axis position minus the pass's
offset (the median over its points; smoothed over ``SMOOTH_M`` of path, inside the travel), and
the arm is solved for
that moving base (the pass's configuration, nearest joints), collision checked. So each pass is
split between the carrier axes (the long motion) and the arm (the rest, within its joint limits).
A gantry follows along the axis its points extend most; its other axis stays where the points'
stops have it (passes split by it).
Between passes the arm goes home, the carrier drives and the arm comes back out (without a home:
the carrier drives with the arm held, then a planned joint move). Rails follow along s, gantries
along x and y; the lift keeps the plan's most common height. Mobile bases stay stop-and-go.

Samples carry their carrier values and base pose (``PathSample.carrier`` / ``T_world_base``), so
playback moves the carrier along and core.cycle_time times the carrier as one more joint.
"""

from __future__ import annotations

import numpy as np
import trimesh
from scipy.spatial.transform import Rotation, Slerp

from core.configs import config_label
from core.path_types import PathPlan, PathSample, StationPath

SMOOTH_M = 0.15  # carrier set points averaged over this much path
HELD_STEP = 0.05  # m: stops whose held gantry axis differs by less work in one pass
KINDS = ("rail", "gantry")


def axis_coords(carrier, P_world: np.ndarray) -> np.ndarray:
    """(n, k) positions in the carrier frame along its following DOFs (rail: x; gantry: x, y)."""
    T = np.linalg.inv(carrier.T_frame)
    L = P_world @ T[:3, :3].T + T[:3, 3]
    return L[:, :1] if carrier.kind == "rail" else L[:, :2]


def follow_dofs(carrier) -> list[str]:
    return ["s"] if carrier.kind == "rail" else ["x", "y"]


def _smooth(X: np.ndarray, path_s: np.ndarray, width: float) -> np.ndarray:
    """Moving average of X over ``width`` of path length (ends kept)."""
    out = np.empty_like(X)
    for i, s in enumerate(path_s):
        m = np.abs(path_s - s) <= width / 2
        out[i] = X[m].mean(axis=0)
    return out


class _Ctx:
    """What every pass needs (to keep the signatures short)."""

    def __init__(self, tr, checker, mesh, carrier, sol, results, points, T_world_part, step, pattern):
        from core.export import station_pose

        self.tr, self.ch, self.mesh, self.carrier = tr, checker, mesh, carrier
        self.sol, self.results, self.points, self.Tp = sol, results, points, T_world_part
        self.step, self.pattern = step, pattern
        stops = [station_pose(carrier.base, st, carrier.T_static) for st in sol.stations]
        self.vals = [carrier.clip(st.carrier) for st in sol.stations]
        self.tool = {}
        for p in results:  # each point's planned tool pose (part frame), from its own stop
            r = results[p]
            if r.status != "unreachable" and r.q is not None and p in sol.assignment and p in points:
                checker.set_base(stops[sol.assignment[p]])
                self.tool[p] = tr.tcp_pose(r)
        self.lift = (
            {"lift": float(np.median([v["lift"] for v in self.vals]))} if "lift" in carrier.dofs else {}
        )
        # the carrier follows along the axis the part's points extend most; a gantry's other axis
        # stays where each point's stop has it (passes are split by that value)
        names = follow_dofs(carrier)
        self.follow, self.held = names, []
        if len(names) > 1 and self.tool:
            A = axis_coords(carrier, np.array([(T_world_part @ T)[:3, 3] for T in self.tool.values()]))
            k = int(np.argmax(np.ptp(A, axis=0)))
            self.follow, self.held = [names[k]], [names[1 - k]]

    def held_key(self, p: int) -> tuple:
        v = self.vals[self.sol.assignment[p]]
        return tuple(round(v[n] / HELD_STEP) for n in self.held)


def trace_coordinated(
    checker,
    mesh: trimesh.Trimesh,
    sol,
    results: dict,
    points: dict,
    carrier,
    T_world_part: np.ndarray,
    step: float = 0.02,
    jump_deg: float = 30.0,
    pattern=None,
    home: np.ndarray | None = None,
    group_of: dict[int, str] | None = None,
) -> PathPlan:
    """Coordinated paths of plan ``sol`` (see the module docstring). ``points``: point id ->
    (p, n) part frame; ``pattern``: core.patterns.PatternSettings of each pass (None: zigzag with
    rows along the first carrier axis); ``group_of``: point id -> target group."""
    from core.paths import _Tracer

    if carrier.kind not in KINDS:
        raise ValueError("coordinated motion needs a rail or a gantry")
    tr = _Tracer(checker, mesh, step, jump_deg)
    c = _Ctx(tr, checker, mesh, carrier, sol, results, points, T_world_part, step, pattern)
    if not c.tool:
        return PathPlan([], step)
    faces: dict[tuple, list[int]] = {}  # per target group (and held gantry axis)
    label = {p: config_label(checker.ik, np.asarray(results[p].q, float)) for p in c.tool}
    for p in c.tool:
        faces.setdefault(((group_of or {}).get(p, ""), c.held_key(p)), []).append(p)
    built: list[list[PathSample]] = []
    for pids in faces.values():  # one pass in the face's most common configuration, unless one
        labels = sorted({label[p] for p in pids}, key=lambda x: -sum(label[p] == x for p in pids))
        one = _pass(c, pids, labels[0])  # pass per configuration has fewer problems
        if len(labels) > 1 and _bad(one):
            split = [_pass(c, [p for p in pids if label[p] == lab], lab) for lab in labels]
            if sum(_bad(s) for s in split) < _bad(one):
                built += [s for s in split if s]
                continue
        if one:
            built.append(one)
    if not built:
        return PathPlan([], step)
    built.sort(key=lambda s: -len(s))
    chain, left = [built[0]], built[1:]  # the nearest pass next

    def centre(seg):
        return np.mean([x.T_part_tcp[:3, 3] for x in seg], axis=0)

    while left:
        here = chain[-1][-1].T_part_tcp[:3, 3]
        j = min(range(len(left)), key=lambda k: float(np.linalg.norm(centre(left[k]) - here)))
        chain.append(left.pop(j))
    home = None if home is None else np.asarray(home, float)
    out: list[PathSample] = []
    for seg in chain:
        if out:
            out += _between(c, out[-1], seg[0], home)
        elif home is not None:
            out += _joint(c, seg[0].carrier, home, seg[0].q, seg[0].tcp, (0, seg[0].b), (False, True))
        out += seg
    if home is not None and out and out[-1].q is not None:
        out += _joint(c, out[-1].carrier, out[-1].q, home, out[-1].tcp, (out[-1].b, 0), (True, False))
    if not out:
        return PathPlan([], step)
    plan = PathPlan([StationPath(0, out[0].T_world_base, out[0].carrier, out)], step)
    plan.smooth = tr.smooth_stats
    return plan


def _pass(c: _Ctx, pids: list[int], label: str) -> list[PathSample]:
    """The samples of one coordinated pass over ``pids`` in joint configuration ``label``."""
    from core.patterns import PatternSettings, order_points

    names, every = c.follow, follow_dofs(c.carrier)
    cols = [every.index(n) for n in names]
    off = []
    for p in pids:  # the arm's offset along the followed axis at the point's own stop
        Pw = (c.Tp @ c.tool[p])[:3, 3]
        off.append(
            axis_coords(c.carrier, Pw[None])[0][cols]
            - np.array([c.vals[c.sol.assignment[p]][n] for n in names])
        )
    d0 = np.median(np.array(off), axis=0)
    held = {n: float(np.median([c.vals[c.sol.assignment[p]][n] for p in pids])) for n in c.held}
    P = np.array([c.points[p][0] for p in pids], float)
    N = np.array([c.points[p][1] for p in pids], float)
    pattern = c.pattern
    if pattern is None:
        ax = (np.linalg.inv(c.Tp) @ c.carrier.T_frame)[:3, 0]
        pattern = PatternSettings("Zigzag", float(np.degrees(np.arctan2(ax[1], ax[0]))))
    order = [pids[i] for i in order_points(P, N, pattern).order]
    rows = [(order[0], order[0], 0.0, c.tool[order[0]])]
    for a, b in zip(order[:-1], order[1:], strict=True):
        (Pa, Na), (Pb, Nb) = c.points[a], c.points[b]
        n = max(1, int(np.ceil(np.linalg.norm(np.asarray(Pb) - Pa) / c.step)))
        s = np.arange(1, n + 1) / n
        chord = np.asarray(Pa) + s[:, None] * (np.asarray(Pb) - Pa)
        on, _, _ = trimesh.proximity.closest_point(c.mesh, chord)
        on[-1] = Pb
        nrm = np.asarray(Na) + s[:, None] * (np.asarray(Nb) - Na)
        nrm /= np.linalg.norm(nrm, axis=1, keepdims=True)
        ra, rb = c.results[a], c.results[b]
        so = ra.standoff + s * (rb.standoff - ra.standoff)
        rot = Slerp([0.0, 1.0], Rotation.from_matrix([c.tool[a][:3, :3], c.tool[b][:3, :3]]))(s).as_matrix()
        for i in range(n):
            T = np.eye(4)
            T[:3, :3], T[:3, 3] = rot[i], on[i] + so[i] * nrm[i]
            rows.append((a, b, float(s[i]), T))
    Pw = np.array([(c.Tp @ T)[:3, 3] for *_, T in rows])
    path_s = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(Pw, axis=0), axis=1))]
    target = _smooth(axis_coords(c.carrier, Pw)[:, cols] - d0, path_s, SMOOTH_M)
    tcp = c.results[order[0]].tcp
    q = np.asarray(c.results[order[0]].q, float)
    out = []
    for (a, b, s, T), x in zip(rows, target, strict=True):
        v = c.carrier.clip({**dict(zip(names, map(float, x), strict=True)), **held, **c.lift})
        Tb = c.carrier.T_world_base(v)
        c.ch.set_base(Tb)
        qi, issue = c.tr.solve(T, tcp, q, label)
        if not out and issue == "jump":  # the first sample: nothing moved before it (q = planned joints)
            issue = ""
        if qi is not None and not issue and c.ch.scene.collides(qi, tcp):
            issue = "collision"
        out.append(PathSample(0, None, a, b, s, T, tcp, qi, issue, carrier=v, T_world_base=Tb))
        if qi is not None:
            q = qi
    return out


def _joint(c: _Ctx, v, qa, qb, tcp, pts, ends) -> list[PathSample]:
    """A planned joint move qa -> qb with the carrier held at ``v``."""
    if qa is None or qb is None:
        return []
    Tb = c.carrier.T_world_base(v)
    c.ch.set_base(Tb)
    mv = c.tr.joint_move(0, None, qa, qb, tcp, *pts, at_point=ends)
    for smp in mv:
        smp.carrier, smp.T_world_base = dict(v), Tb
    return mv


def _between(c: _Ctx, last: PathSample, first: PathSample, home) -> list[PathSample]:
    """From the end of one pass to the start of the next: home, drive, out again."""
    out = []
    q = last.q
    if home is not None:
        out += _joint(c, last.carrier, q, home, last.tcp, (last.b, 0), (True, False))
        q = home
    if q is None:
        return out
    a, b = last.carrier, first.carrier
    d = max((abs(b[k] - a.get(k, 0.0)) for k in b), default=0.0)
    n = max(1, int(np.ceil(d / max(c.step, 1e-3))))
    for s in np.arange(1, n + 1) / n:
        v = {k: a.get(k, 0.0) + s * (b[k] - a.get(k, 0.0)) for k in b}
        Tb = c.carrier.T_world_base(v)
        out.append(
            PathSample(0, None, last.b, first.b, float(s), last.T_part_tcp, last.tcp, q, "", "carrier", v, Tb)
        )
    pts = (0 if home is not None else last.b, first.b)
    return out + _joint(c, b, q, first.q, first.tcp, pts, (home is None, True))


def _bad(seg: list[PathSample]) -> int:
    return sum(1 for x in seg if x.issue)
