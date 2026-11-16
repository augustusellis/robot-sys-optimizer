"""Directional travel along the paths (v11 task 7).

A TCP's ``travel`` (schema.TCP) says how the tool may move over the surface relative to its own axes:

  free             any direction; the tool may spin about the normal (a sander, a polisher)
  along_x          the TCP x axis along the travel direction (a line scanner sweeping along x)
  along_y          the TCP y axis along the travel direction (a line scanner sweeping along y)
  perpendicular_x  the TCP x axis across the travel direction (a drum sander: drum axis = x)

(``along_y`` and ``perpendicular_x`` put the tool the same way; both names exist so a tool reads
naturally.) A directional TCP leaves no spin choice on the surface: at every path sample the spin
about the surface normal is the one that turns the named axis onto the local travel direction,
projected into the surface's tangent plane, or that turned half a turn (these tools are
symmetric). core.path_spin.optimize_sequence picks between the two per sample for the least joint
motion (its DP), so the wrist never turns more than it must.

On a **row change** of a row pattern (core.patterns: zigzag, unidirectional, cross-hatch) the tool
keeps the orientation of the row it leaves and steps sideways: a scanner moves along its other
axis only at the end of a row (the group's edge), never in the middle. With ``overrun_mm`` the
rows are extended past their ends, off the part, and the row change happens out there (``overrun``
samples, straight lines in the end points' tangent planes): the scan covers the part's edge and the
sideways step is off the part. ``scan_width_mm`` and ``overlap_pct`` give the row spacing a group
needs: ``row_pitch`` (Targets: *Use the tool's row spacing*).

``flag`` marks surface samples whose named axis is more than ``TOL_DEG`` off (issue ``direction``),
e.g. when no IK solution exists at the required spin and the paths fell back to the planned spins.
Pure geometry: no viser.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

TRAVEL = ("free", "along_x", "along_y", "perpendicular_x")
TOL_DEG = 2.0


MIN_TRAVEL = 1e-6  # m: shorter moves (coincident points) have no direction of their own


class Rows:
    """Pattern rows by visit (v12 #2). Cross-hatch visits every point twice, once per pass and on
    a different row each time, so a point id alone does not name its row (a dict by point id let
    the second pass overwrite the first). ``keys[i]`` = the row of visit i of one sequence;
    ``members[row]`` = that row's point ids in order; ``seg[(a, b)]`` = (row of a, row of b) for
    every visit of the move a -> b (the same move can be a row move in one pass and a row change
    in the other). Row keys are (sequence tag, row number), so several sequences merge."""

    def __init__(self) -> None:
        self.keys: list[tuple] = []  # one sequence only (``of``); empty after merging
        self.seg: dict[tuple[int, int], list[tuple]] = {}
        self.members: dict[tuple, list[int]] = {}
        self.n = 0  # sequences merged in (the next tag)

    @classmethod
    def of(cls, pids: list[int], rows: list[int] | None, tag: int = 0) -> Rows:
        r = cls()
        if not rows:
            return r
        r.keys = [(tag, int(x)) for x in rows]
        for p, key in zip(pids, r.keys, strict=True):
            r.members.setdefault(key, []).append(p)
        for i in range(len(pids) - 1):
            r.seg.setdefault((pids[i], pids[i + 1]), []).append((r.keys[i], r.keys[i + 1]))
        r.n = tag + 1
        return r

    def merge(self, other: Rows) -> None:
        for k, v in other.seg.items():
            self.seg.setdefault(k, []).extend(v)
        self.members.update(other.members)
        self.n = max(self.n, other.n)

    def __bool__(self) -> bool:
        return bool(self.members)

    def ends_at(self, m: int):
        """(row of visit m, row of visit m + 1) of a single sequence, or None (no rows)."""
        return (self.keys[m], self.keys[m + 1]) if m + 1 < len(self.keys) else None

    def change_at(self, m: int) -> bool:
        e = self.ends_at(m)
        return e is not None and e[0] != e[1]

    def visits(self, a: int, b: int) -> list[tuple]:
        """(row of a, row of b) of every visit of the move a -> b."""
        return self.seg.get((a, b), [])

    def change(self, a: int, b: int) -> bool:
        """Every visit of a -> b is a row change."""
        v = self.visits(a, b)
        return bool(v) and all(e[0] != e[1] for e in v)

    def direction(self, row, points: dict) -> np.ndarray | None:
        """First to last point of ``row``; None for a row of one point (or of coincident points)."""
        same = self.members.get(row, [])
        if len(same) < 2:
            return None
        d = np.asarray(points[same[-1]][0], float) - np.asarray(points[same[0]][0], float)
        return d if np.linalg.norm(d) > MIN_TRAVEL else None


@dataclass
class Direction:
    """What one sequence's TCP needs: ``mode`` (TRAVEL), ``overrun`` (m) and the pattern rows."""

    mode: str
    overrun: float = 0.0
    rows: Rows = field(default_factory=Rows)  # row patterns (by visit)

    @property
    def directional(self) -> bool:
        return self.mode != "free"


def row_pitch(scan_width_mm: float, overlap_pct: float) -> float:
    """Distance between rows (mm) for a stripe ``scan_width_mm`` wide overlapping ``overlap_pct`` %."""
    return float(scan_width_mm) * (1.0 - float(overlap_pct) / 100.0)


def spin_for(T: np.ndarray, d: np.ndarray, mode: str) -> float | None:
    """The spin (rad, about the tool z axis) that puts ``mode``'s axis of pose ``T`` along ``d``;
    None when ``d`` has no part in the tangent plane (v12 #1: no silent spin 0)."""
    R = T[:3, :3]
    z = R[:, 2]
    dt = np.asarray(d, float) - float(np.dot(d, z)) * z
    if np.linalg.norm(dt) < 1e-12:
        return None
    dl = R.T @ dt
    a = float(np.arctan2(dl[1], dl[0]))
    return a if mode == "along_x" else a - np.pi / 2


def offsets_for(T: np.ndarray, d: np.ndarray | None, mode: str) -> np.ndarray | None:
    """The two allowed spin offsets (the axis along ``d``, either way), wrapped to (-pi, pi]; None
    when the direction is undefined."""
    o = None if d is None else spin_for(T, d, mode)
    if o is None:
        return None
    return np.array([(o + np.pi) % (2 * np.pi) - np.pi, (o + 2 * np.pi) % (2 * np.pi) - np.pi])


def deviation_deg(T: np.ndarray, d: np.ndarray, mode: str) -> float:
    """Angle (deg) between ``mode``'s axis of ``T`` and the line of ``d`` in the tangent plane."""
    if mode == "free":
        return 0.0
    R = T[:3, :3]
    z = R[:, 2]
    dt = np.asarray(d, float) - float(np.dot(d, z)) * z
    n = np.linalg.norm(dt)
    if n < 1e-12:
        return 0.0
    axis = R[:, 0] if mode == "along_x" else R[:, 1]
    c = abs(float(np.dot(axis, dt / n)))
    return float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))


def _chord(pids: list[int], points: dict, m: int) -> np.ndarray:
    return np.asarray(points[pids[m + 1]][0], float) - np.asarray(points[pids[m]][0], float)


def travel(pids: list[int], points: dict, rows: Rows, m: int) -> np.ndarray | None:
    """Travel direction of segment m (pids[m] -> pids[m + 1]): along it on a row, the direction of
    the row being left on a row change (the tool steps sideways without turning). A move between
    coincident points (duplicate targets, v12 #1) takes the nearest segment's direction, before
    or after it; None when no segment of the sequence has one (the caller flags it)."""
    if rows and rows.change_at(m):
        d = rows.direction(rows.ends_at(m)[0], points)
        if d is not None:
            return d
    d = _chord(pids, points, m)
    if np.linalg.norm(d) > MIN_TRAVEL:
        return d
    for k in range(1, len(pids)):
        for j in (m - k, m + k):
            if 0 <= j < len(pids) - 1:
                d = _chord(pids, points, j)
                if np.linalg.norm(d) > MIN_TRAVEL:
                    return d
    return None


def row_change(rows, a: int, b: int) -> bool:
    """A row change on the move a -> b (``rows``: Rows, or point id -> row for a single pass)."""
    if isinstance(rows, Rows):
        return rows.change(a, b)
    return bool(rows) and rows.get(a) is not None and rows.get(a) != rows.get(b)


def overrun_poses(Ta: np.ndarray, Tb: np.ndarray, da: np.ndarray, db: np.ndarray, over: float, step: float):
    """(s, poses) of a row change done off the part: from a past its row's end by ``over`` along
    ``da``, across to ``over`` before b along ``db`` (b's row direction), back onto b. Positions
    are straight lines (no projection onto the part); orientations are those of a then b."""

    def unit(v):
        v = np.asarray(v, float)
        return v / max(np.linalg.norm(v), 1e-12)

    pa, pb = Ta[:3, 3], Tb[:3, 3]
    ea, eb = pa + over * unit(da), pb - over * unit(db)
    legs = [(pa, ea, Ta), (ea, eb, None), (eb, pb, Tb)]
    out = []
    for p0, p1, R in legs:
        n = max(1, int(np.ceil(np.linalg.norm(p1 - p0) / step)))
        for s in np.arange(1, n + 1) / n:
            T = np.array(R if R is not None else (Ta if s < 0.5 else Tb), float)
            T[:3, 3] = p0 + s * (p1 - p0)
            out.append(T)
    s = np.arange(1, len(out) + 1) / len(out)
    return s, out


def flag(samples: list, points: dict, d: Direction, tol_deg: float = TOL_DEG) -> int:
    """Mark surface samples (kind "") whose axis is off the travel direction (issue ``direction``);
    returns how many."""
    if not d.directional:
        return 0
    n = 0
    for p in samples:
        if p.kind or p.q is None or p.issue or p.a == p.b or p.a not in points or p.b not in points:
            continue
        if row_change(d.rows, p.a, p.b):  # sideways by design (the row being left sets the axis)
            continue
        chord = np.asarray(points[p.b][0], float) - np.asarray(points[p.a][0], float)
        if np.linalg.norm(chord) <= MIN_TRAVEL:  # coincident points: optimize_sequence used its
            continue  # neighbours' direction, or flagged the sample itself (v12 #1)
        # the same move may be a row change on another visit (cross-hatch): either way is right
        dirs = [chord] + [
            r
            for e in (d.rows.visits(p.a, p.b) if isinstance(d.rows, Rows) else [])
            if e[0] != e[1] and (r := d.rows.direction(e[0], points)) is not None
        ]
        if min(deviation_deg(p.T_part_tcp, v, d.mode) for v in dirs) > tol_deg:
            p.issue = "direction"
            n += 1
    return n


def flag_plan(plan, points: dict, direction, rows: Rows) -> int:
    """``flag`` every station's surface samples with their TCP's Direction (``direction(tcp)``)."""
    n = 0
    for st in plan.stations:
        by: dict[str, list] = {}
        for p in st.samples:
            by.setdefault(p.tcp, []).append(p)
        for tcp, smps in by.items():
            d = direction(tcp)
            if d is not None and d.directional:
                n += flag(smps, points, Direction(d.mode, d.overrun, rows))
    return n


def directional_sequence(tr, k, sq, results, points, label, ref, lift, leave, d: Direction, rows_all: Rows):
    """core.paths.trace: one sequence of a directional TCP (core.path_spin.optimize_sequence with
    the spin fixed by the travel), or None (no IK at the required spins: the tracer's planned spins,
    flagged by ``flag_plan``). ``rows_all`` collects the pattern rows for the flags."""
    from core.path_spin import optimize_sequence

    d.rows = Rows.of(sq.pids, sq.rows, tag=rows_all.n)
    rows_all.merge(d.rows)
    return optimize_sequence(
        tr, k, sq.zone, sq.pids, results, points, label, ref, lambda p: None, sq.lifts, lift, leave, d
    )


def lead_in_out(
    samples: list, pids: list[int], points: dict, rows: Rows, d: Direction, step: float, make, q_plan
):
    """core.path_spin.optimize_sequence: the first row starts ``d.overrun`` before its first point
    and the last row ends that far past its last point (``overrun`` samples, straight, off the
    part), so the tool runs over the part's edges at both ends too. ``make`` builds a sample."""

    def unit(v):
        return np.asarray(v, float) / max(np.linalg.norm(v), 1e-12)

    e0, e1 = rows.ends_at(0), rows.ends_at(len(pids) - 2)
    first = rows.direction(e0[0], points) if e0 else None
    last = rows.direction(e1[1], points) if e1 else None
    if first is None:
        first = travel(pids, points, Rows(), 0)
    if last is None:
        last = travel(pids, points, Rows(), len(pids) - 2)
    if first is None or last is None:  # every point coincides: no lead-in direction
        return samples
    s0, sN = samples[0], samples[-1]
    n = max(1, int(np.ceil(d.overrun / step)))
    lead, tail = [], []
    for i in range(n):  # from the lead-in start towards the first point
        T = np.array(s0.T, float)
        T[:3, 3] = s0.T[:3, 3] - (n - i) / n * d.overrun * unit(first)
        lead.append(make(pids[0], pids[0], 0.0, T, None, q_plan[pids[0]], "overrun", off=s0.off))
    for i in range(1, n + 1):  # past the last point
        T = np.array(sN.T, float)
        T[:3, 3] = sN.T[:3, 3] + i / n * d.overrun * unit(last)
        tail.append(make(sN.a, sN.b, 1.0, T, None, sN.q_ref, "overrun", off=sN.off))
    return lead + samples + tail
