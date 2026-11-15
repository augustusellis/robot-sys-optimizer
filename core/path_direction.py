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


@dataclass
class Direction:
    """What one sequence's TCP needs: ``mode`` (TRAVEL), ``overrun`` (m) and the pattern rows."""

    mode: str
    overrun: float = 0.0
    rows: dict[int, int] = field(default_factory=dict)  # point id -> row number (row patterns)

    @property
    def directional(self) -> bool:
        return self.mode != "free"


def row_pitch(scan_width_mm: float, overlap_pct: float) -> float:
    """Distance between rows (mm) for a stripe ``scan_width_mm`` wide overlapping ``overlap_pct`` %."""
    return float(scan_width_mm) * (1.0 - float(overlap_pct) / 100.0)


def spin_for(T: np.ndarray, d: np.ndarray, mode: str) -> float:
    """The spin (rad, about the tool z axis) that puts ``mode``'s axis of pose ``T`` along ``d``."""
    R = T[:3, :3]
    z = R[:, 2]
    dt = np.asarray(d, float) - float(np.dot(d, z)) * z
    if np.linalg.norm(dt) < 1e-12:
        return 0.0
    dl = R.T @ dt
    a = float(np.arctan2(dl[1], dl[0]))
    return a if mode == "along_x" else a - np.pi / 2


def offsets_for(T: np.ndarray, d: np.ndarray, mode: str) -> np.ndarray:
    """The two allowed spin offsets (the axis along ``d``, either way), wrapped to (-pi, pi]."""
    o = spin_for(T, d, mode)
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


def row_direction(pids: list[int], rows: dict[int, int], points: dict, pid: int) -> np.ndarray | None:
    """Direction of the row that ``pid`` is on (first to last of its row's points in ``pids``)."""
    r = rows.get(pid)
    same = [p for p in pids if rows.get(p) == r] if r is not None else []
    if len(same) < 2:
        return None
    d = np.asarray(points[same[-1]][0], float) - np.asarray(points[same[0]][0], float)
    return d if np.linalg.norm(d) > 1e-12 else None


def travel(pids: list[int], points: dict, rows: dict[int, int], m: int) -> np.ndarray:
    """Travel direction of segment m (pids[m] -> pids[m + 1]): along it on a row, the direction of
    the row being left on a row change (the tool steps sideways without turning)."""
    a, b = pids[m], pids[m + 1]
    if rows and rows.get(a) is not None and rows.get(a) != rows.get(b):
        d = row_direction(pids, rows, points, a)
        if d is not None:
            return d
    return np.asarray(points[b][0], float) - np.asarray(points[a][0], float)


def row_change(rows: dict[int, int], a: int, b: int) -> bool:
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
        pids = [p.a, p.b]
        dirv = travel(pids, points, d.rows, 0) if d.rows else np.asarray(points[p.b][0]) - points[p.a][0]
        if deviation_deg(p.T_part_tcp, dirv, d.mode) > tol_deg:
            p.issue = "direction"
            n += 1
    return n


def flag_plan(plan, points: dict, direction, rows: dict[int, int]) -> int:
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


def directional_sequence(tr, k, sq, results, points, label, ref, lift, leave, d: Direction, rows_all: dict):
    """core.paths.trace: one sequence of a directional TCP (core.path_spin.optimize_sequence with
    the spin fixed by the travel), or None (no IK at the required spins: the tracer's planned spins,
    flagged by ``flag_plan``). ``rows_all`` collects the pattern rows for the flags."""
    from core.path_spin import optimize_sequence

    d.rows = dict(zip(sq.pids, sq.rows, strict=True)) if sq.rows else {}
    rows_all.update(d.rows)
    return optimize_sequence(
        tr, k, sq.zone, sq.pids, results, points, label, ref, lambda p: None, sq.lifts, lift, leave, d
    )


def lead_in_out(
    samples: list, pids: list[int], points: dict, rows: dict, d: Direction, step: float, make, q_plan
):
    """core.path_spin.optimize_sequence: the first row starts ``d.overrun`` before its first point
    and the last row ends that far past its last point (``overrun`` samples, straight, off the
    part), so the tool runs over the part's edges at both ends too. ``make`` builds a sample."""

    def unit(v):
        return np.asarray(v, float) / max(np.linalg.norm(v), 1e-12)

    first = row_direction(pids, rows, points, pids[0]) if rows else None
    last = row_direction(pids, rows, points, pids[-1]) if rows else None
    if first is None:
        first = np.asarray(points[pids[1]][0], float) - np.asarray(points[pids[0]][0], float)
    if last is None:
        last = np.asarray(points[pids[-1]][0], float) - np.asarray(points[pids[-2]][0], float)
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
