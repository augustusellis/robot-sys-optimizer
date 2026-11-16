"""Gantry / rail placement (v9, issue #9): where the whole carrier frame goes relative to the part,
before the stops are planned.

Candidate frames:
    rail    the track along the part's long axis, on either side, its line at each **standoff**
            (``d_min`` .. ``d_max`` from the part outline, step ``step``)
    gantry  the frame turned along or across the part's long axis, centred on the part and moved
            by ``shift_step`` up to ``shift`` along both axes
each with the robot mount height (``T_mount`` z, the **z range** ``z_lo`` .. ``z_hi`` in ``z_steps``
values): how high the robot sits on the carriage (rail riser) or how low it hangs (gantry).

A candidate is kept when the carrier's fixed bodies (track, legs, beams) keep the base's part
standoff (``carrier_part_min``, 3D distance), stay out of the floor keep-outs and away from the
stay-out zones (v10 #6: the keep-out clearance) and its stops can exist (the planner's admissible
stations: standoff, keep-outs, travel). It is scored like the planner's first step: the targets the
admissible stops cover (reach maps) and the fewest stops that cover them (greedy set cover); ranked
by (targets not covered, stops, mean manipulability). The best few are then planned in full by the
caller (ui.frame_ui).
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field

import numpy as np

from core.schema import Base, Pose
from core.transforms import pose_to_matrix


@dataclass
class FrameSettings:
    d_min: float = 0.3  # m, standoff of the track line from the part outline (rail)
    d_max: float = 1.2
    step: float = 0.15  # m
    shift: float = 0.6  # m, gantry: the frame moved this far off the part's centre, both ways
    shift_step: float = 0.3  # m
    z_lo: float | None = None  # m, robot mount height (T_mount z); None: the current one
    z_hi: float | None = None
    z_steps: int = 3
    candidate_step: float = 0.25  # stops along the travel, for scoring (the full plan uses its own)
    reach_threshold: float = 0.5
    top: int = 3  # frames planned in full by the caller
    # v10 T18
    distinct_m: float = 0.6  # top frames scoring alike must be this far apart (or turned)
    refine: bool = True  # compass search around the best grid frame (along, across, mount height)
    refine_evals: int = 30
    beam_lo: float | None = None  # m, gantry frame (beam) height; None: the current one
    beam_hi: float | None = None
    beam_steps: int = 3


@dataclass
class FrameScore:
    pose: Pose
    mount_z: float
    uncovered: int
    stops: int
    manip: float
    reason: str = ""
    base: Base | None = field(default=None, repr=False)
    refined: bool = False  # v10 T18: found by the continuous refinement

    def key(self) -> tuple:
        return (self.uncovered, self.stops, -self.manip)


def _axes(part_fp) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(centre, long axis u, normal n) of the part footprint's minimum rectangle."""
    rect = part_fp.minimum_rotated_rectangle
    c4 = np.asarray(rect.exterior.coords)[:4]
    edges = np.roll(c4, -1, axis=0) - c4
    u = edges[int(np.argmax(np.linalg.norm(edges, axis=1)))]
    u = u / max(np.linalg.norm(u), 1e-12)
    return np.asarray(rect.centroid.coords[0]), u, np.array([-u[1], u[0]])


def _mount_heights(base: Base, st: FrameSettings) -> list[float]:
    z0 = float(base.T_mount.xyz[2])
    lo = z0 if st.z_lo is None else st.z_lo
    hi = z0 if st.z_hi is None else st.z_hi
    if hi < lo:
        lo, hi = hi, lo
    n = max(1, int(st.z_steps)) if hi > lo + 1e-9 else 1
    return [float(v) for v in np.linspace(lo, hi, n)]


def candidate_frames(base: Base, part_fp, st: FrameSettings) -> list[tuple[Pose, float]]:
    """(frame pose, robot mount height) candidates for the base's carrier (see the docstring)."""
    centre, u, n = _axes(part_fp)
    z = base.pose.xyz[2]
    roll, pitch = base.pose.rpy[0], base.pose.rpy[1]
    pts = np.asarray(part_fp.exterior.coords)
    out: list[Pose] = []
    if base.carrier == "rail":
        yaw = float(np.arctan2(u[1], u[0]))
        for side in (1.0, -1.0):
            half = float(np.max((pts - centre) @ (side * n)))
            for d in np.arange(st.d_min, st.d_max + 1e-9, st.step):
                o = centre + side * (half + d) * n
                out.append(Pose(xyz=(float(o[0]), float(o[1]), z), rpy=(roll, pitch, yaw)))
    elif base.carrier == "gantry":
        offs = np.arange(-st.shift, st.shift + 1e-9, st.shift_step) if st.shift > 0 else np.array([0.0])
        for axis in (u, n):  # the frame's x along, then across the part
            ax = np.array([-axis[1], axis[0]])
            yaw = float(np.arctan2(axis[1], axis[0]))
            for a in offs:
                for b in offs:
                    o = centre + a * axis + b * ax
                    out.append(Pose(xyz=(float(o[0]), float(o[1]), z), rpy=(roll, pitch, yaw)))
    else:
        raise ValueError("frame placement is for a rail or a gantry")
    heights = _mount_heights(base, st)
    if base.carrier == "gantry" and st.beam_lo is not None and st.beam_hi is not None:  # v10 T18
        lo, hi = sorted((st.beam_lo, st.beam_hi))
        zs = np.linspace(lo, hi, max(1, int(st.beam_steps))) if hi > lo + 1e-9 else [lo]
        out = [p.model_copy(update={"xyz": (p.xyz[0], p.xyz[1], float(z))}) for p in out for z in zs]
    return [(p, h) for p in out for h in heights]


def with_frame(base: Base, pose: Pose, mount_z: float) -> Base:
    """``base`` on frame ``pose`` with the robot mount at ``mount_z``. v10 T18: a frame at another
    height (a gantry's beams) keeps its legs on the floor: fixed bodies standing on the floor
    (bottom at world z = 0) get taller by the change and their pose drops by it."""
    m = base.T_mount
    dz = float(pose.xyz[2] - base.pose.xyz[2])
    upd = {
        "pose": pose,
        "align_to_part": False,  # the placement chose the frame
        "T_mount": m.model_copy(update={"xyz": (m.xyz[0], m.xyz[1], float(mount_z))}),
    }
    if abs(dz) > 1e-9 and base.bodies:
        upd["bodies"] = [stretch_leg(b, base.pose.xyz[2], dz) for b in base.bodies]
    return base.model_copy(update=upd)


def stretch_leg(b, frame_z: float, dz: float):
    """A fixed box / cylinder whose bottom is on the floor, made ``dz`` taller (frame raised)."""
    if b.attach != "fixed" or b.shape not in ("box", "cylinder") or abs(frame_z + b.pose.xyz[2]) > 1e-3:
        return b
    h = (b.size[2] if b.shape == "box" else b.height) + dz
    if h <= 0.01:
        raise ValueError(f"leg {b.name} would be shorter than 10 mm")
    pose = b.pose.model_copy(update={"xyz": (b.pose.xyz[0], b.pose.xyz[1], b.pose.xyz[2] - dz)})
    if b.shape == "box":
        return b.model_copy(update={"size": (b.size[0], b.size[1], h), "pose": pose})
    return b.model_copy(update={"height": h, "pose": pose})


def fixed_clear(c, obstacles=None) -> str:
    """v10 #6: why the carrier's fixed bodies (track, legs, beams) may not stand there: in a floor
    keep-out (with the keep-out clearance) or too close to a stay-out zone; "" = fine."""
    from core.standoff import Zones
    from core.stayout import carrier_clear

    fixed = c.static_bodies()
    if not fixed:
        return ""
    fp = c._hulls(fixed)
    zones = Zones(c.base, None)
    if zones.keepout_zone is not None and fp is not None and fp.intersects(zones.keepout_zone):
        return "fixed bodies in a keep-out"
    ok, what = carrier_clear(fixed, fp, list(obstacles or []), c.base.standoff_m.carrier_env_min)
    return "" if ok else f"fixed bodies too close to the {what}"


def sweep_clear(c, part_mesh, T_part, step: float = 0.2) -> str:
    """v11 #2: why a gantry's bridge (bodies on the X axis) cannot travel: somewhere along the X
    travel it comes closer to the part than the carrier's minimum standoff (a floor gantry whose
    rails straddle the part); "" = fine (e.g. an overhead bridge above the part)."""
    from core.standoff import distance_3d

    if c.kind != "gantry" or "x" not in c.dofs:
        return ""
    lo, hi = c.bounds[c.dofs.index("x")]
    mid = c.clip({})
    for x in np.linspace(lo, hi, max(2, int(np.ceil((hi - lo) / step)) + 1)):
        bodies = c._bodies("x_axis", c.T_world_axis({**mid, "x": float(x)}))
        if bodies and distance_3d(bodies, part_mesh, T_part) < c.base.standoff_m.carrier_part_min - 1e-6:
            return "the bridge would travel through the part"
    return ""


def lattice_anchor(c, step: float) -> dict[str, float]:
    """v12 #20: DOF offsets that put a rail's / gantry's candidate stops on one world lattice
    (``step`` apart along the frame's axes, through the world origin): frames shifted against each
    other then share stop poses, and ``score_frame`` reuses their coverage rows."""
    T = c.T_frame
    out = {}
    for dof in (d for d in c.dofs if d in ("s", "x", "y")):
        axis = T[:3, 0] if dof in ("s", "x") else T[:3, 1]
        out[dof] = float((-(T[:3, 3] @ axis)) % step)
    return out


def _cached_accept(sites, memo: dict):
    """``sites.accept`` remembered by the moving bodies' world poses (carriage, X axis / bridge,
    lift): the same for every frame of one ranking (only the fixed bodies differ between frames:
    legs, checked by ``score_frame`` itself)."""
    accept, c = sites.accept, sites.c

    def run(v: dict):
        w = sites.full(v)
        if any(not lo - 1e-9 <= w[d] <= hi + 1e-9 for d, (lo, hi) in zip(c.dofs, c.bounds, strict=True)):
            return False, 1.0
        k = (
            tuple(np.round(c.T_world_carriage(w)[:3, :4], 6).ravel()),
            tuple(np.round(c.T_world_axis(w)[:3, :4], 6).ravel()),
            round(float(w.get("lift", 0.0)), 6),
        )
        if k not in memo:
            memo[k] = accept(v)
        return memo[k]

    return run


def _coverage(rmap, t, Ts: np.ndarray, cache: dict | None):
    """(score, mu) of every base pose in ``Ts`` against the targets; rows of base poses seen before
    come from ``cache`` (key: the pose rounded to 1 um / 1e-9: the map depends on nothing else)."""
    from opt.reachability import coverage_matrix

    keys = [tuple(np.round(T[:3, :4], 6).ravel()) for T in Ts]
    todo = [i for i, k in enumerate(keys) if cache is None or k not in cache]
    score = np.zeros((len(Ts), len(t.ids)))
    mu = np.zeros_like(score)
    rots = np.round(Ts[:, :3, 2], 6)
    todo_set = set(todo)
    for key in np.unique(rots[todo], axis=0) if todo else []:
        rows = [i for i in np.flatnonzero(np.all(rots == key, axis=1)) if i in todo_set]
        sc, m = coverage_matrix(rmap, t.P_world, t.N_world, t.cones, Ts[rows, :3, 3], Ts[rows[0], :3, :3])
        score[rows], mu[rows] = sc, m
    for i, k in enumerate(keys):
        if cache is None:
            continue
        if i in todo_set:
            cache[k] = (score[i].copy(), mu[i].copy())
        else:
            score[i], mu[i] = cache[k]
    return score, mu


def score_frame(
    base: Base,
    rmap,
    t,
    part_mesh,
    T_part,
    st: FrameSettings,
    resolve=None,
    obstacles=None,
    cache: dict | None = None,
) -> FrameScore:
    """How well a frame's stops can cover the targets (the planner's first step). ``cache``
    (v12 #20): coverage rows by base pose, shared by the frames of one ranking."""
    from core.carrier import Carrier
    from core.standoff import distance_3d
    from opt.set_cover import greedy_cover
    from opt.sites import CarrierSites

    pose, mount_z = base.pose, float(base.T_mount.xyz[2])
    c = Carrier(base, resolve=resolve)
    fixed = c.static_bodies()
    if fixed and distance_3d(fixed, part_mesh, T_part) < base.standoff_m.carrier_part_min - 1e-6:
        return FrameScore(pose, mount_z, len(t.ids), 0, 0.0, "fixed bodies too close to the part", base)
    why = fixed_clear(c, obstacles) or sweep_clear(c, part_mesh, T_part)
    if why:
        return FrameScore(pose, mount_z, len(t.ids), 0, 0.0, why, base)
    sites = CarrierSites(c, part_mesh, T_part, obstacles=obstacles)
    if cache is not None:  # v12 #20: a stop's acceptance depends only on where its moving bodies are
        sites.accept = _cached_accept(sites, cache.setdefault("accept", {}))
    cand = sites.candidates(t.P_world, st.candidate_step, anchor=lattice_anchor(c, st.candidate_step))
    if not cand:
        return FrameScore(pose, mount_z, len(t.ids), 0, 0.0, "no admissible stop", base)
    Ts = np.array([sites.T(v) for v in cand])
    score, mu = _coverage(rmap, t, Ts, cache)
    covered = score >= st.reach_threshold
    reach = covered.any(axis=0)
    chosen = greedy_cover(covered, mu / max(rmap.max_manip, 1e-12), len(cand))
    best_mu = np.where(covered, mu, 0.0).max(axis=0)
    manip = float(best_mu[reach].mean() / max(rmap.max_manip, 1e-12)) if reach.any() else 0.0
    return FrameScore(pose, mount_z, int((~reach).sum()), len(chosen), manip, "", base)


def rank_frames(
    base: Base,
    rmap,
    t,
    part_mesh,
    T_part,
    part_fp,
    st: FrameSettings,
    resolve=None,
    obstacles=None,
    progress: Callable[[str, float], None] | None = None,
    cancelled: Callable[[], bool] | None = None,
) -> list[FrameScore]:
    """Every candidate frame scored, best first (failed ones last, with their reason)."""
    from core.feasibility import CancelledError

    cands = candidate_frames(base, part_fp, st)
    out = []
    cache: dict = {}  # v12 #20: coverage rows by base pose, shared by every frame
    for k, (pose, h) in enumerate(cands):
        if cancelled is not None and cancelled():
            raise CancelledError()
        if progress is not None:
            progress(f"frame {k + 1} / {len(cands)}", 0.5 * k / max(len(cands), 1))
        f = score_frame(with_frame(base, pose, h), rmap, t, part_mesh, T_part, st, resolve, obstacles, cache)
        out.append(f)
    out.sort(key=lambda f: (bool(f.reason), f.key()))
    if st.refine and out and not out[0].reason:  # v10 T18: continuous refinement of the best one

        def score(pose, h):
            if cancelled is not None and cancelled():
                raise CancelledError()
            return score_frame(
                with_frame(base, pose, h), rmap, t, part_mesh, T_part, st, resolve, obstacles, cache
            )

        best = refine_frame(out[0], score, st, progress)
        if best.key() < out[0].key():
            best.refined = True
            out.insert(0, best)
    return out


def refine_frame(f: FrameScore, score, st: FrameSettings, progress=None) -> FrameScore:
    """Compass search from frame ``f``: steps along / across the frame's x axis (and the mount
    height when it may change), halved when no step improves the score, ``st.refine_evals`` at most."""
    T = pose_to_matrix(f.pose)
    ax, ay = T[:2, 0], T[:2, 1]
    s0 = (st.step if f.base is not None and f.base.carrier == "rail" else st.shift_step) / 2
    dz0 = (abs((st.z_hi or 0) - (st.z_lo or 0)) / max(int(st.z_steps), 1)) / 2 if st.z_lo is not None else 0.0
    best, step, dz, n = f, s0, dz0, 0
    while n < st.refine_evals and step > 0.01:
        moved = False
        dirs = [(ax, 0.0), (-ax, 0.0), (ay, 0.0), (-ay, 0.0)]
        if dz > 0.005:
            dirs += [(np.zeros(2), 1.0), (np.zeros(2), -1.0)]
        for d, sz in dirs:
            if n >= st.refine_evals:
                break
            x, y, z = best.pose.xyz
            pose = best.pose.model_copy(update={"xyz": (float(x + step * d[0]), float(y + step * d[1]), z)})
            h = best.mount_z + sz * dz
            if (
                st.z_lo is not None
                and st.z_hi is not None
                and not min(st.z_lo, st.z_hi) - 1e-9 <= h <= max(st.z_lo, st.z_hi) + 1e-9
            ):
                continue
            n += 1
            if progress is not None:
                progress(f"refining the best frame ({n})", 0.5 + 0.1 * n / st.refine_evals)
            c = score(pose, h)
            if not c.reason and c.key() < best.key():
                best, moved = c, True
                break
        if not moved:
            step, dz = step / 2, dz / 2
    return best


def pick_top(ranked: list[FrameScore], top: int, min_dist: float) -> list[FrameScore]:
    """The best ``top`` usable frames, skipping one that scores like an already chosen frame and
    stands within ``min_dist`` of it with the same heading (v9 planned two such frames 300 mm apart
    with identical results)."""
    out: list[FrameScore] = []
    for f in ranked:
        if f.reason:
            continue
        T = pose_to_matrix(f.pose)
        same = False
        for g in out:
            G = pose_to_matrix(g.pose)
            close = np.linalg.norm(T[:3, 3] - G[:3, 3]) < min_dist and abs(f.mount_z - g.mount_z) < 1e-6
            turned = abs(np.arctan2(T[1, 0], T[0, 0]) - np.arctan2(G[1, 0], G[0, 0])) > 1e-3
            if close and not turned and f.key()[:2] == g.key()[:2]:
                same = True
                break
        if not same:
            out.append(f)
        if len(out) >= max(1, top):
            break
    return out


def frame_text(f: FrameScore) -> str:
    T = pose_to_matrix(f.pose)
    yaw = np.degrees(np.arctan2(T[1, 0], T[0, 0]))
    x, y, z = T[0, 3] * 1000, T[1, 3] * 1000, f.mount_z * 1000
    beam = f", beam z {T[2, 3] * 1000:.0f} mm" if f.base is not None and f.base.carrier == "gantry" else ""
    tag = " (refined)" if f.refined else ""
    return f"x {x:.0f}, y {y:.0f} mm, yaw {yaw:.0f}°, mount z {z:.0f} mm{beam}{tag}"
