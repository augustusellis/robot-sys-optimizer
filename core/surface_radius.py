"""Local surface radius at the targets vs the tool's minimum radius (v10 T12).

A tool that must sit on the surface (a sanding pad, a roller) cannot follow a curve tighter than
its **minimum surface radius** (``schema.TCP.min_radius_mm``). At every target the surface is
sampled over the tool's contact footprint: a disc of points in the target's tangent plane (rings
out to the footprint radius) is projected onto the part mesh (closest point, front-facing faces
only), and a quadric h = a u^2 + b uv + c v^2 + d u + e v + f is fitted to the heights h along the
outward normal. The principal curvatures are the eigenvalues of [[2a, b], [b, 2c]]: positive =
**concave** (the surface rises around the tool: an inside fillet, a bowl), negative = **convex** (an
outside edge, a cylinder seen from outside). Radius = 1 / curvature. A fit over a wide footprint
averages a tight inside corner away, so the concave radius is also the tightest d^2 / 2h of the
samples that rise above the tangent plane (d = distance in the plane, h = rise; exact on a sphere):
a flat pad cannot sit where the surface rises into it. A 90 deg fillet of radius R then reads about
R at its middle and less towards the walls, for any footprint. Mesh-resolution independent (the
disc, not the vertices, is sampled).

``check(...)``: a point is **incompatible** with a TCP when the radius the limit applies to
(concave by default, convex, or both) is below the minimum; with several TCPs allowed for a point
it is compatible if any of them fits. Pure geometry: no viser.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
import trimesh

APPLIES = ("concave", "convex", "both")
FOOTPRINT_MIN, FOOTPRINT_MAX = 0.005, 0.15  # m
RINGS, PER_RING = 4, 12
RISE = 1e-5  # m: a rise above the tangent plane smaller than this is mesh noise
FLAT = 0.2  # 1/m: below this curvature (radius above 5 m, e.g. facet noise) the surface is flat


@dataclass
class Radius:
    concave: float  # m (inf = flat or convex only)
    convex: float
    unsure: bool = False  # v12 #4: face normals around the point disagree: not measured


@dataclass
class Fit:
    """Result of ``check``: per point the radii and, for incompatible points, the limit broken."""

    radii: dict[int, Radius] = field(default_factory=dict)
    bad: dict[int, tuple[str, float, float, str]] = field(default_factory=dict)  # pid -> (tcp, r, min, kind)
    unsure: list[int] = field(default_factory=list)  # v12 #4: inconsistent face normals there

    def lines(self, limit: int = 12) -> str:
        """'2 points not compatible with the tool's minimum radius 25 mm: surface radius 10 mm at
        points 7, 8' (one clause per TCP / minimum), or ''."""
        out = []
        if self.unsure:
            ids = ", ".join(str(p) for p in self.unsure[:limit])
            more = f" and {len(self.unsure) - limit} more" if len(self.unsure) > limit else ""
            out.append(
                f"{len(self.unsure)} point(s) where the part's face normals disagree (flipped faces?): "
                f"surface radius not measured at points {ids}{more}"
            )
        return "; ".join(out + self._bad_lines(limit))

    def _bad_lines(self, limit: int) -> list[str]:
        if not self.bad:
            return []
        by: dict[tuple[str, float], list[tuple[int, float, str]]] = {}
        for pid, (tcp, r, rmin, kind) in sorted(self.bad.items()):
            by.setdefault((tcp, rmin), []).append((pid, r, kind))
        out = []
        for (tcp, rmin), pts in by.items():
            where = ", ".join(f"{p} ({1000 * r:.0f} mm {k})" for p, r, k in pts[:limit])
            more = f" and {len(pts) - limit} more" if len(pts) > limit else ""
            out.append(
                f"{len(pts)} point(s) not compatible with the tool's minimum radius {1000 * rmin:.0f} mm "
                f"(TCP {tcp}): surface radius at points {where}{more}"
            )
        return out


def disc(radius: float) -> np.ndarray:
    """(k, 2) tangent-plane offsets: the centre + RINGS rings of PER_RING points out to ``radius``."""
    pts = [np.zeros(2)]
    for i in range(1, RINGS + 1):
        a = np.linspace(0, 2 * np.pi, PER_RING, endpoint=False) + (i % 2) * np.pi / PER_RING
        pts.append(np.c_[np.cos(a), np.sin(a)] * radius * i / RINGS)
    return np.vstack(pts)


def _frame(n: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    a = np.where(np.abs(n[:, :1]) < 0.9, [[1.0, 0, 0]], [[0, 1.0, 0]])
    u = np.cross(n, a)
    u /= np.linalg.norm(u, axis=1, keepdims=True)
    return u, np.cross(n, u)


def surface_radii(mesh: trimesh.Trimesh, P: np.ndarray, N: np.ndarray, footprint: float) -> list[Radius]:
    """Concave / convex radius at each point over a footprint of radius ``footprint`` (m)."""
    P, N = np.asarray(P, float), np.asarray(N, float)
    N = N / np.linalg.norm(N, axis=1, keepdims=True)
    F = float(np.clip(footprint, FOOTPRINT_MIN, FOOTPRINT_MAX))
    D = disc(F)
    U, V = _frame(N)
    Q = P[:, None, :] + D[None, :, :1] * U[:, None, :] + D[None, :, 1:] * V[:, None, :]  # (n, k, 3)
    on, _, tri = trimesh.proximity.closest_point(mesh, Q.reshape(-1, 3))
    on, tri = on.reshape(Q.shape), tri.reshape(Q.shape[:2])
    fn = mesh.face_normals[tri]
    out = []
    for i in range(len(P)):
        d = on[i] - P[i]
        # v12 #4: front faces relative to the face under the point, so a mesh whose normals are
        # flipped (all of them, or locally) is measured like any other
        c = np.einsum("kj,j->k", fn[i], N[i])
        c = -c if c[0] < 0 else c
        near = np.linalg.norm(d, axis=1) <= 1.5 * F
        keep = (c > 0.2) & near
        u, v, h = d[keep] @ U[i], d[keep] @ V[i], d[keep] @ N[i]
        if len(h) < 8:  # too few front samples: inconsistent normals when the back ones are many
            out.append(Radius(np.inf, np.inf, unsure=int(np.sum((c < -0.2) & near)) >= 8))
            continue
        A = np.c_[u * u, u * v, v * v, u, v, np.ones_like(u)]
        a, b, c = np.linalg.lstsq(A, h, rcond=None)[0][:3]
        k = np.linalg.eigvalsh([[2 * a, b], [b, 2 * c]])
        kc, kx = max(k.max(), 0.0), max(-k.min(), 0.0)
        rc = 1 / kc if kc > FLAT else np.inf
        rise = h > RISE  # the surface rising into the tool's plane: radius d^2 / 2h (exact on a sphere)
        if rise.any():
            rc = min(rc, float(np.min((u[rise] ** 2 + v[rise] ** 2) / (2 * h[rise]))))
        out.append(Radius(rc if rc < 1 / FLAT else np.inf, 1 / kx if kx > FLAT else np.inf))
    return out


def footprint_of(tool_model, tcp: str, default: float) -> float:
    """Radius (m) of a TCP's contact geometry around its axis: the contact links' and contact faces'
    vertices within 5 mm of the TCP plane; ``default`` when the TCP has no contact geometry."""
    t = next((x for x in tool_model.ee.tcps if x.name == tcp), None)
    if t is None:
        return default
    from core.transforms import pose_to_matrix

    T_tcp_head = np.linalg.inv(pose_to_matrix(t.T_head_tcp))
    r = 0.0
    for hl in tool_model.links:
        verts = []
        if hl.name in t.contact_links:
            verts.append(hl.mesh.vertices)
        ids = [i for i in t.contact_faces.get(hl.name, []) if i < len(hl.mesh.faces)]
        if ids:
            verts.append(hl.mesh.vertices[np.unique(hl.mesh.faces[ids])])
        for V in verts:
            X = V @ (T_tcp_head @ hl.T_head_link)[:3, :3].T + (T_tcp_head @ hl.T_head_link)[:3, 3]
            near = np.abs(X[:, 2]) < 0.005
            if near.any():
                r = max(r, float(np.max(np.linalg.norm(X[near, :2], axis=1))))
    return r if r > 0 else default


def limit_radius(rad: Radius, applies: str) -> tuple[float, str]:
    """The radius a limit of kind ``applies`` compares against, and which kind it was."""
    if applies == "convex":
        return rad.convex, "convex"
    if applies == "both" and rad.convex < rad.concave:
        return rad.convex, "convex"
    return rad.concave, "concave"


def check(mesh, ids, P, N, groups, tool_model) -> Fit:
    """Radii and incompatible points for the targets (part frame). ``groups``: per point its
    TargetGroup (its ``tcps`` restrict the TCPs; empty = every TCP)."""
    fit = Fit()
    tcps = [t for t in tool_model.ee.tcps if t.min_radius_mm]
    if not tcps or len(P) == 0:
        return fit
    names = [t.name for t in tool_model.ee.tcps]
    per_tcp = {}
    for t in tcps:
        rmin = t.min_radius_mm / 1000
        per_tcp[t.name] = (t, surface_radii(mesh, P, N, footprint_of(tool_model, t.name, rmin)))
    for i, pid in enumerate(np.asarray(ids)):
        allowed = list(groups[i].tcps) or names
        worst = None
        for name in allowed:
            if name not in per_tcp:  # a TCP without a minimum fits anywhere
                worst = None
                break
            t, radii = per_tcp[name]
            r, kind = limit_radius(radii[i], t.radius_applies)
            if r >= t.min_radius_mm / 1000:
                worst = None
                break
            worst = worst or (name, r, t.min_radius_mm / 1000, kind)
        first = per_tcp.get(allowed[0]) if allowed else None
        if first is not None:
            fit.radii[int(pid)] = first[1][i]
        if any(per_tcp[n][1][i].unsure for n in allowed if n in per_tcp):
            fit.unsure.append(int(pid))
        if worst is not None:
            fit.bad[int(pid)] = worst
    return fit
