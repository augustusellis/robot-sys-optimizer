"""Small plan-view pictures for the report (v10 T22): the part's floor outline and a setup's stops,
as inline SVG (no browser or 3D render needed; the report stays self-contained)."""

from __future__ import annotations

from html import escape

import numpy as np


def plan_view(outline: np.ndarray, stops: np.ndarray, title: str, w: int = 260, h: int = 190) -> str:
    """SVG: ``outline`` (n, 2) world xy of the part footprint, ``stops`` (k, 2) robot base xy,
    numbered in visiting order, with ``title`` under it. Equal scale in x and y, a 1 m scale bar."""
    pts = [np.asarray(outline, float).reshape(-1, 2), np.asarray(stops, float).reshape(-1, 2)]
    allp = np.vstack([p for p in pts if len(p)]) if any(len(p) for p in pts) else np.zeros((1, 2))
    lo, hi = allp.min(axis=0), allp.max(axis=0)
    span = max(float((hi - lo).max()), 0.5)
    pad, top = 14.0, 8.0
    s = min((w - 2 * pad) / span, (h - 40 - top) / span)
    cx, cy = (lo + hi) / 2

    def xy(p):
        return w / 2 + (p[0] - cx) * s, top + (h - 40) / 2 - (p[1] - cy) * s  # y up

    out = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" viewBox="0 0 {w} {h}" '
        'style="border:1px solid #bbb;border-radius:4px;background:#fafafa;margin:4px">'
    ]
    o = pts[0]
    if len(o) >= 3:
        d = " ".join(f"{x:.1f},{y:.1f}" for x, y in (xy(p) for p in o))
        out.append(f'<polygon points="{d}" fill="#c8d3e6" stroke="#4a6fa5" stroke-width="1.2"/>')
    st = pts[1]
    for i in range(1, len(st)):
        (x0, y0), (x1, y1) = xy(st[i - 1]), xy(st[i])
        out.append(
            f'<line x1="{x0:.1f}" y1="{y0:.1f}" x2="{x1:.1f}" y2="{y1:.1f}" '
            'stroke="#999" stroke-dasharray="3,3"/>'
        )
    for i, p in enumerate(st):
        x, y = xy(p)
        out.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="7" fill="#e65c3c"/>')
        out.append(
            f'<text x="{x:.1f}" y="{y + 3.5:.1f}" font-size="9" text-anchor="middle" fill="#fff" '
            f'font-family="sans-serif">{i + 1}</text>'
        )
    bar = s * 1.0 if s * 1.0 < w - 2 * pad else s * 0.1
    label = "1 m" if s * 1.0 < w - 2 * pad else "0.1 m"
    by = h - 28
    out.append(f'<line x1="{pad}" y1="{by}" x2="{pad + bar:.1f}" y2="{by}" stroke="#333" stroke-width="2"/>')
    out.append(f'<text x="{pad}" y="{by - 3}" font-size="9" font-family="sans-serif">{label}</text>')
    out.append(
        f'<text x="{w / 2}" y="{h - 10}" font-size="11" text-anchor="middle" font-family="sans-serif">'
        f"{escape(title)}</text>"
    )
    out.append("</svg>")
    return "".join(out)


def stops_text(stations) -> str:
    """'x y;x y' (m, 3 decimals) of the stations' base positions, kept with a comparison row."""
    return ";".join(f"{st.xyz[0]:.3f} {st.xyz[1]:.3f}" for st in stations)


def stops_from_text(text: str) -> np.ndarray:
    rows = [r.split() for r in (text or "").split(";") if r.strip()]
    return np.array([[float(a), float(b)] for a, b in rows]) if rows else np.zeros((0, 2))
