"""Path data types (split from core.paths, which re-exports them, in v10 task 9)."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

ISSUES = (
    "unreachable", "config_flip", "jump", "collision", "reconfigure", "carrier_blocked", "motion",
    "direction",  # v11 #7: a directional tool's axis off the travel direction
)  # fmt: skip


@dataclass
class PathSample:
    station: int
    zone: int | None
    a: int  # point id the segment starts at
    b: int  # point id it ends at (a == b: the first point of a sequence)
    s: float  # 0..1 along the segment
    T_part_tcp: np.ndarray
    tcp: str
    q: np.ndarray | None
    issue: str = ""
    kind: str = ""  # "" on the surface, "lift", "home" (joint move), "retreat", "carrier" (between stops)
    carrier: dict | None = None  # carrier values of this sample (None = its stop's)
    T_world_base: np.ndarray | None = None  # base pose when it differs from the stop's


@dataclass
class StationPath:
    station: int
    T_world_base: np.ndarray
    carrier: dict
    samples: list[PathSample] = field(default_factory=list)


@dataclass
class PathPlan:
    stations: list[StationPath]
    step: float
    smooth: list = field(default_factory=list)  # v10 T13: core.traj_opt.SmoothStats per planned move

    @property
    def samples(self) -> list[PathSample]:
        return [p for st in self.stations for p in st.samples]

    def counts(self) -> dict[str, int]:
        out = dict.fromkeys(ISSUES, 0)
        for p in self.samples:
            if p.issue:
                out[p.issue] += 1
        return out

    def length(self) -> float:
        """TCP path length (m) on the surface (no lifts, home or carrier moves)."""
        total = 0.0
        for st in self.stations:
            for p0, p1 in zip(st.samples[:-1], st.samples[1:], strict=True):
                if p1.issue != "reconfigure" and p1.a != p1.b and not p1.kind and not p0.kind:
                    total += float(np.linalg.norm(p1.T_part_tcp[:3, 3] - p0.T_part_tcp[:3, 3]))
        return total
