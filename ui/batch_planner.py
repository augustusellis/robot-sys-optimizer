"""Planner settings of a batch job (``BatchJob.planner``, plan Part 2 step B4).

``planner_snapshot(stations_panel)`` records the Station planner panel at *Add to queue*;
``spec_for(planner, session)`` rebuilds the ``plan_job.station_job`` spec from it exactly as
``StationsPanel.start`` builds it. JSON-safe both ways; no viser import at module level.
"""

from __future__ import annotations

from dataclasses import asdict, fields, replace

SNAPSHOT_VERSION = 1
# planner dict keys (BatchJob.planner): version, mode ("cost" | "quick" | "rules"), settings
# (PlannerSettings fields; max_stations as typed, static bases plan one), weights (Weights dict;
# None = the cell's optimizer weights), workers, collision (COLLISION_KEYS), and per mode:
# objective, pareto_keys, robust {tolerance, measure} (cost); rules {objective, sing_pct}
# v12 #10: the fields of core.collision_config.CollisionConfig
COLLISION_KEYS = ("dress_m", "hose", "geom", "part_keep", "arm_pad", "link_pad", "part_low", "part_low_scale")
_NOT_STORED = ("speed_zones", "exclude")  # set by the job / the rules planner, not by the user


# --- planner settings ----------------------------------------------------------------------


def settings_dict(st) -> dict:
    """PlannerSettings -> a JSON-safe dict (cycle settings as a dict)."""
    d = asdict(st)
    for k in _NOT_STORED:
        d.pop(k, None)
    return d


def collision_dict(cfg) -> dict:
    """A ``core.collision_config.CollisionConfig`` -> JSON-safe (COLLISION_KEYS)."""
    return {
        "dress_m": float(cfg.dress_m),
        "hose": None if cfg.hose is None else asdict(cfg.hose),
        "geom": {c: asdict(g) for c, g in cfg.geom.items()},
        "part_keep": [int(f) for f in cfg.part_keep],
        "arm_pad": float(cfg.arm_pad),
        "link_pad": {str(int(k)): float(v) for k, v in cfg.link_pad.items()},
        "part_low": cfg.part_low,
        "part_low_scale": float(cfg.part_low_scale),
    }


def collision_from(c: dict):
    """``collision_dict`` -> CollisionConfig (keys a batch from before v12 lacks get defaults)."""
    from core.collision_config import CollisionConfig
    from core.collision_mesh import Geom
    from core.hose import Hose

    return CollisionConfig(
        dress_m=float(c.get("dress_m", 0.0)),
        hose=Hose(**c["hose"]) if c.get("hose") else None,
        geom={k: Geom(**v) for k, v in (c.get("geom") or {}).items()},
        part_keep=tuple(int(f) for f in c.get("part_keep") or ()),
        arm_pad=float(c.get("arm_pad", 0.0)),
        link_pad={int(k): float(v) for k, v in (c.get("link_pad") or {}).items()},
        part_low=c.get("part_low"),
        part_low_scale=float(c.get("part_low_scale", 1.0)),
    )


def planner_settings_from(planner: dict, session=None):
    """``BatchJob.planner`` -> PlannerSettings. With ``session``, a static base plans one
    station (as StationsPanel.planner_settings does). Unknown keys raise ValueError."""
    from core.cycle_time import CycleSettings
    from opt.placement import PlannerSettings

    raw = dict(planner.get("settings") or {})
    unknown = sorted(set(raw) - {f.name for f in fields(PlannerSettings)})
    if unknown:
        raise ValueError(f"unknown planner setting(s) {', '.join(unknown)}: queued by another app version")
    if isinstance(raw.get("cycle"), dict):
        raw["cycle"] = CycleSettings(**raw["cycle"])
    st = PlannerSettings(**raw)
    if session is not None and session.sites() is None:
        st.max_stations = 1
    if planner.get("mode") == "quick":  # exactly StationsPanel.start(quick=True)
        st = PlannerSettings(
            max_stations=st.max_stations,
            candidate_step=st.candidate_step,
            refine=False,
            zoning=False,
            areas=False,
            patience=1,
            quick_first=True,
            cycle=st.cycle,
            wrap_limit_deg=st.wrap_limit_deg,
        )
    return st


def spec_for(planner: dict, session) -> dict:
    """The ``station_job`` spec, as StationsPanel.start builds it, from a job's planner dict."""
    from core.schema import Optimizer, Weights
    from opt.tolerance import ToleranceSettings

    w = planner.get("weights")
    weights = Weights.model_validate(w) if w else (session.extra.get("optimizer") or Optimizer()).weights
    c = planner.get("collision")
    spec = {
        "settings": planner_settings_from(planner, session),
        "weights": weights,
        "rules": None,
        "workers": int(planner.get("workers", 1)),
    }
    if c:  # the collision settings recorded at queue time (else the process's own); batch_job uses it
        spec["collision"] = collision_from(c)
    mode = planner.get("mode", "cost")
    if mode == "cost":
        spec["objective"] = planner.get("objective", "weighted")
        spec["pareto_keys"] = tuple(planner.get("pareto_keys") or ("stations", "cycle", "clearance"))
        r = planner.get("robust") or {}
        spec["robust"] = {
            "tolerance": ToleranceSettings(**(r.get("tolerance") or {})),
            "measure": r.get("measure", "Worst case"),
        }
    elif mode == "rules":
        spec["rules"] = dict(planner["rules"])
    return spec


def planner_snapshot(panel, quick: bool = False) -> dict:
    """The Station planner panel's settings for *Add to queue* (JSON-safe BatchJob.planner).
    Reads the widgets like StationsPanel.planner_settings / start, without side effects."""
    from core.collision_config import current
    from core.schema import Weights
    from opt.tolerance import ToleranceSettings
    from ui import jobs, rules_ui

    settings = settings_dict(panel.planner_settings())
    settings["max_stations"] = int(panel.max_stations.value)  # the static-base rule applies at run time
    costs = panel.costs
    w = Weights(**{**costs.weights().model_dump(), **{k: float(f.value) for k, f in costs.fields.items()}})
    rules = not quick and rules_ui.rules_mode(panel)
    out = {
        "version": SNAPSHOT_VERSION,
        "mode": "quick" if quick else ("rules" if rules else "cost"),
        "settings": settings,
        "weights": w.model_dump(),
        "workers": int(jobs.CHECK_WORKERS),
        "collision": collision_dict(current()),
    }
    if out["mode"] == "cost":
        tol = panel.tolerance_settings() if panel.tolerance_settings else ToleranceSettings()
        out["objective"] = panel.objective_m.key
        out["pareto_keys"] = [k for k, cb in panel.pareto_keys.items() if cb.value]
        out["robust"] = {
            "tolerance": asdict(replace(tol, samples=int(panel.robust_samples.value))),
            "measure": panel.robust_measure.value,
        }
    elif rules:
        out["rules"] = {"objective": panel.objective.value, "sing_pct": float(panel.sing_pct.value)}
    return out
