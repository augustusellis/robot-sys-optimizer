"""Measure the paths of a planned project (v10 task 9): wrist jumps, unwinding moves, joint travel
and cycle time.

    python scripts/measure_paths.py plan <example> <robot> <out_folder>   # plan once, save the project
    python scripts/measure_paths.py paths <project_folder> [--no-spin] [--no-smooth] [--trapezoid]

``plan`` loads a shipped example in a headless app, switches the robot, confirms the setup, runs the
station planner and saves the project (with its plans) to ``out_folder``. ``paths`` opens that
project, builds the paths of the best plan with the current Paths settings and prints the flagged
samples, the unwinding moves (a joint step over 90 deg), the wrist joints' travel on the surface and
the cycle time, so two code versions can be compared on the same plan. Uses a temporary workspace.
"""

import os
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
os.environ.setdefault("RSO_WORKSPACE", tempfile.mkdtemp())

import numpy as np  # noqa: E402
import viser  # noqa: E402

from app import build_app  # noqa: E402


def numbers(plan, plan_limits) -> dict:
    out = {"samples": len(plan.samples), **{k: v for k, v in plan.counts().items() if v}}
    unwind, wrist = 0, 0.0
    for st in plan.stations:
        prev, prev_kind = None, ""
        for p in st.samples:
            if p.q is None:
                continue
            q = np.asarray(p.q, float)
            if prev is not None:
                d = np.abs(q - prev)
                unwind += int(d.max() > np.radians(90.0))
                if not p.kind and not prev_kind:
                    wrist += float(d[3] + d[5])
            prev, prev_kind = q, p.kind
    lo, hi = plan_limits
    marg = [
        float(np.degrees(np.min(np.minimum(np.asarray(p.q) - lo, hi - np.asarray(p.q)))))
        for p in plan.samples
        if p.q is not None and not p.kind
    ]
    out["joint limit margin on the surface, min / mean (deg)"] = (
        round(min(marg), 1),
        round(float(np.mean(marg)), 1),
    )
    out["unwinding moves"] = unwind
    out["wrist travel on the surface (deg)"] = round(float(np.degrees(wrist)), 1)
    return out


def main() -> None:
    from tests.helpers import confirm_all

    server = viser.ViserServer(host="127.0.0.1", port=8795, verbose=False)
    try:
        app = build_app(server)
        s = app["session"]
        if sys.argv[1] == "plan":
            example, robot, out = sys.argv[2], sys.argv[3], Path(sys.argv[4])
            assert app["project"].load_example(example)
            app["cell"].robot_choice.value = robot
            confirm_all(s)
            app["workflow"].refresh(force=True)
            app["stations"].start(block=True)
            print("plans:", [(r.label, len(r.unreached), round(r.cost, 3)) for r in s.results])
            app["project"].save_now(out)
            print("saved", out)
            return
        assert app["project"]._load(None, Path(sys.argv[2]))
        sim = app["sim"]
        if "--no-spin" in sys.argv and hasattr(sim, "spin_opt"):
            sim.spin_opt.value = False
        if "--no-smooth" in sys.argv and hasattr(sim, "smooth"):  # v10 T13
            sim.smooth.value = False
        if "--trapezoid" in sys.argv:  # v10 T14: the pre-v10 timing
            app["cycle"].timing.value = "Trapezoid per move (pre-v10)"
        app["results"].refresh()
        import time

        t0 = time.perf_counter()
        sim.build(block=True)
        print(f"build: {time.perf_counter() - t0:.1f} s")  # v12 #18: the build alone
        print(sim.info.content.replace("  \n", "\n"))
        from core.path_spin import STATS

        print("sequences:", STATS)
        for k, v in numbers(sim.plan, (s.robot.lower, s.robot.upper)).items():
            print(f"{k}: {v}")
        sm = getattr(sim.plan, "smooth", [])
        if sm:  # v10 T13: the planned joint moves that were optimised
            used = [x for x in sm if x.used]
            print(f"smoothed moves: {len(used)} of {len(sm)}")
            if used:
                print(
                    "  length seed -> smooth (rad):",
                    round(sum(x.length_seed for x in used), 2),
                    "->",
                    round(sum(x.length for x in used), 2),
                )
                print(
                    "  min clearance seed -> smooth (mm):",
                    round(1000 * min(x.clear_seed for x in used), 1),
                    "->",
                    round(1000 * min(x.clear for x in used), 1),
                )
    finally:
        server.stop()


if __name__ == "__main__":
    main()
