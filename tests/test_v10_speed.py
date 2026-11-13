"""V10 part B (docs/V10_PLAN.md tasks 7-8): reused check workers, Tool tab / status strip / builder tabs."""

import numpy as np
import pytest

from ui.session import Session, load_project


@pytest.fixture
def rail(tmp_path):
    import shutil
    from pathlib import Path

    src = Path(__file__).resolve().parents[1] / "examples" / "rail_long_part"
    shutil.copytree(src, tmp_path / "rail")
    s = Session()
    load_project(s, tmp_path / "rail")
    return s


def _spec(s, robot_name=None):
    from core.robot import Robot
    from opt.parallel import CheckerSpec

    robot = Robot(s.robot.spec) if robot_name is None else Robot.from_registry(robot_name)
    bb, wb = s.sites().scene_bodies()
    return CheckerSpec(
        robot.spec, s.tool_model, s.part_mesh(), s.T_world_base, s.part.T_world_part, bb, wb, []
    )


def _calls(s, dx=0.0):
    from ui.stations import world_targets

    t = world_targets(s)
    T = s.T_world_base.copy()
    T[0, 3] += dx
    return [
        (T, "point", int(t.ids[i]), t.P_part[i], t.N_part[i], t.groups[i], None, True)
        for i in range(len(t.ids))
    ]


# --- task 7: reuse worker processes -------------------------------------------------------------


def test_workers_retarget_to_another_setup(rail):
    """The same processes serve a second setup (another robot) with the results of a fresh checker."""
    from opt import parallel

    s = rail
    try:
        a = parallel.pool_for(_spec(s), 2)
        ca = _calls(s)
        ra = a.run(_spec(s).build().settings, ca)
        ex = parallel._executor(2)
        pids = set(ex._processes)
        spec_b = _spec(s, "ur10e")
        b = parallel.pool_for(spec_b, 2)
        assert b is not a and b.key != a.key
        cb = _calls(s, 0.5)
        rb = b.run(spec_b.build().settings, cb)
        assert parallel._executor(2) is ex and set(ex._processes) == pids  # no new processes
        ref = spec_b.build()
        for c, r in zip(cb, rb, strict=True):
            T, _k, pid, p, n, g, mp_, cl = c
            ref.set_base(T)
            e = ref._check_point(pid, p, n, g, mp_, cl)
            assert (r.status, r.reason) == (e.status, e.reason)
            assert (r.q is None) == (e.q is None) and (r.q is None or np.allclose(r.q, e.q))
        assert len(ra) == len(ca)
    finally:
        parallel.shutdown_all()
