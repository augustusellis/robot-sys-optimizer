"""v12 item 18: the spin DP runs only on sequences that need it (core.path_spin.needs_spin).

Done when clean samples clear of the joint limits skip it, and a problem sample, a joint within
LIMIT_BAND of its limit or an unwinding step make it run; trace and the Paths panel use the test.
Measured (docs/V12_PLAN.md): rail_long_part build 7.3 -> 2.6 s, same paths; CRX-30 gantry_overhead
188.6 -> 148.2 s, cycle 515 -> 517 s.
"""

import inspect

import numpy as np

from core import path_spin, paths
from core.path_spin import LIMIT_BAND, needs_spin
from core.path_types import PathSample
from ui import simulate

LO, HI = -np.full(6, np.pi), np.full(6, np.pi)


def _s(q, issue="", kind=""):
    return PathSample(0, None, 1, 2, 0.5, np.eye(4), "t", np.asarray(q, float), issue, kind=kind)


def test_clean_samples_skip_the_dp():
    assert not needs_spin([_s(np.zeros(6)), _s(np.full(6, 0.1))], LO, HI)


def test_problems_limits_and_unwinding_run_it():
    assert needs_spin([_s(np.zeros(6)), _s(np.zeros(6), "collision")], LO, HI)
    near = np.zeros(6)
    near[5] = np.pi - LIMIT_BAND / 2
    assert needs_spin([_s(near)], LO, HI)
    assert not needs_spin([_s(near, kind="home")], LO, HI)  # only the surface counts for limits
    jump = np.zeros(6)
    jump[3] = np.radians(120)
    assert needs_spin([_s(np.zeros(6)), _s(jump)], LO, HI)


def test_trace_and_panel_use_it():
    assert "spin_sequence" in inspect.getsource(paths.trace)
    assert "needs_spin" in inspect.getsource(path_spin.spin_sequence)
    assert "plan_needs_spin" in inspect.getsource(simulate.SimPanel.build)
