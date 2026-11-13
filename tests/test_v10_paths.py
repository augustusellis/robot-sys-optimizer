"""V10 part C (docs/V10_PLAN.md tasks 9-10): free tool spin along the path, dense edge checks."""

import numpy as np
import pytest
import viser

from core.path_spin import meet, offsets
from tests.helpers import confirm_all


@pytest.fixture(scope="module")
def app():
    from app import build_app

    server = viser.ViserServer(host="127.0.0.1", port=8854, verbose=False)
    st = build_app(server)
    yield st
    st["builder_window"].stop()
    server.stop()


# --- task 9: free tool spin along the path -------------------------------------------------------


def test_spin_offsets():
    free = offsets(None)
    assert len(free) == 24 and np.isclose(free, 0.0).any()
    r = offsets((np.radians(-10), np.radians(40)))
    assert np.isclose(r[0], np.radians(-10)) and np.isclose(r[-1], np.radians(40)) and 0.0 in r
    assert list(offsets((0.0, 0.0))) == [0.0]
    assert meet(None, (0.1, 0.2)) == (0.1, 0.2) and meet((0.0, 0.3), (0.1, 0.5)) == (0.1, 0.3)
    assert meet((0.0, 0.1), (0.2, 0.3)) == (0.0, 0.0)


@pytest.mark.slow
def test_spin_paths_no_worse_than_planned_spins(app):
    """With *Optimize tool spin* the paths have no more problem samples and no longer cycle time
    than with the planned spins (the same plan): the build keeps the better of the two."""
    from ui.sim_build import score

    s, sim = app["session"], app["sim"]
    assert app["project"].load_example("rail_long_part")
    confirm_all(s)
    app["workflow"].refresh(force=True)
    app["stations"].start(block=True, quick=True)
    out = {}
    for on in (False, True):
        sim.spin_opt.value = on
        sim.build(block=True)
        out[on] = score(sim.plan, sim.cycle)
    assert out[True] <= out[False]
    assert sim.spin_used is not None and "Tool spin" in sim.info.content
