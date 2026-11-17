"""v13 #7: Results -> Batch results: open a batch, switch between its parts."""

import numpy as np
import pytest
import viser

from tests.test_v13_batch import RAIL, make_parts
from ui.batch_view import table


def test_table_lists_failures():
    md = table(
        {
            "updated": "now",
            "rows": [
                {
                    "part": "a",
                    "status": "done",
                    "reached": 5,
                    "points": 5,
                    "stops": 2,
                    "cycle_s": 40.0,
                    "cost": 3,
                },
                {"part": "b", "status": "failed", "reason": "no plan"},
            ],
        }
    )
    assert "**1 of 2 parts planned**" in md and "| a | ✔ | 5/5 | 2 | 40.0 | 3 |" in md
    assert "✘ **b**: no plan" in md


@pytest.mark.slow
def test_switch_between_parts(tmp_path):
    from app import build_app
    from ui.batch_run import run_batch

    out = tmp_path / "out"
    rows = run_batch(RAIL, make_parts(tmp_path / "parts"), out, workers=2)
    assert all(r["status"] == "done" for r in rows), rows
    server = viser.ViserServer(host="127.0.0.1", port=8988, verbose=False)
    try:
        app = build_app(server)
        view, s = app["batch_view"], app["session"]
        assert not view.open_batch(tmp_path / "nothing")
        assert view.open_batch(out)
        assert view.part.options == ("rail_a", "rail_b") and "2 of 2 parts planned" in view.summary.content
        for name, x in (("rail_b", 0.4), ("rail_a", 0.0)):
            assert view.open_part(name=name)
            assert np.isclose(s.part.T_world_part[0, 3], x) and s.results
            app["results"].refresh()
            assert app["results"].selected() is not None
            sim = app["sim"]
            if sim._thread is not None:
                sim._thread.join()
            assert sim.plan is not None and len(sim.plan.samples) > 0  # paths built when opened
            assert view.report_path(name).is_file()
        # remembered for the next start
        app2_server = viser.ViserServer(host="127.0.0.1", port=8990, verbose=False)
        try:
            app2 = build_app(app2_server)
            assert app2["batch_view"].data is not None and app2["batch_view"].out == out
        finally:
            app2["builder_window"].stop()
            app2_server.stop()
    finally:
        app["builder_window"].stop()
        server.stop()
