"""v10 T22: the report's setup comparison (project's setup always in, a picture per setup), the
gantry placement section and the coordinated vs stop-and-go cycle time."""

import numpy as np
import pytest
import viser

from core.report_svg import plan_view, stops_from_text, stops_text
from core.schema import Station


def test_plan_view_svg_and_stops_text():
    st = [Station(xyz=(0.5, -1.0, 0)), Station(xyz=(2.0, -1.0, 0))]
    txt = stops_text(st)
    assert txt == "0.500 -1.000;2.000 -1.000" and np.allclose(stops_from_text(txt), [[0.5, -1], [2, -1]])
    svg = plan_view(np.array([[0, 0], [3, 0], [3, 0.5], [0, 0.5]]), stops_from_text(txt), "ur20 <project>")
    assert svg.startswith("<svg") and svg.count("<circle") == 2 and "<polygon" in svg
    assert "ur20 &lt;project&gt;" in svg and "<script" not in svg


@pytest.mark.slow
def test_report_sections_compare_frames_and_carrier_motion():
    from app import build_app
    from tests.helpers import confirm_all
    from ui.sim_fields import MOTION

    server = viser.ViserServer(host="127.0.0.1", port=8961, verbose=False)
    try:
        app = build_app(server)
        cmp_, sim, results = app["compare"], app["sim"], app["results"]
        assert app["project"].load_example("rail_long_part")
        for name, cb in cmp_.pick.items():
            cb.value = name == "ur10e"
        app["stations"].refine.value = "none"
        confirm_all(app["stations"])
        cmp_.start(block=True)
        app["stations"].start(block=True, quick=True)
        results.refresh()
        sol = results.selected()
        for m in MOTION:  # both carrier motions built for this plan
            sim.motion_mode.value = m
            sim.build(block=True)
        sim.motion_mode.value = MOTION[0]
        app["frame"].rows = [
            {"frame": "x 0, y -1500 mm", "stations": 2, "reached": 25, "cycle_s": 80.0, "cost": 9.1},
            {
                "frame": "x 0, y -1800 mm (refined)",
                "stations": 2,
                "reached": 25,
                "cycle_s": 82.0,
                "cost": 9.4,
            },
        ]
        sec = dict(results.export.sections(sol))
        comp = sec["Setup comparison"]
        assert comp.index("ur20 (project)") < comp.index("ur10e")  # the project's setup first
        assert comp.count("<svg") == 2  # a picture per setup
        assert "x 0, y -1800 mm (refined)" in sec["Gantry placement"]
        assert "coordinated" in sec["Carrier motion"] and "stop and go" in sec["Carrier motion"]
        path, data = results.export.export_report(sol)
        html = data.decode("utf-8")
        assert "Setup comparison" in html and html.count("<svg") >= 2 and "Gantry placement" in html
    finally:
        app["builder_window"].stop()
        server.stop()
