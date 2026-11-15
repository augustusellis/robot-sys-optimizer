"""v10 T17: Compare setups: robots and / or tools, every combination, the project's setup always in."""

from types import SimpleNamespace

import pytest
import viser

from tests.helpers import confirm_all
from ui.compare import setups

E = SimpleNamespace(client=None)


def test_setups_are_robot_tool_combinations_with_the_project_first():
    ee = object()
    out = setups(["ur10e", "ur30"], {"scanner": ee}, "ur20", "orbital_sander")
    assert [x["key"] for x in out] == ["ur20", "ur10e · scanner", "ur30 · scanner"]
    assert out[0]["project"] and out[0]["tool"] is None and out[1]["tool"] is ee
    # only robots: the project's tool; the project's robot ticked = the project's own row
    out = setups(["ur20", "ur10e"], {}, "ur20", "orbital_sander")
    assert [(x["key"], x["project"]) for x in out] == [("ur20", True), ("ur10e", False)]
    # only tools: the project's robot; ticking the project's own tool gives no duplicate
    out = setups([], {"orbital_sander": ee, "scanner": ee}, "ur20", "orbital_sander")
    assert [x["key"] for x in out] == ["ur20", "ur20 · scanner"]


@pytest.mark.slow
def test_compare_a_tool_view_it_and_save(tmp_path):
    from app import build_app

    server = viser.ViserServer(host="127.0.0.1", port=8951, verbose=False)
    try:
        app = build_app(server)
        s, cmp_, view = app["session"], app["compare"], app["compare_view"]
        assert app["project"].load_example("rail_long_part")
        cmp_.refresh_tools()
        assert "scanner" in cmp_.tool_pick and "orbital_sander" in cmp_.tool_pick
        for cb in cmp_.pick.values():
            cb.value = False
        cmp_.tool_pick["scanner"].value = True
        app["stations"].refine.value = "none"
        confirm_all(app["stations"])
        cmp_.start(block=True)
        rows = {r["setup"]: r for r in cmp_.rows}
        assert set(rows) == {"ur20", "ur20 · scanner"} and rows["ur20"]["project"]
        assert "ur20 (project)" in cmp_.status.content and "ur20 · scanner" in cmp_.status.content
        # the project's own plans stay the project's; its row is kept for the report (v10 T22)
        assert set(s.compared) == {"ur20 · scanner", "ur20 (project)"}
        assert not s.compared["ur20 (project)"]["results"]
        assert s.tool.name == "orbital_sander"  # the live session is untouched
        view.view("ur20 · scanner")
        assert s.tool.name == "scanner" and s.robot_name == "ur20"
        assert [r.label for r in s.results] == [r.label for r in s.compared["ur20 · scanner"]["results"]]
        assert "Viewing ur20 · scanner" in view.banner.content
        p = s.snapshot()  # saving while viewing saves the project's own tool
        assert p.end_effector.name == "orbital_sander"
        view.view(None)
        assert s.tool.name == "orbital_sander" and s.view_stash is None
        app["project"].save_now(tmp_path / "study")
        assert app["project"]._load(E, tmp_path / "study")
        c = s.compared["ur20 · scanner"]
        assert c["tool"].name == "scanner" and c["robot"] == "ur20" and c["results"]
    finally:
        app["builder_window"].stop()
        server.stop()
