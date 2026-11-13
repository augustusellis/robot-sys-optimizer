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


# --- task 8: Tool tab per type, one home place, status strip, builder tab bar --------------------


@pytest.fixture(scope="module")
def app():
    import viser

    from app import build_app

    server = viser.ViserServer(host="127.0.0.1", port=8853, verbose=False)
    st = build_app(server)
    yield st
    st["builder_window"].stop()
    server.stop()


def test_tool_tab_shows_its_type(app):
    from core.tool import load_tool
    from ui.tool_setup import SOURCES

    tab, s = app["tool"], app["session"]
    s.set_tool(load_tool("orbital_sander"))
    tab.refresh()
    assert tab.source.value == SOURCES[0] and tab.lib_folder.visible and not tab.mesh_panel.folder.visible
    assert tab.pick.visible  # sanding: contact faces may be picked
    tab.source.value = SOURCES[1]
    assert not tab.lib_folder.visible and tab.mesh_panel.folder.visible
    tab.source.value = SOURCES[2]
    assert "Package builder" in tab.source_md.content and not tab.mesh_panel.folder.visible
    s.set_tool(load_tool("scanner"))  # contact-free: no face picking
    tab.refresh()
    assert tab.source.value == SOURCES[0] and not tab.pick.visible and not tab.links_folder.visible


def test_home_in_one_place(app):
    s, cell = app["session"], app["cell"]
    hp = cell.home_panel
    assert cell.home is hp.go  # Go to home lives in Home position
    assert not hp.use_suggested.visible
    s.suggested_home = np.radians([0.0, -150.0, -120.0, -180.0, -90.0, 0.0])
    hp.refresh()
    assert hp.use_suggested.visible and "-150" in hp.suggested.content
    assert hp.take_suggested()
    assert s.home_deg == pytest.approx([0.0, -150.0, -120.0, -180.0, -90.0, 0.0])
    assert s.suggested_home is None and not hp.use_suggested.visible


def test_status_strip(app):
    from tests.helpers import confirm_all
    from ui.workflow import strip_text

    s, wf = app["session"], app["workflow"]
    assert app["project"].load_example("rail_long_part")
    wf.refresh(force=True)
    assert "○ 1 Robot & cell" in wf.strip_md.content and "no plan yet" in wf.strip_md.content
    confirm_all(s)
    wf.refresh(force=True)
    assert "✔ 4 Targets" in wf.strip_md.content and "○" not in wf.strip_md.content
    t = strip_text(
        {k: "confirmed" for k in ("Cell", "External Axis", "Tool Setup", "Targets")}, 2, ["tool"], [], True
    )
    assert "⚠ plan out of date (tool)" in t and "✔ check up to date" in t


def test_builder_tab_bar(app):
    from ui.package_tabs import bar_html

    b = app["builder"]
    b.tab_bar.command("tab:rail,123")
    assert b.pkg.kind == "rail" and b.tab_bar.active == "rail"
    assert "background:#3b82f6;color:#fff" in b.tab_bar.html.content.split("Rail")[0].rsplit("<button", 1)[1]
    b.tab_bar.command("tab:nonsense,1")  # ignored
    assert b.pkg.kind == "rail"
    b._set_kind("tool")  # changed in code: the bar follows
    assert b.tab_bar.active == "tool"
    assert bar_html({"A": "a"}, "a").count("<button") == 1
