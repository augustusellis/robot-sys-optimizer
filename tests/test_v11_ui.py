"""v11 tasks 4-5: Inspect result under the station solutions; Confirm all setup steps."""

from types import SimpleNamespace

import pytest
import viser

E = SimpleNamespace(client=None)


@pytest.fixture(scope="module")
def app():
    from app import build_app

    server = viser.ViserServer(host="127.0.0.1", port=8969, verbose=False)
    st = build_app(server)
    yield st
    st["builder_window"].stop()
    server.stop()


def test_inspect_result_sits_right_under_the_solutions(app):
    results, optimize = app["results"], app["optimize"]
    assert results.compare.order < optimize.inspect_folder.order < results.summary.order
    assert results.choice.order < optimize.inspect_folder.order < results.station.order


def test_confirm_all_setup_steps(app):
    from ui.staleness import step_state
    from ui.workflow import STEPS, setup_reason

    s, wf = app["session"], app["workflow"]
    assert app["project"].load_example("rail_long_part")
    s.steps.clear()
    wf.refresh(force=True)
    todo = wf.unconfirmed()
    assert len(todo) == len(STEPS) and all("not confirmed yet" in t for t in todo)
    assert setup_reason(s)  # the planners are gated
    assert wf.confirm_all(E)  # no browser client: no dialog
    assert all(step_state(s, k) == "confirmed" for k in STEPS) and not setup_reason(s)
    assert wf.unconfirmed() == [] and wf.confirm_all(E)
    # a change shows up as such in the dialog's list
    s.base = s.base.model_copy(update={"localization_m": s.base.localization_m + 0.001})
    assert any("External axis (changed since it was confirmed)" in t for t in wf.unconfirmed())
    # with a browser client the dialog comes first: nothing is confirmed until its button
    import contextlib

    modal = SimpleNamespace(close=lambda: None)

    class Gui:
        buttons = []
        text = ""

        @contextlib.contextmanager
        def add_modal(self, title):
            yield modal

        def add_markdown(self, text):
            Gui.text = text

        def add_button(self, label, **kw):
            b = SimpleNamespace(label=label, cb=None)
            b.on_click = lambda fn, b=b: setattr(b, "cb", fn)
            Gui.buttons.append(b)
            return b

    ev = SimpleNamespace(client=SimpleNamespace(gui=Gui(), add_notification=lambda *a, **kw: None))
    assert wf.confirm_all(ev) is False and wf.unconfirmed()
    assert "External axis" in Gui.text
    Gui.buttons[0].cb(None)  # "Confirm all"
    assert wf.unconfirmed() == []


def test_jog_arrows_commands(app):
    import numpy as np

    from ui.jog_arrows import pad_html, parse

    assert parse("jog:X,-1,1731.5") == ("X", -1) and parse("jog:J3,1,17") == ("J3", 1)
    assert parse("jog:") is None and parse("tab:tool,1") is None and parse("jog:X,2,1") is None
    full, joints = pad_html(6), pad_html(6, cartesian=False)
    assert full.count("<button") == 2 * (3 + 3 + 6) and joints.count("<button") == 12
    assert "onpointerdown" in full and "setInterval" in full and "<script" not in full
    s, cell = app["session"], app["cell"]
    assert app["project"].load_example("rail_long_part")
    jp = cell.jog_panel
    cell.set_q(np.radians([0, -100, 100, -90, -90, 0]))
    jp.joint_step.value = 5.0
    q0 = np.asarray(s.q, float).copy()
    assert jp.arrows.command("jog:J1,1,1.0")  # one step per command: held = one every 120 ms
    assert jp.arrows.command("jog:J1,1,1.1")
    assert np.degrees(s.q[0] - q0[0]) == pytest.approx(10.0)
    jp.frame.value = "World"
    jp.refresh()
    jp.step.value = 20.0
    p0 = jp.T_world_point()[:3, 3].copy()
    assert jp.arrows.command("jog:Z,1,2.0")
    assert jp.T_world_point()[2, 3] - p0[2] == pytest.approx(0.02, abs=1e-6)
    assert not jp.arrows.command("garbage")
    jp.frame.value = "Joint"
    jp.refresh()
    assert jp.arrows.html.content.count("<button") == 12  # joint arrows only in the Joint frame
