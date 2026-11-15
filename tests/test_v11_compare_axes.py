"""v11 task 8: compare external axis types on the same part and points.

Done when rail_long_part compared as static / rail / floor gantry / overhead gantry / mobile gives
five rows with stations, reach, cycle and cost, and each is viewable in Results (the carrier swaps
in and out, the project's own base is saved meanwhile).
"""

from types import SimpleNamespace

import pytest
import viser

from tests.helpers import confirm_all
from ui.compare import setups

E = SimpleNamespace(client=None)


def test_setups_with_axes():
    rail, mob = object(), object()
    out = setups([], {}, "ur20", "sander", {"Rail": rail, "Mobile base": mob})
    assert [x["key"] for x in out] == ["ur20", "ur20 · Rail", "ur20 · Mobile base"]
    assert out[0]["project"] and out[1]["axis"] is rail and out[2]["axis_name"] == "Mobile base"
    out = setups(["ur10e"], {}, "ur20", "sander", {"Rail": rail})
    assert [x["key"] for x in out] == ["ur20", "ur10e · Rail"]


def test_axis_choices_and_placement_kinds(tmp_path):
    from ui.compare_axes import STATIC, axis_choices, kind_of
    from ui.session import Session

    s = Session(project_dir=tmp_path)
    ch = axis_choices(s)
    assert {STATIC, "Rail", "Floor XY gantry", "Overhead gantry", "Mobile base"} <= set(ch)
    kinds = {n: kind_of(b) for n, b in ch.items()}
    assert kinds[STATIC] == "static" and kinds["Rail"] == "rail" and kinds["Mobile base"] == "mobile"
    assert kinds["Floor XY gantry"] == kinds["Overhead gantry"] == "gantry"


@pytest.mark.slow
def test_rail_part_on_five_axis_types(tmp_path):
    from app import build_app

    server = viser.ViserServer(host="127.0.0.1", port=8975, verbose=False)
    try:
        app = build_app(server)
        s, cmp_, view = app["session"], app["compare"], app["compare_view"]
        assert app["project"].load_example("rail_long_part")
        cmp_.refresh_axes()
        for cb in cmp_.pick.values():
            cb.value = False
        names = ["Static arm (one base)", "Rail", "Floor XY gantry", "Overhead gantry", "Mobile base"]
        for n in names:
            cmp_.axis_pick[n].value = True
        app["stations"].refine.value = "none"
        confirm_all(app["stations"])
        own_base = s.base.model_dump_json()
        cmp_.start(block=True)
        rows = {r["setup"]: r for r in cmp_.rows}
        assert set(rows) == {"ur20", *(f"ur20 · {n}" for n in names)}
        for n in names:
            r = rows[f"ur20 · {n}"]
            assert r.get("stations") is not None and r.get("reached") is not None, (n, r)
            assert r.get("cost") is not None
        assert rows["ur20 · Static arm (one base)"]["stations"] == 1
        assert rows["ur20 · Overhead gantry"]["axis_kind"] == "gantry"
        assert s.base.model_dump_json() == own_base  # the live project is untouched
        key = "ur20 · Floor XY gantry"
        assert key in s.compared and s.compared[key]["base"].carrier == "gantry"
        view.view(key)
        assert s.base.carrier == "gantry" and s.base.mode == "movable"
        p = s.snapshot()  # saving while viewing saves the project's own axis
        assert p.base.carrier == "rail"
        view.view(None)
        assert s.base.model_dump_json() == own_base
        app["project"].save_now(tmp_path / "study")
        assert app["project"]._load(E, tmp_path / "study")
        assert s.compared[key]["base"].carrier == "gantry"
    finally:
        app["builder_window"].stop()
        server.stop()
