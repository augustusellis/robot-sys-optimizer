"""v12 item 8: plans are keyed by an id, not their label (ui.plan_ids).

Done when two plans with the same label keep their own joint solutions and stamps, ids survive a
save and reopen, copies of one plan get their own ids, and projects saved before v12 (no ids, data
keyed by label) open with their joint solutions.
"""

from pathlib import Path

from core.feasibility import PointResult
from core.schema import Solution, Station
from ui.plan_ids import give_id, give_ids, key
from ui.session import Session, load_project, save_project

REPO = Path(__file__).resolve().parents[1]


def _sol(label="2 stations: 3/3 reached", **kw):
    return Solution(label=label, stations=[Station(xyz=(0, 0, 0))], assignment={0: 0}, **kw)


def _res(q0):
    return {0: PointResult(id=0, status="reachable", q=[q0, 0, 0, 0, 0, 0])}


def test_same_label_plans_keep_their_own_joints():
    a, b = _sol(), _sol()
    give_ids([a, b])
    assert a.id and b.id and a.id != b.id and a.label == b.label
    pts = {key(a): _res(0.1), key(b): _res(0.2)}
    assert pts[key(a)][0].q[0] == 0.1 and pts[key(b)][0].q[0] == 0.2  # one label, two plans


def test_copies_get_their_own_id():
    a = _sol()
    give_id(a)
    c = a.model_copy(update={"label": "design 2"})
    assert c.id == a.id  # model_copy keeps it ...
    give_id(c, [a])
    assert c.id != a.id  # ... and entering the results gives it its own
    keep = a.id
    give_id(a, [c])
    assert a.id == keep  # an id nobody else uses is kept
    assert _sol().key == _sol().label  # no id (saved before v12): the label


def test_ids_survive_save_and_old_label_keys_load(tmp_path):
    s = Session()
    a, b = _sol(), _sol()
    give_ids([a, b])
    s.results = [a, b]
    s.result_points = {key(a): _res(0.1), key(b): _res(0.2)}
    s.stamps = {key(a): {"part": "x"}, key(b): {"part": "y"}}
    save_project(s, tmp_path)
    s2 = Session()
    load_project(s2, tmp_path)
    assert [r.id for r in s2.results] == [a.id, b.id]
    assert s2.result_points[a.id][0].q[0] == 0.1 and s2.result_points[b.id][0].q[0] == 0.2
    assert s2.stamps[b.id]["part"] == "y"  # (staleness.migrate adds the newer components)
    # a project of v11 (no id, keyed by the label) still finds its joint solutions
    old = Session()
    o = _sol("old plan")
    old.results, old.result_points = [o], {"old plan": _res(0.3)}
    save_project(old, tmp_path / "old")
    s3 = Session()
    load_project(s3, tmp_path / "old")
    assert s3.results[0].id == "" and s3.result_points["old plan"][0].q[0] == 0.3
