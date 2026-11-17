"""v13 #3: part packages (core.part_package): save, load, scan a folder, apply to a project."""

import json
from pathlib import Path

import pytest

from core import part_package as pp
from core import schema

REPO = Path(__file__).resolve().parents[1]
RAIL = REPO / "examples" / "rail_long_part" / "project.json"


def _resolve(rel: str) -> Path:
    return REPO / rel


def _part_and_targets():
    proj = schema.load_project(RAIL)
    part = proj.part.model_copy(update={"pose": schema.Pose(xyz=(0.1, 0.9, 0.45), rpy=(0.0, 0.0, 0.2))})
    return proj, part, proj.targets


def test_round_trip(tmp_path):
    proj, part, targets = _part_and_targets()
    d = pp.save_part_package("Rail A/1", part, targets, _resolve, tmp_path)
    assert d.name == "Rail_A_1" and (d / "part.json").is_file() and (d / "long_rail.stl").is_file()
    pkg = pp.load_part_package(d)
    assert pkg.name == "Rail A/1" and pkg.part.mesh == "long_rail.stl"
    assert pkg.part.pose == part.pose and pkg.targets == targets
    new = pp.apply_to_project(proj, pkg, d)
    assert Path(new.part.mesh) == (d / "long_rail.stl").resolve()
    assert new.part.pose == part.pose and new.targets == targets
    assert new.results == [] and new.compared == []
    assert new.robot == proj.robot and new.end_effector == proj.end_effector and new.base == proj.base


def test_overwrite_rules(tmp_path):
    _, part, targets = _part_and_targets()
    pp.save_part_package("a", part, targets, _resolve, tmp_path)
    with pytest.raises(FileExistsError):
        pp.save_part_package("a", part, targets, _resolve, tmp_path)
    pp.save_part_package("a", part, schema.Targets(), _resolve, tmp_path, overwrite=True)
    assert pp.load_part_package(tmp_path / "a").targets.points == []
    (tmp_path / "other").mkdir()
    with pytest.raises(ValueError, match="not a part package"):
        pp.save_part_package("other", part, targets, _resolve, tmp_path, overwrite=True)


def test_scan_reports_broken_packages(tmp_path):
    _, part, targets = _part_and_targets()
    pp.save_part_package("good", part, targets, _resolve, tmp_path)
    bad_json = tmp_path / "bad_json"
    bad_json.mkdir()
    (bad_json / "part.json").write_text("{not json", encoding="utf-8")
    d = pp.save_part_package("no_mesh", part, targets, _resolve, tmp_path)
    (d / "long_rail.stl").unlink()
    d = pp.save_part_package("typo", part, targets, _resolve, tmp_path)
    data = json.loads((d / "part.json").read_text(encoding="utf-8"))
    data["colour"] = "red"
    (d / "part.json").write_text(json.dumps(data), encoding="utf-8")
    (tmp_path / "not_a_package").mkdir()
    ok, bad = pp.scan_folder(tmp_path)
    assert [x.name for x, _ in ok] == ["good"]
    reasons = {x.name: r for x, r in bad}
    assert set(reasons) == {"bad_json", "no_mesh", "typo"}
    assert "cannot be read" in reasons["bad_json"] and "missing" in reasons["no_mesh"]
    assert "colour" in reasons["typo"]


def test_unknown_tcp_is_a_clear_reason(tmp_path):
    proj, part, targets = _part_and_targets()
    groups = [g.model_copy(update={"tcps": ["no_such_tcp"]}) for g in targets.groups]
    d = pp.save_part_package("x", part, targets.model_copy(update={"groups": groups}), _resolve, tmp_path)
    with pytest.raises(ValueError, match="unknown TCPs"):
        pp.apply_to_project(proj, pp.load_part_package(d), d)
