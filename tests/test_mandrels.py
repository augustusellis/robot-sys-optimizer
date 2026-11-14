"""Synthetic mandrel generator, metrics and the two-file sample format (T0)."""

import numpy as np
import pytest
import trimesh

from core.mandrels import (
    SYNTHETIC_CASES,
    build_case,
    faceplate_eggcrate,
    thin_shell,
    vary,
)
from core.tool_surface_eval import (
    largest_smooth,
    load_sample,
    normal_up,
    save_sample,
    score,
)


@pytest.fixture(scope="module")
def plate():
    return faceplate_eggcrate(seed=1, resolution=0.04)


@pytest.fixture(scope="module")
def male():
    return thin_shell("male", seed=1, resolution=0.02)


@pytest.fixture(scope="module")
def female():
    return thin_shell("female", seed=1, resolution=0.02)


def _mean_nz(mesh, truth):
    a = mesh.area_faces[truth]
    return float((mesh.face_normals[truth, 2] * a).sum() / a.sum())


def test_generators_have_truth_facing_up(plate, male, female):
    for mesh, truth, info in (plate, male, female):
        assert truth.shape == (len(mesh.faces),) and truth.any()
        assert _mean_nz(mesh, truth) > 0.5
        assert info["winding_consistent"] and info["watertight"]
        assert mesh.volume > 0  # outward normals


def test_surfaces_have_convex_and_concave_regions(plate, male, female):
    for mesh, truth, _ in (plate, male, female):
        top = mesh.submesh([np.flatnonzero(truth)], append=True)
        v = top.vertices
        # sign of the second difference along x on a vertex sample: both signs must occur
        order = np.argsort(v[:, 0])
        z = v[order, 2]
        curv = np.gradient(np.gradient(np.convolve(z, np.ones(25) / 25, "same")))
        assert (curv > 1e-9).any() and (curv < -1e-9).any()


def test_flange_is_not_truth():
    m, t, info = thin_shell("male", flange=True, resolution=0.02)
    m2, t2, _ = thin_shell("male", flange=False, resolution=0.02)
    assert t.sum() == t2.sum()
    assert len(m.faces) > len(m2.faces) and info["flange"]


def test_baselines_fail_where_expected(female):
    mesh, truth, _ = female
    s = score(normal_up(mesh), truth, mesh)
    assert s["recall"] > 0.95  # the trough faces up
    assert s["precision"] < 1.0  # but the flange lip also scores


def test_deterministic():
    a = build_case(SYNTHETIC_CASES[0])
    b = build_case(SYNTHETIC_CASES[0])
    assert np.array_equal(a[0].vertices, b[0].vertices) and np.array_equal(a[1], b[1])


@pytest.mark.parametrize("remesh", ["coarse", "fine", "irregular"])
def test_vary_keeps_labels_aligned(female, remesh):
    mesh, truth, _ = female
    m2, t2, info = vary(mesh, truth, remesh=remesh, seed=3)
    assert len(t2) == len(m2.faces) and t2.any()
    # re-derive labels from the original truth faces by proximity (the sample-format rule)
    sub = trimesh.Trimesh(mesh.vertices, mesh.faces[truth], process=False)
    from core.tool_surface_eval import labels_from_proximity

    ref = labels_from_proximity(m2, sub, tol=0.0008)
    assert score(t2, ref, m2)["iou"] >= 0.95


def test_vary_holes_noise_tilt(male):
    mesh, truth, _ = male
    m2, t2, info = vary(mesh, truth, noise_m=0.0002, tilt_deg=8.0, holes=3, seed=1)
    assert len(m2.faces) < len(mesh.faces) and len(t2) == len(m2.faces)
    assert not info["watertight"]
    assert _mean_nz(m2, t2) > 0.5
    assert abs(np.degrees(np.arccos(np.clip(m2.face_normals[t2, 2].mean(), -1, 1))) - 0) < 30


def test_score_simple():
    m = trimesh.creation.box((1, 1, 1))
    truth = m.face_normals[:, 2] > 0.5
    assert score(truth, truth, m) == {"iou": 1.0, "precision": 1.0, "recall": 1.0}
    s = score(np.ones(len(m.faces), bool), truth, m)
    assert s["recall"] == 1.0 and s["precision"] == pytest.approx(1 / 6)
    assert s["iou"] == pytest.approx(1 / 6)
    s = score(~truth, truth, m)
    assert s["iou"] == 0.0 and s["recall"] == 0.0
    with pytest.raises(ValueError):
        score(truth[:-1], truth, m)


def test_load_sample_recovers_truth(tmp_path, male):
    mesh, truth, _ = male
    save_sample(tmp_path / "s1", mesh, truth)
    m2, t2, info = load_sample(tmp_path / "s1")
    assert len(m2.faces) == len(mesh.faces)
    assert score(t2, truth, m2)["iou"] > 0.98
    assert info["truth_faces"] == int(t2.sum())


def test_load_sample_errors(tmp_path):
    (tmp_path / "a").mkdir()
    with pytest.raises(FileNotFoundError):
        load_sample(tmp_path / "a")
    box = trimesh.creation.box((1, 1, 1))
    (tmp_path / "b").mkdir()
    box.export(tmp_path / "b" / "mandrel.stl")
    (tmp_path / "b" / "tool_surface.step").write_text("x")
    with pytest.raises(NotImplementedError, match="T3"):
        load_sample(tmp_path / "b")


def test_largest_smooth_runs(plate):
    mesh, truth, _ = plate
    pred = largest_smooth(mesh)
    assert pred.any() and pred.shape == truth.shape
