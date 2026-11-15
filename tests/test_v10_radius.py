"""v10 T12: minimum surface radius per tool (core.surface_radius, ui.radius_ui).

Done when a part with a fillet tighter than the minimum flags exactly the fillet points in Results
and the report.
"""

from types import SimpleNamespace

import numpy as np
import pytest
import trimesh
import viser

from core import surface_radius as sr
from core.schema import Solution, Station, TargetGroup, TargetPoint, Targets
from tests.fillet_part import fillet_corner, fillet_point

E = SimpleNamespace(client=None)
R = 0.010  # fillet radius
FLOOR_X = (0.1, 0.2, 0.3)
WALL_Z = (0.1, 0.25)
FILLET_T = (0.2, 0.5, 0.8)
YS = (0.08, 0.15, 0.22)


def _targets():
    """Floor and wall points (> 85 mm from the fillet) and points on the fillet; ids of the latter."""
    pts, fil = [], []
    for y in YS:
        for x in FLOOR_X:
            pts.append(((x, y, 0.0), (0, 0, 1)))
        for z in WALL_Z:
            pts.append(((0.0, y, z), (1, 0, 0)))
        for t in FILLET_T:
            p, n = fillet_point(R, y, t)
            fil.append(len(pts))
            pts.append((tuple(p), tuple(n)))
    return pts, fil


def test_fillet_radius_concave_and_flats_flat():
    mesh = fillet_corner(R)
    pts, fil = _targets()
    P, N = np.array([p for p, _ in pts]), np.array([n for _, n in pts])
    rad = sr.surface_radii(mesh, P, N, footprint=0.005)  # a small footprint sees the fillet itself
    for i, r in enumerate(rad):
        if i in fil:
            assert r.concave == pytest.approx(R, rel=0.15) and r.convex > 1.0
        else:
            assert not np.isfinite(r.concave) and not np.isfinite(r.convex)
    # a wider footprint also sees the walls rising: tighter still, never looser than the fillet
    wide = sr.surface_radii(mesh, P[fil], N[fil], footprint=0.025)
    assert all(R * 0.5 < r.concave < R * 1.1 for r in wide)


def test_cylinder_is_convex():
    c = trimesh.creation.cylinder(radius=0.05, height=0.3, sections=64)
    r = sr.surface_radii(c, np.array([[0.05, 0, 0]]), np.array([[1.0, 0, 0]]), 0.02)[0]
    assert r.convex == pytest.approx(0.05, rel=0.1) and not np.isfinite(r.concave)
    assert sr.limit_radius(r, "concave")[0] == np.inf
    assert sr.limit_radius(r, "convex") == (r.convex, "convex")
    assert sr.limit_radius(r, "both") == (r.convex, "convex")


def test_fit_lines_and_tcp_choice():
    fit = sr.Fit(bad={7: ("pad", 0.0101, 0.025, "concave"), 8: ("pad", 0.0123, 0.025, "concave")})
    s = fit.lines()
    assert s.startswith("2 point(s) not compatible with the tool's minimum radius 25 mm (TCP pad)")
    assert "7 (10 mm concave), 8 (12 mm concave)" in s


@pytest.fixture(scope="module")
def app():
    from app import build_app

    server = viser.ViserServer(host="127.0.0.1", port=8935, verbose=False)
    st = build_app(server)
    yield st
    st["builder_window"].stop()
    server.stop()


def _setup(app, tmp_path):
    from core.transforms import make_T
    from ui.session import PartState

    s = app["session"]
    assert app["project"].load_example("rail_long_part")
    path = tmp_path / "fillet.stl"
    fillet_corner(R).export(path)
    s.part = PartState(str(path), "m", make_T(p=(1.0, 0.0, 0.5)))
    pts, fil = _targets()
    g = TargetGroup(name="corner")
    s.targets = Targets(
        groups=[g], points=[TargetPoint(id=i, p=p, n=n, group="corner") for i, (p, n) in enumerate(pts)]
    )
    tool = app["tool"]
    tool.refresh()
    tool.min_radius.radius.value = 25.0  # Tool tab -> TCPs -> Minimum surface radius (mm)
    assert s.tool.tcps[0].min_radius_mm == 25.0 and s.tool.tcps[0].radius_applies == "concave"
    return s, pts, fil


def test_fillet_points_flagged_in_results_and_report(app, tmp_path):
    from ui.part_heatmap import FIT
    from ui.radius_ui import fit_of, skip_ids
    from ui.stations import world_targets

    s, pts, fil = _setup(app, tmp_path)
    fit = fit_of(s)
    assert sorted(fit.bad) == fil  # exactly the fillet points
    assert all(fit.bad[p][3] == "concave" and fit.bad[p][1] < 0.025 for p in fil)
    # Results: the selected plan's card names them; the report has a Tool fit section
    sol = Solution(label="P", stations=[Station(xyz=(0, 0, 0))], assignment={i: 0 for i in range(len(pts))})
    s.results = [sol]
    app["results"].refresh()
    card = app["results"].card.content
    assert "**Tool fit:** 9 point(s) below the tool's minimum surface radius" in card
    secs = dict(app["results"].export.sections(sol))
    assert "Tool fit" in secs
    html = secs["Tool fit"]
    for p in fil:
        assert f"{p} (" in html
    for p in set(range(len(pts))) - set(fil):
        assert f" {p} (" not in html and f">{p} (" not in html
    # Targets: markers and Colour part by
    targets = app["targets"]
    targets.refresh()
    assert all(f"unfit_label_{p}" in targets._handles for p in fil)
    targets.heat.choice.value = FIT
    targets.heat.draw()
    assert (targets.heat.face_bins >= 0).any()
    # convex only: an inside fillet is fine
    app["tool"]._commit(lambda d: d["tcps"][0].update(radius_applies="convex"), E)
    assert not fit_of(s).bad
    app["tool"]._commit(lambda d: d["tcps"][0].update(radius_applies="concave"), E)
    # leave them out of planning: the planners get the other points only
    app["stations"].skip_unfit.value = True
    assert skip_ids(s) == fil
    t = world_targets(s, skip_ids(s))
    assert sorted(int(i) for i in t.ids) == sorted(set(range(len(pts))) - set(fil))
    assert "left out of planning" in dict(app["results"].export.sections(sol))["Tool fit"]
    app["stations"].skip_unfit.value = False


def test_builder_frames_carry_the_minimum_radius(tmp_path):
    from core.urdf_package import Frame, Package, Part, to_tool, write_package, zip_folder
    from core.urdf_package_read import open_package

    p = Package(
        kind="tool",
        name="t",
        parts=[Part(name="pad", stage="tool", shape="box", size=(0.1, 0.1, 0.02))],
        tcps=[Frame(name="tcp", min_radius_mm=40.0, radius_applies="both")],
    )
    t = to_tool(p, "t.urdf").tcps[0]
    assert t.min_radius_mm == 40.0 and t.radius_applies == "both"
    write_package(p, None, tmp_path / "t")
    back, _ = open_package(zip_folder(tmp_path / "t"), tmp_path / "back")
    assert back.tcps[0].min_radius_mm == 40.0 and back.tcps[0].radius_applies == "both"
