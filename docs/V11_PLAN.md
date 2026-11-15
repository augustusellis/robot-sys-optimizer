# V11 plan: task list (branch `v11`, from `v10`)

From the user's feedback of 2026-11-15 (after v10 was pushed). Tasks 1-6 are done (see Status).
**To continue, start a session with: "Implement docs/V11_PLAN.md on branch v11, tasks 7-13, in
the order given, committing each."** Read `CLAUDE.md` (sections v10 and v11) first.

## Ground rules (as in docs/V10_PLAN.md)

- Python `C:/Users/Ben/miniforge3/envs/rso/python.exe`; ruff `.../envs/rso/Library/bin/ruff.exe`
  (`ruff check . && ruff format .`); gh `C:/Users/Ben/miniforge3/Library/bin/gh.exe`.
- Run only the affected test files while working; the full fast suite
  (`scripts/test_batches.py --fast`, ~32 min, 505 tests at the end of v10) once at the end.
  Tests never write to `workspace/` (conftest sets `RSO_WORKSPACE`). Each new test file takes its
  own viser port (v11 used 8963-8969; take 8973+).
- Files under 500 lines (several are close: `core/paths.py` 482, `ui/simulate.py` 493,
  `ui/targets_ui.py` 499, `ui/tool_setup.py` 489, `ui/package_builder.py` 483, `core/collision.py`
  498, `core/patterns.py` 477): put new code in new modules and leave one-line hooks.
- Keep `docs/USER_GUIDE.md` in sync with every GUI label, rebuild with `scripts/build_guide.py`
  (tests/test_guide.py fails otherwise); add one line per task to the v11 section of `CLAUDE.md`;
  fill the Status table below with measurements and what was not verified in a browser.
- Commit locally after each task: no `Co-Authored-By` trailer; end with the `Claude-Session:` line.
  No push / merge / release unless the user asks. Leave `examples/mobile_test/` and
  `v1_feedback.txt` untracked.
- Patch with the Edit tool or Python scripts written with the Write tool. **Shell heredocs mangle
  `\n` inside strings and can break quoting** (it happened repeatedly in v10): never put code with
  escapes in a heredoc.
- Viser runs widget callbacks on a thread pool, also for values set from code: guard callbacks
  (`_busy`) and expect them to arrive late (v10 T17 found two bugs of this kind).

## Part C: features (to do)

7. **Directional requirements for paths** (feedback 1). A tool says how it may travel over the
   surface relative to its own axes; the paths obey it.
   - Schema: `schema.TCP.travel: Literal["free", "along_x", "along_y", "perpendicular_x"] = "free"`
     (free = a sander: any direction, the tool may spin about the normal; along_x / along_y = a line
     scanner: the TCP's x or y axis along the travel direction; perpendicular_x = a drum sander:
     the drum axis (TCP x) across the travel direction). Add `scan_width_mm: float | None` (the
     stripe one pass covers across the travel) and `overlap_pct: float = 10` on the TCP, and on
     `schema.TargetGroup`: `overrun_mm: float = 0` (run past the group's edge, off the part) and
     `step_over: Literal["anywhere", "edges"] = "edges"` (moves along the other axis only at the
     group's edge or off the part). Mirror the TCP fields on `urdf_package.Frame` (as v10 T12 did
     for `min_radius_mm`: `to_tool`, `tool_contacts.frames_with_contacts`,
     `urdf_package_read._authored_contacts`, `package_actions._tool_from` keep list) and add the
     widgets next to *Minimum surface radius* (`ui/radius_ui.RadiusFields` for the Tool tab,
     `ui/package_contacts.ContactPicker` for the builder).
   - Planning the spin: the target's spin about the normal is no longer free for a directional TCP.
     The simplest correct place is `core.path_spin` (v10 T9): `allowed_from(groups, res, group_of)`
     returns the allowed spin per point; for a directional TCP return a **fixed** spin per sample =
     the angle that puts the TCP axis along (or across) the local travel direction (tangent of the
     ordered path, projected into the target's tangent plane; spin reference = `target_frame` in
     core.feasibility: x = part +x projected). `optimize_sequence` then has one spin choice per
     sample (plus 180 deg for a scanner / drum, which are symmetric) and its DP picks the joints.
     The station planner must also know: targets of a directional group need a pose at the travel
     spin, so set the group's `spin_deg` to the row direction's angle range before planning
     (rows from `core.patterns` raster angle: travel direction is known per group when its pattern
     is a row pattern; refuse non-row patterns for directional TCPs with a clear message).
   - Coverage: with `scan_width_mm`, the row spacing a group needs is
     `scan_width * (1 - overlap/100)`; Targets -> generators get a **Rows for the tool** button
     that sets the row spacing from the group's TCP (surface grid / raster generators in
     core.targets / core.surface_grid) and reports the overlap of the existing points
     (row spacing from `core.patterns.group_rows`).
   - Overrun and step-over: in `core.path_order.ordered_sequences` (row patterns), extend each
     row by `overrun_mm` at both ends along the row direction (extra path samples off the part:
     `_Tracer.segment` projects chords onto the mesh; overrun samples must skip the projection:
     add them as straight extensions in the tangent plane of the row's end point, kind `overrun`,
     checked for reach / collision like the others), and make the move to the next row happen at
     the overrun ends (`step_over = "edges"`), lifted when `Lift on wide turns` says so.
     `core.cycle_time._kind`: `overrun` counts as process.
   - *Done when*: (a) a scanner TCP (`along_x`, 60 mm width, 10 % overlap) on the flat plate
     scans rows 54 mm apart, every surface sample's TCP x within 2 deg of the travel direction,
     rows extended 20 mm past both edges, row changes only off the part; (b) a drum TCP
     (`perpendicular_x`) has its x axis within 2 deg of across-travel on a zigzag; (c) the sander
     (`free`) paths are unchanged (same samples as before: regression on rail_long_part).
8. **Compare external axis types** on the same part and points (feedback 3): static arm, rail,
   floor XY gantry, overhead gantry, mobile.
   - Generalise v10 T17 (`ui/compare.py`): a setup gets a third dimension `axis` = a `schema.Base`
     (None = the project's). `setups(robots, tools, own_robot, own_tool)` -> add `axes: dict[name,
     Base]`; `compare_job` sets `s.base = su["axis"]` (and `s.T_world_base` for a static base)
     before `station_job`; keys `robot · tool · axis`; `ComparedRobot.base: Base | None`;
     `CompareView` STASH + `base` (and `carrier.load` / refresh on view, as `FramePanel.use_frame`
     does; keep the confirmed steps as v11 T1 does).
   - Axis choices (`axis_choices(s)`): the project's own, *Static (robot at its current base)*,
     and one per carrier type built from `ui/carrier_defaults.DEFAULTS` placed relative to the
     part with `opt.frame_place` (rail / gantry: the best-scoring grid frame, no full plans:
     `rank_frames(...)[0]` with `top` irrelevant; mobile: defaults with `fixed_standoff_m`), plus
     carrier packages in `<project>/assets/packages/*` (kind rail / gantry / mobile via `to_base`).
     Overhead gantry = the gantry default with the gantry_overhead example's mount (inverted,
     frame at 4.4 m) : reuse `scripts/make_examples.py` builders rather than new geometry.
   - Table columns add the axis and its travel / footprint; the report's *Setup comparison*
     (ui/report_sections.py) already draws a picture per setup: add the carrier's fixed-body
     outline (`Carrier._hulls(static_bodies())`) to `core.report_svg.plan_view`.
   - *Done when*: rail_long_part compared as static / rail / floor gantry / overhead gantry /
     mobile gives five rows with stations, reach, cycle and cost; each is viewable in Results
     (the carrier swaps in and out, the project's own base is saved meanwhile).
9. **Mobile robot with a vertical stage in the package builder** (feedback 4).
   - `urdf_package.STAGES["mobile"]` already has stages chassis and lift (check); what is missing
     is a guided way to build the full robot: in the builder's *Mobile robot* tab add **Add
     vertical stage** (a mast on the chassis = fixed part on stage `chassis`, a carriage plate on
     stage `lift`, sets *Lift travel* and puts the *Robot mount* on the carriage) and **Add
     chassis** (box with the footprint fields). `build_urdf` writes `lift_link` on a prismatic
     z joint already (core/carrier_urdf.py for carriers; check `urdf_package.build_urdf` writes the
     same chain: world -> x -> y -> yaw -> chassis -> lift -> mount) and `to_base` maps it to
     `Base.joints.lift` + bodies with `attach="lift"`.
   - *Done when*: a mobile package built with the two buttons exports a URDF whose `mount` frame
     moves with the lift joint (Pinocchio FK test, as tests/test_v8_packages.py does) and, used
     in the mobile_amr project, plans with the lift as a DOF (`Carrier.dofs` has `lift`).
10. **Coordinate systems from STEP files as frames** (feedback 5).
    - `core/step_datums.read_datums` already reads named AXIS2_PLACEMENT_3D from STEP text (used
      by Tool -> Mesh / STEP tool). In the builder, keep the datums of every imported STEP part
      (`import_mesh`: read them before meshing; `core.mesh_units.step_in_process` only meshes:
      read datums in the parent, they are text) on the builder (`b.datums[part name] = [Datum]`,
      posed in the part's frame; apply the part's pose when using them).
    - One dropdown **Frame from STEP** (`part / datum name`) + **Assign to** dropdown: *TCP
      (selected)*, *Robot mount*, *Vertical stage mount* (the lift carriage part's pose), *Floor
      frame* (shifts every part so the datum becomes the package origin), *Part origin* (part
      packages) + an **Assign** button. Module `ui/package_frames.py` (builder is at 483 lines).
    - Limitation to state in the guide: placements nested in assemblies are not composed
      (core/step_datums docstring), only top-level named placements.
    - *Done when*: tests/step_fixture.py's probe (it injects named datums) imported in the builder
      lists its datums; assigning one to the TCP gives the TCP that pose (1e-6); assigning one to
      the robot mount of a mobile package moves `Package.mount`.

## Part D: review (to do)

11. **Senior-engineer review** -> `docs/V11_REVIEW.md` (the user asked for direct, not diplomatic).
    Sections, each with file:line references:
    1. *Architecture and trade-offs*: Viser-only GUI; `core` / `opt` free of viser (check with
       grep that this still holds); the `Session` object as shared mutable state between GUI
       threads, the autosave thread and job threads; module-level globals in `core.collision`
       (`GEOM`, `ARM_PAD`, `LINK_PAD`, `PART_LOW`, `DRESS_M`, `HOSE`) shipped by hand through
       `geom_state()` / job specs; `app.py` as a 570-line wiring function with cross-panel
       attribute injection (`x.sim = sim`, `targets.edge_gaps = ...`); the hidden-input HTML bridge
       used three times (joystick, tab bar, jog arrows) without a shared helper; settings saved by
       attribute name (`ui/settings.SAVED`) so renames silently lose settings; results kept in
       parallel dicts keyed by plan **label** (`result_points`, `stamps`, compared plans).
    2. *Abstractions at the wrong level*: `core.paths._Tracer` (segment / joint_move / around /
       retreat with flags set from outside: `park`, `via_home`, `smooth`); three path builders
       that duplicate sampling (`trace`, `core.coordinated._pass`, `path_spin.optimize_sequence`);
       `FrameScore` / compare rows / frame rows as untyped dicts; version-named tests and
       docstrings (v6..v11) instead of feature names; the 500-line rule producing thin shim
       modules (`sim_fields`, `sim_build`, `package_actions`) that share one object's state.
    3. *What will hurt*: thread-pool callbacks + unguarded session mutation; label-keyed plans;
       no schema migration story beyond `REMOVED_FIELDS`; CLAUDE.md as the only architecture
       record (now ~600 lines of history); slow suite (32 min fast, >1 h full) on a memory-tight
       PC; exports not validated on controllers; CI flake seen 2026-11-15 (ruff crash).
    4. *Edge cases not handled* ("every edge case"): go module by module and list concretely, at
       least: empty / single-point groups in patterns and coordinated passes; degenerate normals;
       targets on non-manifold or open meshes for `surface_radius` and `edge_check`; 2*pi wrap at
       joint limits in `trajectory.time_path` (zero-length segments, NaN on repeated waypoints);
       `core.coordinated` with a lift DOF, with `held` axis at travel limits, with unreachable
       first sample; `robot_program` with several TCPs, with unreachable samples (skipped
       silently), numeric robots without a format; `mesh_units.guess_units` for inch files;
       `step_in_process` when cadquery is missing; `compare` with an uploaded (non-registry)
       robot; `use_frame` while viewing a compared setup; projects moved between PCs (absolute
       paths in `PART_LOW`); two browser clients at once; cancel during the edge check's pool run.
       Verify each claim against the code before writing it down; mark the ones confirmed by a
       quick test.
    5. *Optimization methods*: review `opt/placement.py` (coverage -> CP-SAT set cover -> TSP ->
       refinement), `opt/refine*.py`, `opt/zoning.py`, `opt/areas.py`, `opt/tool_*`,
       `opt/frame_place.py`, `core/path_order*.py`, `core/path_spin.py`, `core/traj_opt.py`,
       `core/trajectory.py`. List opportunities with expected gain and cost, e.g.: analytic
       clearance gradients (closest points + Jacobian) for traj_opt instead of finite differences
       (measured: +80 s per build); batch collision checks across IK branches; warm-started
       CP-SAT between station counts; caching coverage matrices across frames in gantry placement
       (frames differ by a translation); refining frames on the full-plan cost for the top 2
       instead of the map score (v10 T18 showed the map score misleads); Viterbi spin search
       pruned by joint-limit feasibility first; reuse of the uncoordinated plan's samples in
       coordinated passes; profile with `scripts/profile_plan.py` / `profile_frame.py` /
       `measure_paths.py` and quote numbers from `docs/V10_PLAN.md` Status.
    6. *Simpler implementations* (only where genuinely simpler; say so where the code is already
       fine): candidates to check: `scipy.signal`/`np.convolve` already used for the jerk filter
       (fine); `scipy.sparse.csgraph.dijkstra` instead of the hand-written heap in
       `patterns_surface.surface_distance`; `scipy.spatial.transform.Rotation` everywhere instead
       of `core.transforms` helpers that duplicate it; one `html_bridge` helper for the three HTML
       bridges; dataclasses for the dict rows; `functools.lru_cache` for the session-attribute
       caches (`_radius_fit`); pydantic `model_copy` chains in the builder -> a small `update()`
       helper; collapse `sim_fields` / `sim_build` back into one module if the 500-line rule is
       relaxed for it.
12. **Missing unit tests** (write them; pure-core first, no viser where possible): from the edge
    cases of 11.4 that turn out to be real, plus gaps seen in v10: `core.robot_program._fanuc_config`
    / `_abb_conf` against known poses; `core.trajectory._limit_jerk` on a triangle profile
    (jerk actually within the limit at tau/2); `core.cycle_time.runs` / `_kind` table test;
    `core.coordinated` on a gantry base (unit, no app); `core.surface_radius.footprint_of` with
    contact faces; `core.edge_check` zone points with real configurations; `ui.compare.setups`
    with duplicates; `opt.frame_place.candidate_frames` with beam heights; `core.mesh_units`
    boundaries. Bugs found while writing them get fixed in the same commit and listed.
13. **Software guide for engineers** in `docs/USER_GUIDE.md` (new last chapter "For software
    engineers", so it ships in the HTML guide): languages (Python 3.12 only; HTML / JS only as
    inline snippets for the bridges; no build step), packages and what each is used for (viser,
    yourdfpy, pinocchio, coal, trimesh, numpy / scipy, shapely, ortools, cma, optuna, cadquery,
    manifold3d, fast_simplification, numba, pydantic, markdown-it-py, pytest, ruff), the layers
    (`core` math / `opt` optimizers / `ui` panels / `app.py` wiring), a one-paragraph "how it
    works" per subsystem (kinematics + IK, collision, feasibility, reach maps, station planner,
    zoning, paths, timing, exports, package builder, jobs / worker processes, session + project
    file + staleness), how to run / test / lint / build the portable zip, and where to start
    reading. Keep it brief (2-3 pages) and link to CLAUDE.md for history.

## Status

| Task | State | Notes |
|---|---|---|
| 1 | done | *Use this frame* re-confirms the steps that were confirmed before (test: floor_xy_rails, all four steps stay confirmed, `setup_reason` empty). Also checked end to end on gantry_overhead: frame placed and used, steps confirmed, Plan stations runs (108/108, 2 stops). |
| 2 | done | The model always moved rails, bridge and carriage together (checked on floor_xy_rails); the display kept the carrier values that held the robot where it was, so the bridge looked left behind: after *Use this frame* the carrier is set to the plan's first stop. New `sweep_clear`: a gantry frame whose bridge comes closer than the carrier standoff to the part anywhere along its X travel is refused (test: rails astride the tall wall refused, beside it and the overhead bridge accepted). Not verified in a browser. |
| 3 | done | The Optimizer list offers *Gantry placement* only with a rail or a gantry (watcher hook). Overhead gantry placement works on the shipped example (see task 1); if the user's own gantry failed, their project is needed. Other axis-specific options were already conditional (Paths *Carrier motion*, External axis fields). |
| 4 | done | *Inspect result* folder sits between the plan list and the summary (order checked in a test). |
| 5 | done | Optimize -> **Confirm all setup steps**: dialog lists each unconfirmed tab and why; no dialog without a browser client (tests drive both paths with a fake client). Not verified in a browser. |
| 6 | done | Jog arrows (HTML pad, click = one step, held = a step every 0.12 s after 0.35 s); the old step buttons are in *Single steps*. Tested through the bridge input; press-and-hold itself not verified in a browser (an e2e test like tests/e2e/test_v10_browser.py would do it). |
| 7 | done | Flat plate, UR20 + sander pad as the TCP, rows 54 mm apart (row_pitch(60 mm, 10 %)), Zigzag, overrun 20 mm: every surface sample's TCP x within 2 deg of the row direction (along_x) / y (drum: perpendicular_x), no surface sample joins two rows, overrun samples reach 20 mm past both row ends, 0 collision / unreachable / direction samples; a free TCP gives identical samples. rail_long_part in the app with the sander set to along_x: lead-in / lead-out overrun samples, surface samples follow the rows (or are flagged). Deviations from the plan: the TCP carries overrun (not the target group: targets_ui is at 499 lines and one tool usually has one overrun); step-over is always at the row ends (no 'anywhere' option: with a directional tool the sideways step only happens there); the station planner does not plan for the travel spin (documented). Also fixed a stale v10 T16 test (Carrier motion section). Not verified in a browser: the new fields. |
| 8 | done | rail_long_part compared on Static arm / Rail / Floor XY gantry / Overhead gantry / Mobile base: six rows (+ the project) each with stations, reach, cycle and cost; the floor-gantry setup viewed in Results (carrier swapped in, saving keeps the project's rail, back restores it) and saved / reopened with its base. Found: the shipped overhead gantry (beam 4.4 m) cannot reach a 0.65 m high part at all (25/25 uncovered at every frame) -> the comparison also tries lower beam heights down to 1.2 m above the part (it then plans 25/25 at 1.85 m). Not verified in a browser: the External axes checkboxes. |
| 9 | done | *Add mobile base* + *Add vertical stage* (mast 1.5 m, travel 1.0 m): URDF robot_mount moves 0.6 m with the lift joint at 0.6 (Pinocchio FK), `to_base` gives a mobile carrier with `lift` in its DOFs and the plate on the lift stage. The URDF chain and to_base already supported the lift; the task was the guided build. |
| 10 | done | STEP probe (two named datums) imported in the builder: both listed; CSYS_TCP -> TCP at (0, 0, 130 mm), z up; CSYS_FLANGE as the package origin re-expresses the TCP at 140 mm along its -z; datums saved and read back with the package; a datum as the robot mount / vertical stage mount moves the mount / the lift parts (mount rides along). Not verified in a browser. |
| 11-13 | in progress | |
