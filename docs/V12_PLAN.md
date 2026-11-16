# V12 plan: review follow-ups (branch `v12`, from `v11`)

From `docs/V11_REVIEW.md` (2026-11-15). The user asked for items 1-24 below on branch `v12`.
Ground rules as in `docs/V11_PLAN.md` (rso env, files < 500 lines, guide in sync, commit locally
per task, no push, no `Co-Authored-By`, new test files take viser ports 8983+).

## Phase A: silent wrong output

1. Duplicate / coincident target points: the travel direction falls back to the neighbouring
   sample's; none at all = a `direction` issue (no silent spin 0).
2. Cross-hatch rows: `Seq.rows` must not let the second pass overwrite the first pass's row of a
   point.
3. Robot programs: no URScript guess for robots of unknown convention; Export asks for a format.
4. `surface_radius` with flipped face normals: orient by the target normal; report inconsistent
   normals instead of "flat".
5. Edge check: warn when a group has targets on opposite sides of a sheet.
6. Unit guess: implausible mm sizes (> 5 m) become unsure.
7. Missing files in old projects: offer to locate them instead of only dropping them.

## Phase B: risks

8. Plans keyed by an id (`Solution.id`), not the label.
9. One `RLock` on `Session` for GUI mutations; ownership table in the software guide.
10. Collision settings as one `CollisionConfig` value.
11. Saved-settings keys checked against the live app (no duplicates, every key resolves).
12. CI: ruff pinned; lint on Linux.

## Phase C: duplicated code

13. `ui/html_bridge.py` for the hidden-text-input JS.
14. One chord sampler for the three path builders.
15. `TraceOptions` for `_Tracer`.
16. `core/transforms.py` on `scipy.spatial.transform.Rotation`, gimbal convention kept.

## Phase D: performance

17. CP-SAT hint from the k solution for k + 1.
18. Spin DP only on sequences that need it.
19. Gantry placement: refine the top two frames on the quick-plan cost.
20. One coverage matrix shared by all frames.
21. Analytic clearance gradient in `traj_opt`.

## Phase E: tests and docs

22. Reduced scenes for the slowest tests; feature names for test files touched.
23. Test: `core/` and `opt/` never import viser or `ui`.
24. CLAUDE.md slimmed: release history to `docs/HISTORY.md`.

## Phase F: Gantry placement testing feedback (2026-11-16)

From the user's tests of Optimize -> Gantry placement with default settings.

25. **"Use this frame" leaves the fixed gantry bodies behind (gantry_overhead).**
    - *Found:* the stored carrier is right (frame, posts, beams, bridge and carriage turned and
      moved together; plans use it), but the scene is not. `CarrierPanel.commit()` -> `refresh()`
      only re-poses the moving bodies (`_move_bodies`) when the DOF ranges did not change, and the
      fixed bodies (posts, beams) are drawn only by `draw()`. After using the refined frame
      (x -150, y -300 mm, yaw 90 deg) the posts were drawn 4.5-5.0 m and the beams 2.2-2.9 m from
      where they are; bridge / carriage / drop were right. Any frame-pose edit that keeps the travel
      ranges has the same fault.
    - *Fix:* `commit()` redraws all bodies (`draw()`) when the frame pose or any fixed body changed
      (commit is not a hot path; jogging keeps `_move_bodies`).
    - *Done when:* after *Use this frame* on gantry_overhead every drawn carrier body is where the
      model has it (0 mm), for a turned and a shifted frame and for a typed frame-pose edit.

26. **"No frame works" on floor_xy_rails although the station planner reaches every point.**
    - *Found:* `opt.frame_place.candidate_frames` only centres gantry frames on the part (+-0.6 m,
      0.3 m steps, both headings): the overhead layout. On floor_xy_rails 45 of 50 frames put a floor
      X rail within the 300 mm standoff (or inside the 3 m wall) and the other 5 sweep the low bridge
      through the wall; the user's own frame (beside the part, y = -1.4 m, 425 mm clear) is never a
      candidate.
    - *User decisions (2026-11-16):* try frames **beside the part and over it** (over = the current
      centred frames, kept when the bridge clears the part); beside frames in **both headings**
      (X rails parallel to the side and across it) on **every side**; the standoff of a beside
      frame is measured from the part to the **nearest gantry body anywhere along its travel**
      (fixed rails / legs and the swept area of the bridge, carriage, riser), stepped over the
      existing standoff range (d_min..d_max, 300-1200 mm by default); the **current frame is always
      a candidate**, listed with the others and marked as the user's.
    - *Plan:*
      1. `opt/frame_place.py`: `swept_outline(base)` = the gantry's floor outline in its own
         frame: hulls of the fixed bodies plus the moving bodies at the travel corners (and both
         lift ends). `beside_frames(base, part_fp, st)`: for each side of the part's minimum
         rectangle (+-u, +-n) and each heading (X along the side / across it), the frame origin
         that puts the outline's near edge at the part outline + d (d over d_min..d_max at
         `step`), shifted along the side over +-`shift` at `shift_step`; mount / beam heights as
         for the other frames. `candidate_frames` = over frames + beside frames (+ the current
         frame), deduplicated.
      2. Frames keep being rejected by the existing checks (fixed bodies vs part 3D, keep-outs,
         stay-outs, `sweep_clear`); `sweep_clear` also sweeps the carriage bodies, not only the
         bridge.
      3. `FrameScore.own` marks the current frame; `frame_text` adds "(your frame)"; it is
         scored with the others (no refinement of it), and when it is usable but outside the top
         it is still listed so the result is never "no frame works" while it plans.
      4. Gantry placement panel: no new fields; the guide's Gantry placement section says where
         frames are tried (over / beside, headings, standoff measured to the nearest body).
      5. Time: about 4 sides x 2 headings x 7 distances x 5 shifts = 280 more frames per mount /
         beam height; with the v12 #20 caches ~0.15 s each, so ~40 s more scoring. If it is
         much slower, fewer shifts for beside frames (measured and stated in Status).
    - *Done when:* floor_xy_rails with default settings ranks at least two usable frames beside
      the part, the best one plans every point (as the station planner does from the user's
      frame) and *Use this frame* draws correctly (#25); the user's frame appears marked;
      gantry_overhead still finds a frame at least as cheap as before (11.73); scoring time
      stated for both examples. Tests: tests/test_v12_gantry_frames.py (port 8986).

## Phase G: Package builder feedback (2026-11-16, planned with the user, not started)

27. **End effector frame when loading a mesh / STEP into a tool package.**
    - *Found:* a tool package's frame is the robot flange (`tool0` / `fanuc_flange`: z out of the
      flange face); the builder window puts it exactly on the drawn flange (checked for all 10
      library robots: 0.000 mm, 0.000 deg). An imported file is placed with its own origin and
      axes on that frame, so a CAD file whose origin is not the mounting face comes out offset /
      turned (the user's case). Correcting it needed the part's pose fields or, after the import,
      *Frame from STEP -> Package origin*.
    - *User decisions (2026-11-16):* a **Load relative to** choice when adding a mesh / STEP to a
      tool package, with **Robot flange** (the default for end effectors: the file's origin and
      axes are the flange) and **STEP coordinate system** (a named CSYS of the STEP file is the
      mounting face).
    - *Plan:*
      1. Parts folder: dropdown *Load relative to* (tool packages only; other kinds keep the
         package frame): *Robot flange* (default) / *STEP coordinate system*.
      2. STEP + *STEP coordinate system*: after meshing, the file's named coordinate systems
         (already read, `Part.datums`): one -> used; several -> a dialog lists them (and *None: the
         file origin*); none -> placed on the flange with a note. The part's pose becomes
         inv(T_datum), so that CSYS sits on the flange (z out of the flange face). STL / OBJ / PLY
         have no coordinate systems: the choice is hidden for them (flange).
      3. The import status line says what was used ("STEP CSYS *mount* on the robot flange").
         The datums stay listed under *Frame from STEP* (TCP, robot mount, package origin).
      4. Guide: Building URDF packages section. Tests (`tests/test_v12_builder_frames.py`, port 8987):
         STEP fixture with a named CSYS -> the CSYS is at the flange (1e-9); default -> identity;
         several CSYS -> the dialog's choice is used; no CSYS -> flange + note.

28. **Four decimal places in every mm / deg field (user: every field, not only Frames and travel).**
    - *Found:* viser rounds a typed value to the decimals of the field's arrow step (pose fields:
      1 mm / 0.5 deg; travels: 10 mm), so 12.3456 became 12 or 12.5. viser's `precision` can be set
      apart from the step (checked), so the arrows keep their steps.
    - *Plan:* one hook where the app builds its GUI (main window and builder window): every
      number / vector / slider whose label is in mm or deg gets 4 decimals (`precision = 4`),
      including fields made later (pose widgets, carrier sliders, TCP rows); typed values are kept
      as typed (never rounded back). Tests: typing 12.3456 mm / 1.2345 deg into the builder's TCP and
      robot mount, the X travel, the Tool tab's TCP and the robot base gives 0.0123456 m /
      1.2345 deg in the model; every mm / deg field of the built app has precision 4.

## Status

| Item | State | Notes |
| --- | --- | --- |
| 1-2 | done (eadb0a7) | `core.path_direction.Rows` keeps rows per visit; coincident points take a neighbour's travel direction; none at all = `direction` issue. tests/test_v12_rows.py (the duplicate-point test fails on v11). |
| 3-7 | done (3626b81) | Unknown robots: Export asks for a format. Flipped normals: measured (cylinder r 50 mm: 48.5 mm either way); disagreeing normals reported. Two-sided groups: edge-check warning. >5 m in mm: unsure. Missing files: found by name in the project folder, kept in the saved file, *Locate missing files* dialog. Not tried in a browser: the Locate dialog's uploads. |
| 8 | done (4bbb111) | `Solution.id` / `Solution.key`; plans saved before v12 keep label keys. 22 test files moved from `.label` to `.key` lookups. |
| 9 | done (bb94a9b) | `ui.session_lock.LOCK` (one per process); guide 10.4a. Viser button callbacks are not wrapped (only drags, the frame placement and the background writers): wrapping every callback would serialise them and could deadlock a callback that waits for a planner. |
| 10 | done (b9755bd) | `core.collision_config.CollisionConfig`; tolerance and edge jobs had used the worker's leftover settings before (no collision settings in their specs). |
| 11 | done (2d7b821) | `settings.missing` is empty on the built app; source check for duplicates; `RENAMED`. |
| 12 | done (41653f3) | ruff 0.17.3 pinned; lint job on ubuntu. Not run on GitHub (no push). |
| 13 | done (efe3999) | `ui/html_bridge.py`; tab bar / jog arrows byte-identical, joystick equivalent; e2e 12/12. |
| 14-15 | done (e7034b1, a8b28d7) | `core/path_sample.chord_poses` (v11 samples to 1e-12); `TraceOptions`. |
| 16 | done, partly (86fa591) | scipy for matrix -> rpy / quaternion; `rpy_to_matrix` kept closed form: scipy 43 us vs 6 us per call, on hot paths. |
| 17 | **dropped** | CP-SAT `AddHint` from the k - 1 cover, synthetic 40 m part, k = 20..32: full-coverage scene 26.5 -> 27.5 s (limit 2 s) and 36.2 -> 39.9 s (10 s), same covers; coverage-limited scene 9.9 -> 14.5 s and at a 1 s limit a worse k = 32 cover (265 vs 298 targets). The hint steers the search, it does not help it. Reverted. |
| 19 | **dropped** | gantry_overhead (UR20, `scripts/profile_frame.py`, 4 check workers). (a) Refined frame kept only if its quick plan beats the best grid frame's: kept (quick 13.75), but the frame that plans best is another grid frame (full plans 11.73 vs 13.72). (b) Quick-plan the top 4 distinct candidates, full-plan the 2 cheapest: quick costs 13.80 / 13.75 for full costs 11.73 / 13.72, same two frames chosen, +79 s (a quick plan takes ~20 s here, not ~3 s). The quick plan does not predict the full plan's ranking (refinement and zoning change it). Reverted. |
| 20 | done | The review's premise was wrong as stated (in frame coordinates the targets shift, so every lookup still differs). Instead: candidate stops on one world lattice (`lattice_anchor`, `Carrier.samples(anchor=)`), coverage rows cached by base pose and stop checks by the moving bodies' poses, shared by the frames of one ranking (exact: tests/test_v12_frame_cache.py). gantry_overhead (UR20, 51 frames): `rank_frames` 42.3 -> 8.0 s (map lookups 23.5 -> 1.1 s, stop checks 17.4 -> 5.4 s); whole Gantry placement 228 -> 147 s, same best frame (11.73), the refined frame now plans 12.81 (was 13.72). |
| 21 | done | `CollisionScene.part_clearance_grad`: nearest points + link Jacobian (`-n^T J_p`); vs central differences on 41 UR20 + sander poses 5-150 mm from a box: relative error median 3e-11, max 1.5e-8; one distance pass 5.0 ms vs 34.5 ms for the old 7-pass forward difference. CRX-30 gantry_overhead path build (smoothing on): 148.9 -> 105.2 s (-29 %), cycle 517 s both, 6 of 14 moves smoothed both, smoothed length 18.54 -> 18.51 rad. |
| 22 | done, partly | Full fast suite on this branch before item 22: **582 passed, 0 failed, 2102 s** (30 batches of 4 files). Per-test timings of the three slowest batches show app start-up and project loads, not planning, dominate: most test files build their own app (~12 s each) and many load an example (~5 s). `test_work_state.py` now shares one second app between four tests: ~119 -> 84 s. The < 10 min target is **not** reached: it would need one app shared across test files (a session fixture), which the per-batch processes (needed for memory on this PC) prevent. Not done: reduced-scene rewrites of the `slow` tests, renaming test files by feature. |
| 23 | done (4357a2b) | tests/test_v12_layers.py: ast check of every import + a fresh-interpreter import of all core / opt modules. |
| 24 | done (07e5c58) | CLAUDE.md 536 -> 205 lines; docs/HISTORY.md. |
| 25 | done | `CarrierPanel.commit` redraws all bodies when the frame pose or a fixed body changed (`_fixed_moved`). After *Use this frame* (turned + shifted frame) and after a typed frame-pose edit every drawn body is where the model has it (0 mm); both tests fail on v11's code. |
| 26 | done | `swept_outline`, `beside_frames` (4 sides x 2 headings x standoffs x shifts), the user's frame always scored (`FrameScore.own`, "(your frame)", appended to the planned list when usable). floor_xy_rails (defaults): 281 of 331 frames usable (was 0 of 50); best frame beside the part, rails across the side (x 300, y -3500 mm, yaw -90): **1 station, 77/77, cost 7.32**; the user's frame 2 stations, 77/77, 8.94; whole Gantry placement 65 s. gantry_overhead: same best frame (11.73), the user's frame listed (12.74); scoring 7.1 -> 25.5 s (the beside frames), total 233 s with 3 frames planned. Not done from the plan: sweeping the carriage bodies in `sweep_clear` (each stop's check already rejects stops whose carriage is too close; moves between stops are checked when the paths are built). Not tried in a browser. |
| 18 | done | Spin DP only where the planned-spin samples have a problem, a joint within 20 deg of its limit on the surface, or an unwinding step; Paths skips the whole spin build when the plan has none. rail_long_part (2 stops): build 7.3 -> 2.6 s, same paths (cycle 83 s, no flags). CRX-30 gantry_overhead (3 stops, `scripts/measure_paths.py`): 188.6 -> 148.2 s (-21 %), 7 of 12 sequences skip the DP, cycle 515 -> 517 s (+2 s), flags and joint margins unchanged. |
