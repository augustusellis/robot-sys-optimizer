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

## Status

| Item | State | Notes |
| --- | --- | --- |
| 1-2 | done (314003c) | `core.path_direction.Rows` keeps rows per visit; coincident points take a neighbour's travel direction; none at all = `direction` issue. tests/test_v12_rows.py (the duplicate-point test fails on v11). |
| 3-7 | done (3626b81) | Unknown robots: Export asks for a format. Flipped normals: measured (cylinder r 50 mm: 48.5 mm either way); disagreeing normals reported. Two-sided groups: edge-check warning. >5 m in mm: unsure. Missing files: found by name in the project folder, kept in the saved file, *Locate missing files* dialog. Not tried in a browser: the Locate dialog's uploads. |
| 8 | done (4bbb111) | `Solution.id` / `Solution.key`; plans saved before v12 keep label keys. 22 test files moved from `.label` to `.key` lookups. |
| 9 | done (bb94a9b) | `ui.session_lock.LOCK` (one per process); guide 10.4a. Viser button callbacks are not wrapped (only drags, the frame placement and the background writers): wrapping every callback would serialise them and could deadlock a callback that waits for a planner. |
| 10 | done (b9755bd) | `core.collision_config.CollisionConfig`; tolerance and edge jobs had used the worker's leftover settings before (no collision settings in their specs). |
| 11 | done | `settings.missing` is empty on the built app; source check for duplicates; `RENAMED`. |
| 12 | done (41653f3) | ruff 0.17.3 pinned; lint job on ubuntu. Not run on GitHub (no push). |
| 13 | done (efe3999) | `ui/html_bridge.py`; tab bar / jog arrows byte-identical, joystick equivalent; e2e 12/12. |
| 14-15 | done (e7034b1, a8b28d7) | `core/path_sample.chord_poses` (v11 samples to 1e-12); `TraceOptions`. |
| 16 | done, partly (86fa591) | scipy for matrix -> rpy / quaternion; `rpy_to_matrix` kept closed form: scipy 43 us vs 6 us per call, on hot paths. |
| 17 | **dropped** | CP-SAT `AddHint` from the k - 1 cover, synthetic 40 m part, k = 20..32: full-coverage scene 26.5 -> 27.5 s (limit 2 s) and 36.2 -> 39.9 s (10 s), same covers; coverage-limited scene 9.9 -> 14.5 s and at a 1 s limit a worse k = 32 cover (265 vs 298 targets). The hint steers the search, it does not help it. Reverted. |
| 19 | **dropped** | gantry_overhead (UR20, `scripts/profile_frame.py`, 4 check workers). (a) Refined frame kept only if its quick plan beats the best grid frame's: kept (quick 13.75), but the frame that plans best is another grid frame (full plans 11.73 vs 13.72). (b) Quick-plan the top 4 distinct candidates, full-plan the 2 cheapest: quick costs 13.80 / 13.75 for full costs 11.73 / 13.72, same two frames chosen, +79 s (a quick plan takes ~20 s here, not ~3 s). The quick plan does not predict the full plan's ranking (refinement and zoning change it). Reverted. |
| 20 | done | The review's premise was wrong as stated (in frame coordinates the targets shift, so every lookup still differs). Instead: candidate stops on one world lattice (`lattice_anchor`, `Carrier.samples(anchor=)`), coverage rows cached by base pose and stop checks by the moving bodies' poses, shared by the frames of one ranking (exact: tests/test_v12_frame_cache.py). gantry_overhead (UR20, 51 frames): `rank_frames` 42.3 -> 8.0 s (map lookups 23.5 -> 1.1 s, stop checks 17.4 -> 5.4 s); whole Gantry placement 228 -> 147 s, same best frame (11.73), the refined frame now plans 12.81 (was 13.72). |
| 21 | done | `CollisionScene.part_clearance_grad`: nearest points + link Jacobian (`-n^T J_p`); vs central differences on 41 UR20 + sander poses 5-150 mm from a box: relative error median 3e-11, max 1.5e-8; one distance pass 5.0 ms vs 34.5 ms for the old 7-pass forward difference. CRX-30 gantry_overhead path build (smoothing on): 148.9 -> 105.2 s (-29 %), cycle 517 s both, 6 of 14 moves smoothed both, smoothed length 18.54 -> 18.51 rad. |
| 18 | done | Spin DP only where the planned-spin samples have a problem, a joint within 20 deg of its limit on the surface, or an unwinding step; Paths skips the whole spin build when the plan has none. rail_long_part (2 stops): build 7.3 -> 2.6 s, same paths (cycle 83 s, no flags). CRX-30 gantry_overhead (3 stops, `scripts/measure_paths.py`): 188.6 -> 148.2 s (-21 %), 7 of 12 sequences skip the DP, cycle 515 -> 517 s (+2 s), flags and joint margins unchanged. |
