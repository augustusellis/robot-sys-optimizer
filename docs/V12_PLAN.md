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
