# V10 plan: task list (branch `v10`, from `v9`)

Agreed with the user on 2026-11-13 after the v9 code review. **Start a new session with:
"Implement docs/V10_PLAN.md on branch v10, task by task, in the order given."**

## Checkpoint

**Stop after task 10 and ask the user for confirmation before starting task 11.** Report what tasks
1-10 did (with measurements), what was not verified in a browser, and anything that changed the
plan. Tasks 11-22 start only after the user says to continue (they may change the order or scope).

## Ground rules

- Python `C:/Users/Ben/miniforge3/envs/rso/python.exe`; ruff `C:/Users/Ben/miniforge3/envs/rso/Library/bin/ruff.exe`
  (`ruff check . && ruff format .`); gh `C:/Users/Ben/miniforge3/Library/bin/gh.exe`.
- While working, run only the affected test files. The full fast suite runs in batches (memory is
  tight; run it in the foreground or watch it; don't restart a run that was killed for memory unless
  the user asks). Tests never write to `workspace/` (conftest sets `RSO_WORKSPACE`).
- Files under 500 lines: split before a file grows past that.
- Keep `docs/USER_GUIDE.md` in sync with every GUI label; rebuild the HTML with `scripts/build_guide.py`;
  add a v10 section to `CLAUDE.md` (one line per task).
- Commit locally after each task. Commits: no `Co-Authored-By` trailer (no `attribution.commit` in
  `.claude/settings.json`); end with the `Claude-Session:` line. Push / release only when the user asks;
  releases are drafts with the `scripts/make_portable.py` zip (new dependencies must be in it). Don't
  merge into main. Leave `examples/mobile_test/` and `v1_feedback.txt` untracked.
- Patch with the Edit tool or Python scripts written with the Write tool (`encoding="utf-8"`); heredocs
  mangle backslashes and non-ASCII.
- Anything only a browser can show goes in the final report as "not verified in a browser" unless an
  e2e test (tests/e2e, headless Edge) covers it.
- Each task ends with tests for its "done when".

## Part A: bugs found in the v9 review

1. **Edited tools keep their contact pad.** *Edit the project's tool* -> *Use in this project* merges
   the tool into one `mount` link, so `TCP.contact_links` (e.g. `pad_link`) and `contact_faces` point at
   links / faces that no longer exist (verified with the orbital sander: no tool piece keeps a contact
   signature). The builder writes one link per part with the original link name (read from the URDF);
   contact links and faces carry over; anything that cannot be mapped is reported (warning in the
   builder status and the log). *Done when* the sander edited and used through the builder still
   reaches its points with the pad as its contact geometry.
2. **Saving no longer clears plans.** `ui.project_assets.bundle` rewrites `Part.mesh_path`, and
   `ui.staleness.components` hashes the path, so the plans count as "part changed" and `clear_stale`
   drops them (verified). Fingerprint the part by file content (hash, cached by path + mtime) instead
   of its path (also tool / carrier meshes where hashed by path), or re-stamp when bundling rewrites a
   path. *Done when* a save that bundles an outside part keeps the plans and feasibility results.
3. **No session swap while viewing a compared robot.** `Session.snapshot` swaps the stashed robot /
   plans in and out; autosave runs it on a background thread. Build the project from the stash
   without touching the live session (e.g. `to_project(overrides=...)`). *Done when* snapshot leaves
   every session field untouched (test with a concurrent reader).
4. **Carrier edits keep "stretch with the axis"** (`Primitive.stretch`, used by three examples) and the
   bodies' existing collision meshes through the builder (Part fields + `to_base`). *Done when* the
   rail example's track still follows the travel after an edit round trip.
5. **Collision changes mark plans out of date:** the part's collision mesh (content), the arm padding,
   tool / carrier collision meshes join the setup fingerprint. `ui.cell_collision` rebuilds when a
   collision file changes on disk (mtime in its key). *Done when* changing any of them flags plans, and
   the jog / left-panel overlay show the new part collision mesh.
6. **Gantry placement fixes:** fixed bodies (track, legs, beams) are checked against keep-outs and
   stay-out zones; the height range resets for another project (and on carrier changes); *Use this
   frame* asks before replacing plans; the chosen Optimizer and the Gantry placement fields are saved
   with the project (ui.settings).

## Part B: speed and groundwork

7. **Reuse worker processes** across robots in Compare and across frames in Gantry placement (one
   checker pool per job, re-targeted per robot / frame), instead of starting a pool per run. Needed by
   task 17. *Done when* the gantry placement run (247 s for 50 frames + 2 plans with the UR20 on
   gantry_overhead, v9) is measurably faster.
8. **The rest of #10:** the Tool tab shows only the chosen tool type's fields; the home pose is set in
   one place; a status strip (each workflow step's state and what is out of date); the package builder
   gets a tab bar that shows the active tab (HTML bar using the joystick's hidden-input bridge).

## Part C: plan quality

9. **Free tool spin along the path.** Where a target group's spin is free (or a range), choose the spin
   per path sample (dynamic programming over spin steps) to minimise joint motion and keep away from
   joint limits; an option to turn it off. *Done when* the CRX-30 gantry_overhead example has fewer
   wrist jumps / unwinding moves and a shorter cycle time than v9 (measure both).
10. **Reach check dense at zone edges.** After planning, verification points along the boundaries
    between stops' areas and between configuration zones, along part edges and creases, and around
    low-margin points; exact checks; gaps reported (log, *Colour part by*, unreached view). Optional
    **Densify edges** adds the failing edge points as targets. Work points inside zones stay sparse.
    *Done when* a deliberately marginal case shows edge gaps the sparse points miss.

**-> Checkpoint: stop here and ask the user before task 11.**

## Part C2: tool contact and surface fit (added 2026-11-13)

11. **What may touch the part, chosen in the package builder.** For tool packages, per TCP: tick whole
    **parts** that may touch the part (the pad, a brush, a nozzle tip) and / or **pick STL faces** on a mesh
    part in the builder's 3D view (click, brush and box select like Targets; the one-ring tolerance of
    core.collision). Shown in the builder preview (green = may touch); saved in package.yaml; *Use in this
    project* writes them into the tool's TCPs (``contact_links`` = those parts' links, one link per part
    from task 1; ``contact_faces`` = face ids of that link's visual mesh). Editing the project's tool
    brings them back. Everything not picked still collides with the part. *Done when* a sander built from
    STL parts with only its pad face picked reaches its points, and the body still counts as a collision.
12. **Minimum surface radius per tool.** Each tool gets a **minimum surface radius** (mm; per TCP when a
    tool has several): set in the package builder for packages and in the Tool tab for library tools,
    saved with the tool. The local surface radius at every target is estimated from the part mesh over
    the tool's contact footprint (curvature from a quadric fit to the surface around the point; concave
    and convex separately, with an option for which ones the limit applies to: concave by default). A
    point whose radius is below the tool's minimum is **incompatible with the tool**: shown in Targets /
    *Colour part by* and the unreached view, listed in the log, and in the report ("N points not
    compatible with the tool's minimum radius R mm: surface radius r at points ..."); an option leaves
    them out of planning instead of only reporting them. *Done when* a part with a fillet tighter than the
    minimum flags exactly the fillet points in Results and the report.

## Part D: path algorithms

13. **Trajectory optimisation for the moves between points** (home, groups, configuration changes):
    smoother and shorter joint paths with clearance as a cost (CHOMP / TrajOpt style, numpy), seeded by
    the current RRT-connect + shortcut, which stays as the fallback.
14. **Time-parameterised trajectories** with joint velocity, acceleration and jerk limits (TOPP-style)
    for an accurate cycle time and smooth playback; the cycle-time panel and report use it.
15. **Coverage patterns:** contour-parallel and spiral passes for curved faces, raster passes with
    turn-arounds sized to the tool footprint; chosen per target group.
16. **Coordinated gantry / rail motion, as an option.** A **Carrier motion** setting: **Uncoordinated**
    (current stop-and-go: the carrier moves, then the arm works) or **Coordinated** (the carrier moves
    while the arm works: long parts in continuous passes). Coordinated planning splits each pass between
    the carrier axes and the arm within each axis's limits; paths, cycle time, playback, the report and
    the program export (task 21) support both. *Done when* the rail example can be built both ways and
    the cycle times are compared.

## Part E: comparisons

17. **Compare setups** (generalises Compare robots): tick robots and / or tools (library tools and builder
    packages), plan each combination, one table, any of them viewable in Results with the existing viewer
    (ui.compare_view); the project's own setup always included.
18. **Gantry placement improvements:** continuous refinement of the best frame after the grid search;
    distinct top frames (v9 planned two frames 300 mm apart with identical results); optional gantry beam
    height with legs that stretch.

## Part F: builder and collision details

19. **Collision options:** padding per robot link (e.g. forearm and wrist only); surfaces kept exact on
    parts with several meshes; Cancel for Simplify.
20. **Importing:** Cancel for STEP import; guess a mesh's units from its size (ask when unsure).

## Part G: outputs

21. **Robot programs:** export URScript (UR), TP (FANUC CRX) and RAPID (ABB) from the built paths,
    coordinated or not, with user frames and tool definitions.
22. **Report:** the comparison always includes the project's setup, with a picture per setup; gantry
    placement results get their own section; coordinated vs uncoordinated cycle time when both exist.

## Status

| Task | State | Notes |
|---|---|---|
| 1 | done | One URDF link per tool part; contacts carried over or reported. Found and fixed: yourdfpy drops a `<collision>` origin when the element is named like its link. Sander edited through the builder: 25/25 reached from the quick plan's stops; without the pad as contact all 25 collide. |
| 2 | done | Setup hashes follow file contents (cached by mtime); v9 stamps migrate on load. A save that bundles an outside part keeps the plan and the check (test); changing the file's content still flags them. |
| 3 | done | `to_project(overrides)`; test: 30 snapshots against a concurrent reader, every session field the same object. |
| 4-22 | not started | |
