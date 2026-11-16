# Code review: robot-sys-optimizer (v11, 2026-11-15)

Written as a senior engineer who has lived in this repo for years. Direct on purpose. Every claim
below was checked against the code at branch `v11`; file references are to that state. Items marked
**(fixed, tested)** were cheap enough to fix in task 12 and are pinned by `tests/test_v11_review.py`;
everything else is still open.

Size for context: 34 k lines of Python in `core/`, `opt/`, `ui/` (173 modules), `app.py` 570 lines,
107 test files, a fast suite of ~32 minutes and a full suite over an hour.

## 1. Architecture and trade-offs

**What is right and should be kept.**

- **The layering holds.** `core/` (geometry, kinematics, collision, paths) and `opt/` (planners)
  import neither viser nor `ui/` (`grep -rln "viser\|from ui" core opt` is empty). That is the one
  architectural rule that made ten rounds of GUI churn survivable. Keep it enforced; add a test.
- **One language, no build step.** Python end to end, Viser for the 3D GUI, no hand-written
  frontend. For a one-person tool this was the correct call: every feature landed in one place.
- **Typed project file.** `core/schema*.py` (pydantic, `extra="forbid"`) is the contract between
  sessions, workers and saved projects, and it caught real bugs. `REMOVED_FIELDS` is a crude but
  working migration.
- **Exact checks are the source of truth.** Reach maps, set cover and refinement are all
  heuristics that are verified by the exact checker at the end (`opt/placement.verify`). This is
  why the planner's output can be trusted even when a heuristic is wrong.

**What it costs.**

- **`Session` is a shared mutable god object.** `ui/session.py` holds the cell, tool, part,
  targets, plans, joint solutions, stamps, compare stashes and caches, and it is read and written
  by viser's callback thread pool, the 1 s watcher, the autosave thread and the planner threads
  (21 `threading.Thread(` call sites). There is no lock and no ownership rule; correctness rests on
  "the planners work on a snapshot" (`plan_job.payload_for`) and on luck elsewhere. Two bugs of
  exactly this kind were found in v10 (the Tool tab reloading a tool from a late callback; saving the
  viewed compare tool). More will come.
- **Module-level globals carry collision settings.** `core/collision.py` lines 42-60 (`DRESS_M`,
  `HOSE`, `GEOM`, `PART_KEEP`, `ARM_PAD`, `LINK_PAD`, `PART_LOW`, `PART_LOW_SCALE`) are set by the GUI
  and shipped by hand into job specs (`geom_state()`, `spec["dress_m"]`, `spec["hose"]`,
  `CheckerSpec.geom`). Every new setting needs edits in four places and one missed place silently
  plans with the wrong geometry. These belong in one `CollisionConfig` value passed to
  `CollisionScene`.
- **`app.py` is a 570-line wiring function** with 38 lines of the form `panel.attr = other` (cross-
  panel injection: `ex.sim = sim`, `targets.edge_gaps = ...`, `compare_view.carrier = carrier`). It
  works, but the dependency graph only exists in that file's order of statements; nothing tells you
  which panel may call which.
- **Plans are keyed by their human-readable label.** `Solution.label` ("2 rail stations: 25/25
  reached") is the key of `session.result_points`, `session.stamps`, the compare stash and the paths
  (`SimPanel.label`), 18 lookups. Labels are unique only by convention (the GUI replaces plans per
  station count). Two planners producing the same label overwrite each other's joint solutions.
  A plan id would cost one field.
- **Settings are saved by attribute name** (`ui/settings.SAVED`, ~100 keys like `"sim.smooth"`).
  Renaming a widget attribute silently drops the user's saved setting; a duplicate key silently drops
  the other one (it happened in v10 T21 and was only caught by reading the dict).

## 2. Abstractions at the wrong level

- **`core.paths._Tracer` is a bag of flags.** `park`, `via_home`, `smooth`, `smooth_stats` are set
  from outside after construction (`core/paths.py` `trace`), and `segment` / `joint_move` /
  `change_move` / `around` / `retreat` each consult them. It is a procedure split into methods, not
  an object; a `TraceOptions` dataclass passed once would make it honest.
- **Three path builders duplicate the sampling.** `_Tracer.segment`, `core.path_spin.segment_poses`
  and `core.coordinated._pass` each project chords onto the mesh, interpolate normals and slerp
  orientations (the same ~20 lines). A fix to one (for instance the overrun samples of v11 T7) has to
  be made in the others by hand.
- **Results travel as untyped dicts.** Compare rows, frame rows (`ui/frame_ui.frame_job`) and the
  report rows are `dict`s with 10-15 keys whose spelling is the interface (`"stations"`, `"reached"`,
  `"stops_xy"`, `"placed"`). A dataclass per row type would let the type checker catch the next
  rename.
- **The 500-line rule produces shim modules.** `ui/sim_fields.py`, `ui/sim_build.py`,
  `ui/package_actions.py`, `ui/package_presets.py`, `ui/package_frames.py`, `ui/travel_ui.py` exist
  mostly because their host files hit the limit; they take the host object (`panel`, `b`, `tab`) and
  reach into its attributes. That is the same coupling in more files. The rule is fine for `core`;
  for GUI panels, split by responsibility, not by line count.
- **Version-named everything.** Tests (`test_v8_*` .. `test_v11_*`), comments ("v10 T12:") and
  CLAUDE.md sections are organised by release, not by feature. Finding "the tests for collision
  padding" means grepping four files. Rename tests by feature when they are next touched.
- **The HTML input bridge is copy-pasted three times.** `ui/joystick.py`, `ui/package_tabs.py` and
  `ui/jog_arrows.py` each carry their own `_FIND` / `_HIDE` JavaScript for the same hidden-text-input
  trick. One `ui/html_bridge.py` with `find_js(prefix)`, `hide_js(prefix)`, `send_js(prefix, payload)`
  would make the next one (and a fix to the trick) one place.

## 3. What will hurt

1. **Threading without ownership** (above). The next intermittent bug report will be a race. Cheap
   first step: route every GUI mutation of `Session` through one `threading.RLock` held by the
   watcher and the callbacks; planners already work on snapshots.
2. **Label-keyed plans** (above): a silent wrong-joints bug waiting for a new planner.
3. **CLAUDE.md is the architecture document**, and it is now ~650 lines of history by release.
   New contributors read history to learn structure. The software chapter of the user guide (v11
   T13) is the start of a real one; keep CLAUDE.md for conventions and decisions.
4. **Test runtime.** 32 min fast / >1 h full on a memory-tight PC means the full suite is not run
   per change (it was run once in v10 and once at the end of each round). Most slow tests plan a whole
   example; the same assertions on a 6-point scene would run in seconds.
5. **Exports are unvalidated against controllers.** URScript / TP / RAPID (v10 T21) are checked by
   recomposing poses in Python, never by a controller or a simulator. Configuration data
   (`_fanuc_config`, `_abb_conf`) is derived by rule of thumb.
6. **CI flake.** The 2026-11-15 run lost its Lint step to a ruff crash on the Windows runner; pin
   ruff (`environment.yml` has `ruff>=0.8`) and add a retry or run lint on Linux.

## 4. Edge cases not handled

Grouped by module. "Silent" = no error and no flag: the output is just wrong.

**Programs (`core/robot_program.py`)**
- Several TCPs in one plan: one tool definition was written (`export_program` takes the first
  sample's TCP) and every point expressed for it. Now refused with a message; per-TCP tool switching
  in the programs is still to do. **(fixed, tested)**
- Path samples without a joint solution (`q is None`, flagged unreachable in the paths) were dropped
  from the program without a word. Now a `skipped (point a -> b): no joint solution` comment.
  **(fixed, tested)**
- Numeric-IK robots (uploaded URDFs) get URScript by default (`default_format`), which is wrong for
  anything that is not a UR.
- FANUC CONFIG and ABB confdata are heuristics; wrist-flip / turn numbers near ±180° can be off by
  one turn. Nothing checks them.

**Units and import (`core/mesh_units.py`, `core/step_datums.py`)**
- An inch STL larger than 20 units (a 30-inch part) was read as millimetres with "sure"
  confidence. Now 20-100 units is unsure (mm assumed, the dialog asks); a 150-inch part would still
  read as mm. **(fixed, tested)**
- STEP datums in assemblies ignore the sub-part's placement (documented in the module).

**Timing (`core/trajectory.py`, `core/cycle_time.py`)**
- A URDF joint with `velocity="0"` (common in hand-written URDFs) capped that joint's speed to 0
  and gave absurd cycle times. A limit of 0 now means "not given" (the program speed applies).
  **(fixed, tested)**
- Short moves with a jerk limit come out ~17 % slower than an ideal double-S (moving-average
  S-curve; stated in V10_PLAN).

**Paths (`core/paths.py`, `core/path_direction.py`, `core/coordinated.py`)**
- A directional TCP (v11 T7) is planned by the station planner as if its spin were free; the paths
  then cannot always reach the travel spin and fall back to flagged samples. Documented, not solved.
- Cross-hatch visits each point twice; `Seq.rows` keys rows by point id, so the second pass's row of a
  point overwrites the first's (`directional_sequence`): row changes can be misjudged on cross-hatch
  with a directional tool.
- Duplicate target points (two points at one position) give a zero travel direction: the tool's spin
  falls back to 0 (`spin_for`) and is not flagged.
- `core.coordinated._smooth` was O(n²) in path samples; now prefix sums + `searchsorted`, same
  results. **(fixed, tested)**
- Coordinated motion on parts served from both sides of a gantry still produced collision samples on
  gantry_overhead (v10 T16 status); the passes do not re-check the arm against the part beyond the
  samples' own collision check.

**Gantry placement and compare (`ui/frame_ui.py`, `ui/compare_axes.py`)**
- *Use this frame* while Results shows a compared setup applied the frame to the viewed state,
  lost on Back. Now refused with a message. **(fixed, tested)**
- Compare with an axis whose placement found no usable frame planned the unplaced example frame.
  Now the row says `no plan (not placed: ...)`. **(fixed)**

**Surface radius / edge check**
- `surface_radius` on a mesh with flipped face normals: the front-face filter (`fn · n > 0.2`)
  drops the samples and the point reads as flat. Silent.
- `edge_check` treats every target of a group as on one surface; targets on both sides of a thin
  sheet within 1.6 x spacing get no edge points between them (by design since v10 T10) but also no
  warning that the group spans two sides.

**Sessions and files**
- Two browser tabs on one server share one session; dialogs are per client but state is not, so
  two people editing get last-writer-wins on every field.
- A project copied to another PC with its part outside the project folder: `bundle` copies files at
  save time (v9 #12), but a project saved by an older version may still hold absolute paths;
  `tolerate_missing` drops the item with a warning instead of asking for the file.

## 5. Optimization methods: opportunities

Numbers are from `docs/V10_PLAN.md` Status and the v11 measurements.

| Where | Today | Opportunity | Expected gain | Cost |
|---|---|---|---|---|
| `core/traj_opt.py` clearance gradient | finite differences, 6 distance queries per waypoint per iteration (~2.5 ms each): +80 s on the CRX-30 path build | analytic gradient from Coal's nearest points + the link Jacobian (`pin.computeFrameJacobian`): 1 query per waypoint | build time with smoothing ~2-3x faster | medium: needs nearest-point output from `CollisionScene` |
| `opt/frame_place.rank_frames` | every frame builds its own `CarrierSites` candidates and coverage matrix (~0.6 s per frame, 31-39 s for 50 frames) | frames differ by a rigid motion: compute one coverage matrix in the frame's own coordinates and shift the targets instead | scoring 5-10x faster | medium |
| `opt/frame_place.refine_frame` | refines on the reach-map score; v10 T18 showed the refined frame planning *worse* (13.7 vs 11.7) | refine the top two on the quick-plan cost (no refinement, no zoning, ~3 s each) | better frames, +6-10 s | low |
| SimPanel builds | with *Optimize tool spin* every path is built twice (with and without) and the better kept: ~3x build time | build the planned-spin path once and run the spin DP only on sequences with problem samples or high motion | build time ~2x faster | low |
| `opt/placement.py` CP-SAT per k | each station count solved from scratch | hint the k+1 solve with the k solution (`AddHint`) | small on examples, large on 30+ stops | low |
| Exact checks | `check_point` tries poses in preference order and IK-solves each | the batch IK already runs in chunks; the collision checks are the bottleneck (88 % of plan time in v7): check branches of one pose in one `computeCollisions` call with all geometries placed once | 20-40 % on planning | medium |
| `core/coordinated._smooth` | O(n²) | prefix sums + `searchsorted` over path length | O(n log n) | trivial (done in task 12) |
| Test suite | slow tests plan whole examples | reduced scenes for the same assertions; `pytest-xdist` is not usable (memory), so fewer slow tests is the lever | fast suite < 10 min | medium |

## 6. Simpler implementations

Only where it is genuinely simpler. Where the code is already the simple version, that is said.

- **`core/patterns_surface.surface_distance`**: 20 lines of hand-written Dijkstra with `heapq` ->
  `scipy.sparse.csgraph.dijkstra(graph, indices=sources, min_only=True)` on a sparse matrix built from
  the neighbour lists: fewer lines, faster, and scipy is already used for `connected_components`
  in `core/facing.py`. **Done in task 12** (same distances as the heap version, tested).
- **`core/transforms.py`** (84 lines): `rpy_to_matrix`, `matrix_to_rpy`, `wxyz_to_matrix`,
  `matrix_to_wxyz` reimplement `scipy.spatial.transform.Rotation` (`from_euler("xyz")`,
  `as_euler("xyz")`, `from_quat` / `as_quat` with a reorder). ~10 lines instead of 60. **Do it, but**
  keep the gimbal-lock convention of `matrix_to_rpy` (all rotation into yaw) behind a test: saved
  project poses round-trip through it.
- **The three HTML bridges** -> one helper module (see 2). Removes ~40 duplicated lines of JS
  strings and makes the trick fixable in one place.
- **`core/coordinated._smooth`**: the O(n²) loop -> `np.cumsum` + `np.searchsorted` window means
  (same lines, no quadratic cost). **Done in task 12** (same results, tested).
- **Collision settings globals** -> one frozen dataclass passed to `CollisionScene` and pickled
  into `CheckerSpec`: fewer moving parts than five globals plus `geom_state()` /
  `set_geom_state()` plus the spec keys.
- **Result rows as dataclasses** (`CompareRow`, `FrameRow`): not fewer lines, but less state to
  remember; worth it only when those files are next changed.
- **Already fine, leave alone**: `core/trajectory.time_path` (the forward / backward pass is the
  textbook form; the jerk filter is one `np.convolve`); `core/path_direction` (small pure
  functions); `opt/set_cover.py` (CP-SAT is the right tool); `core/edge_check.py`; the pydantic schema.
- **Not simpler, do not do**: merging `sim_fields` / `sim_build` back into `simulate.py` only moves
  lines; replacing the Viterbi in `path_spin` with a generic graph library would add a dependency and
  a translation layer for no gain.
