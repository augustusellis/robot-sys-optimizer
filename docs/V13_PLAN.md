# V13 plan (branch `v13`, from `v12`)

Started 2026-11-17. The user asked for (4) the parts of v12 left open and (5) a new feedback round.

## Phase A: v12 items left open

1. **Gantry frame check sweeps the carriage (v12 #26 leftover).**
   - *Found:* `opt.frame_place.sweep_clear` only moved the bridge (bodies on the X axis) along X.
     The carriage and lift stage ride along with the bridge; a carriage that hangs low enough to
     hit the part at every Y stops the X travel, but the frame was still scored.
   - *Fix:* after the bridge sweep, at every X sample (0.2 m) some Y sample (0.2 m) and lift end
     must leave the carriage + lift bodies clear of the part (carrier part standoff). The carriage
     is not required to clear the part everywhere: stops it cannot take are rejected one by one by
     the stop checks, and moves between stops are checked when the paths are built. The search
     starts at the last clear (Y, lift), so the first try usually succeeds. Reason shown: *the
     carriage cannot pass the part at x ... mm*.
   - *Done when:* a gantry whose drop hits a wall across the whole Y travel is refused; one that
     can go around the wall, or passes over it, is not; the shipped examples rank as before.
     Tests: `tests/test_v13_sweep.py`.

2. **Test suite under 10 minutes (v12 #22 leftover).**
   - *Found (profile of `build_app`):* 13.3 s per app start, of which ~10 s was PIL PNG-encoding:
     viser sends each URDF mesh as a GLB and trimesh re-encodes its texture every time; every
     UR20 mesh carries a 2048 x 2048 baked atlas (logo, shading), 20 meshes x 2 robot views
     (Cell, package builder). Loading an example took 6.3 s for the same reason (the robot view
     is rebuilt). This also slowed the real app at every start and robot change.
   - *Fix:* `ui.robot_view.light_urdf` loads the URDF itself and downscales textures above
     `TEXTURE_PX` (1024; env `RSO_TEXTURE_PX`, the tests set 128), the small copies cached per
     file. Turning textures into vertex colours was tried and dropped: 35 % of the UR20 faces
     straddle atlas swatches (the logo and shading would be lost).
   - The shared-app session fixture is **not** done: with an app start of ~2 s in the tests it
     would save ~2 s per test file and needs every one of the 92 app fixtures rewritten.

## Phase B: batch station optimization over many parts (user, 2026-11-17)

Goal: run the station optimizer automatically on several parts with the same end effector. For
each part the user gets a PDF report and a solved result they can open and view. Parts are made in
the package builder's **Part** tab, with their pose in the world frame and their targets, and are
saved to a folder the user chooses. The optimizer can then run on several parts from that folder.

User decisions (2026-11-17):
- **Targets:** reuse the Targets tab. The Part tab sets the mesh and the world pose; *Edit targets*
  loads the part into the Cell so the normal Targets tools apply; *Save part package* writes it.
  No second set of target tools.
- **Per part:** only the mesh, the world pose and the targets (groups with tolerances + points).
  The robot, end effector, external axis, stay-outs, optimizer settings and weights all come from
  the open project.
- **Stages per part:** plan the stations, build the paths (cycle time), save the solved project and
  print the PDF report. No robot programs.
- **Tool:** the project's end effector, fixed (no tool-geometry search).
- **External axis:** stops are planned per part. The rail / gantry frame stays where the project
  has it; a checkbox *Search the axis frame per part* (off by default) runs the frame search
  (opt.frame_place) for each part first.
- **Package format:** one folder per part, `<folder>/<part name>/part.json` plus the mesh.
- **Selection:** the chosen folder is scanned and every part package is listed with a checkbox,
  all ticked by default.
- **Running:** parts run in parallel, dropping to one at a time when cores (or memory) are short.
  Progress per part, Cancel, and a part that fails is logged and skipped. There is also a
  command-line script for runs without the GUI.
- **Results:** an output folder with one subfolder per part (a full project with its results +
  `report.pdf`), a summary of all parts, and a **Batch results** viewer in the Results tab to switch
  between parts.

### Tasks

3. **Part package format (`core/part_package.py`).**
   - `PartPackage` (pydantic, `extra="forbid"`): `name`, `mesh` (file name in the folder),
     `units`, `pose` (world, `schema.Pose`), `collision_keep`, `targets` (`schema.Targets`, part
     frame), `datums` (STEP coordinate systems, kept from the builder), `version`.
   - `save_part_package(pkg, mesh_src, folder) -> Path` writes `<folder>/<safe name>/part.json` +
     the mesh. It refuses to overwrite another part's folder unless asked. `load_part_package(dir)`
     validates and checks that the mesh exists. `scan_folder(folder)` returns the packages found
     (one level deep) and lists unreadable ones with their reason, without failing.
   - `apply_to_project(project, pkg, pkg_dir) -> Project` gives a copy of the project with this
     part, pose and targets. Results and work state are cleared; robot, tool, base, stay-outs and
     optimizer are kept.
   - *Done when:* round trip save -> load -> apply gives the same part, pose and targets; bad
     JSON / missing mesh / unknown keys give a clear reason. Tests: `tests/test_v13_part_package.py`.

4. **Package builder Part tab: world pose, targets, save to a folder.**
   - The Part tab gets *Position in world* (mm / deg PoseWidgets + gizmo; four decimals as in v12
     #28) and a *Parts folder* field with a folder picker (remembered in the settings).
   - *Edit targets* puts the part (mesh + world pose) into the Cell as the project's part and opens
     the Targets tab; a banner says that a part package is being edited and links back to the
     builder. *Save part package* takes the project's current targets and writes the package with
     task 3. *Open part package* loads one back for editing (part, pose, targets).
   - Saving with no targets, or a name that is already in the folder, asks first.
   - *Done when:* a part made in the builder, given targets in the Targets tab and saved, reloads
     with the same pose and targets; the user guide's package builder chapter describes it. Tests
     drive the panel in-process (`tests/test_v13_part_tab.py`).

5. **Batch runner core (`opt/batch.py`, no viser).**
   - `run_part(project_json, pkg_dir, out_dir, options, emit, cancelled)`, one part per call:
     apply the package -> (optional) frame search -> plan the stations -> build the paths of the
     best plan -> save `out_dir/<part>/project.json` (+ the part mesh as an asset, so the folder
     opens alone) -> write `report.html` and print `report.pdf` (core.report). It returns a row:
     points, reached / unreached, stops, cost, cycle time, run time, status and reason.
   - `run_batch(...)` runs `run_part` over the ticked parts with a process pool. Workers =
     `min(parts, cores // checks_per_part, free_memory // memory_per_part)`, and 1 when that is
     below 2. The memory per part is measured on the shipped examples and written in Status. Each
     part's planner keeps its own parallel checks only when running alone (no nested pools).
   - A part that raises or times out is logged with its reason; the others go on. Cancel stops
     the parts not started and ends the running ones.
   - `out_dir/summary.csv` and `out_dir/summary.pdf` (one table: part, reached, stops, cycle
     time, cost, status, link to each report) are written after every part, so a cancelled run
     still has a summary of what finished.
   - The GUI job and the CLI share this code. *Done when:* two golden parts on the rail example
     give two folders that each open as a project with results, two PDFs and a summary; one
     broken package is reported and skipped; serial and parallel runs give the same plans.
     Tests: `tests/test_v13_batch.py` (PDF printing mocked when no Edge is found).

6. **Optimize tab: *Run on several parts*.**
   - A section under the station planner: *Parts folder* (picker) -> *Scan* -> a checklist of the
     packages found (all ticked; unreadable ones shown greyed with their reason), *Output folder*,
     *Search the axis frame per part* checkbox, *Run batch* / *Cancel*.
   - Runs as a background job (ui.jobs worker), with a progress line per part (queued / running
     x % / done / failed: reason) and the planned worker count shown with why ("4 parts at once:
     16 cores, 37 GB free").
   - Disabled-button reasons as elsewhere (no robot, no end effector, no folder, nothing ticked).
   - *Done when:* the job runs from the panel in-process in the tests and the progress reaches
     every part; Cancel leaves a summary of the finished parts.

7. **Results tab: *Batch results* viewer.**
   - *Open batch* (picker) reads an output folder's summary. It shows the summary table, and a
     part dropdown loads that part's solved project into the session (as Project -> Open does) so
     the plans, paths and simulation can be viewed. *Open report* opens the part's PDF (or HTML).
   - The open batch is remembered in the work state, so the viewer comes back after a restart.
   - Opening a part's project from the batch replaces the open project, so it asks first if the
     project has unsaved changes.
   - *Done when:* after a batch, switching parts in the viewer shows each part's plans in the
     Cell. Tests: `tests/test_v13_batch_view.py`.

8. **Command-line script `scripts/batch_run.py`.**
   - `python scripts/batch_run.py <project folder> <parts folder> <output folder> [--parts a,b]
     [--frame-search] [--workers N]`; prints one line per part and the summary path; exit code 1
     when a part failed. Same code as task 5.
   - *Done when:* it runs the task-5 test case from a subprocess.

9. **Docs, guide and examples.**
   - User guide: a section in the package builder chapter (making part packages) and one in the
     Optimize chapter (batch runs, output folder layout, the viewer, the CLI); chapter 10 lists
     the new modules. `docs/HISTORY.md` v13 notes.
   - An example parts folder `examples/batch_parts/` with three golden-part packages (flat plate,
     curved panel, open box) placed for the rail example, used by the tests and as a demo.
   - Full fast suite at the end of the round; measurements in Status.

Not in this round (user's choices): per-part stay-outs or optimizer overrides, the tool-geometry
optimizer in batches, and robot programs per part.

## Status

| # | Status | Measurements / deviations |
|---|--------|---------------------------|
| 1 | done | Over the 331 candidate frames of each gantry example: bridge-only sweep 0.8 s, with the carriage 2.7-2.8 s (+2 s of a scoring run of 25-65 s); same verdicts as before (gantry_overhead 331 ok; floor_xy_rails 281 ok, 50 bridge through the part). Not tried in a browser. |
| 2 | done, partly | App start 13.3 -> 5.4 s (1024 px textures), 2.3 s in the tests (128 px); loading an example 6.3 -> 2.0 s (0.6 s in the tests). Full fast suite (4 files per process): **596 passed, 0 failed, 805 s** (v12: 2102 s; 1171 s with 1024 px textures). The < 10 min target is still not reached: the slowest batches are now the GUI tests that plan (`test_app.py`, `test_work_state.py`, ...). Textures at 1024 px not yet looked at in a browser. |
| 3 | planned | |
| 4 | planned | |
| 5 | planned | |
| 6 | planned | |
| 7 | planned | |
| 8 | planned | |
| 9 | planned | |
