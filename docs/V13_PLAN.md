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

## Phase B: feedback round

Items to be added from the user's feedback.

## Status

| # | Status | Measurements / deviations |
|---|--------|---------------------------|
| 1 | done | Over the 331 candidate frames of each gantry example: bridge-only sweep 0.8 s, with the carriage 2.7-2.8 s (+2 s of a scoring run of 25-65 s); same verdicts as before (gantry_overhead 331 ok; floor_xy_rails 281 ok, 50 bridge through the part). Not tried in a browser. |
| 2 | done, partly | App start 13.3 -> 5.4 s (1024 px textures), 2.3 s in the tests (128 px); loading an example 6.3 -> 2.0 s (0.6 s in the tests). Full fast suite (4 files per process): **596 passed, 0 failed, 805 s** (v12: 2102 s; 1171 s with 1024 px textures). The < 10 min target is still not reached: the slowest batches are now the GUI tests that plan (`test_app.py`, `test_work_state.py`, ...). Textures at 1024 px not yet looked at in a browser. |
