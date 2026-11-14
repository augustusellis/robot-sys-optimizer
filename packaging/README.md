# Windows executable (PyInstaller, onedir)

Files: `rso.spec` (PyInstaller spec), `launch.py` (entry point), `RobotCellOptimizer.iss` (Inno Setup),
`../scripts/build_exe.py` (the build). Nothing is downloaded at install or run time.

## Build locally

```
PY=<path to the rso env>/python.exe
$PY scripts/build_exe.py --zip                   # build + selftest + dist/RobotCellOptimizer-<version>-win64.zip
$PY scripts/build_exe.py --maps --zip --installer  # also prebuild reachability maps; installer needs Inno Setup (iscc)
```

Options: `--skip-selftest`, `--no-cad` (without CadQuery, about 500 MB smaller, no STEP import/export),
`--any-env` (skip the check that Python is the `rso` env). The script must run in the `rso` env (the same
conda env the tests use, so the native libraries are the tested ones). It sets `PYTHONNOUSERSITE=1` (packages
in the user site-packages must not leak into the bundle) and puts `<env>\Library\bin` on PATH. Output:
`dist/RobotCellOptimizer/RobotCellOptimizer.exe`, `dist/SHA256SUMS.txt`, the zip and/or setup exe.
The git version goes in a generated `_version.py` (the exe has no git); `core.provenance` reads it.

## Selftest

`RobotCellOptimizer.exe --selftest` runs headless and exits 0 or 1. The build runs it with `RSO_DATA` set to a
temp folder and `RSO_OFFLINE_GUARD=1`, and fails when it fails. It checks: the Viser web client is served
(loopback HTTP, proxy ignored); the example `mobile_amr` loads; a quick plan runs in the worker process
(spawn workers and the check pool work frozen); the HTML report is written; STEP export works (or reports "not
included in this build"); numba compiles the CRX offset-wrist IK from the bundle.

## Offline guard

`RSO_OFFLINE_GUARD=1` makes any non-loopback socket connect or DNS lookup in Python (main process and spawn
workers) raise and print `[offline-guard] blocked ...` on stderr. Loopback (127.x, ::1, localhost) is allowed.
It does not cover a browser or native code that opens its own sockets, so CI also uses a firewall rule.
Other variables: `--no-browser` (do not open a browser), `RSO_SELFTEST_PORT` (selftest server port, default
free port), `RSO_DATA` (data folder; default `%LOCALAPPDATA%\RobotCellOptimizer`).

## Installer

Per-user (`PrivilegesRequired=lowest`), into `%LOCALAPPDATA%\Programs\RobotCellOptimizer`, Start-menu entry,
uninstaller. The data folder `%LOCALAPPDATA%\RobotCellOptimizer` (projects, caches, tools) is NOT removed on
uninstall. Unsigned: Windows SmartScreen shows a warning once.

## Why onedir

D0 spike numbers (CadQuery included): 1,011 MB on disk, 2,135 files, 321 MB zipped; CadQuery is about 495 MB
of that (mostly VTK). Server answers in 1.1 s, GUI ready in about 11 s (same as from source). Selftest 55 s cold
(reachability map and numba compile built), 18.5 s warm. One-file mode was rejected: it unpacks for about 5 s
on every start, leaves about 1 GB in `%TEMP%` when killed (and managed PCs often block running from `%TEMP%`),
and recompiles numba every start. numba needs `module_collection_mode={"core.ik.offset_wrist_nb": "py"}` so its
cache uses `NUMBA_CACHE_DIR` (set by `core.data_paths.configure_numba_cache`).

## Releases from CI (`.github/workflows/release.yml`)

Cut a release:
1. Merge to the branch you want to ship, then `git tag vX.Y.Z` and `git push origin vX.Y.Z`. (Or Actions ->
   release -> Run workflow; its `draft` input is ticked by default.)
2. The `windows` job builds with `scripts/build_exe.py --maps --zip --installer` (prebuilt reachability maps,
   zip, Inno Setup installer, SHA256SUMS.txt). The version comes from `git describe`, so the tag is the version.
3. The `release` job creates a **draft** GitHub release `vX.Y.Z` with the zip, the setup exe and SHA256SUMS.txt
   (it refuses files over GitHub's 2 GB per-file limit; use `--no-cad` if that ever happens).
4. Open the draft on GitHub, check the generated notes and the files, then **Publish**. Nothing is published
   automatically. A manual run on a branch makes a prerelease draft named `manual-<run number>`.

What CI checks before the draft exists (any failure stops the release):
- the build's own `--selftest` (headless plan, report, STEP, numba, bundled web client);
- with outbound network blocked by a Windows firewall rule and `RSO_OFFLINE_GUARD=1`: `--selftest` again, then
  the exe is started with `--no-browser --port 8765`, `packaging/ci_serve_check.ps1` waits for the "Robot Cell
  Optimizer ready:" line, loads the page in headless Edge and checks that the Viser client rendered (root
  populated and a canvas); the rule is removed in an `always()` step. A hidden download fails the build, not a customer;
- the installer is installed silently per user (`/VERYSILENT /CURRENTUSER /SUPPRESSMSGBOXES`), the installed
  exe's selftest runs, and it is uninstalled silently.

Run the serve check locally (not the firewall part, which needs admin) with
`powershell -File packaging\ci_serve_check.ps1 -Exe dist\RobotCellOptimizer\RobotCellOptimizer.exe`
(about 3 min; Edge in swiftshader mode is slow).

Linux AppImage: not built yet (decision: after the first Windows release); a commented placeholder job is in the workflow.

### SmartScreen (unsigned builds)

The exe and installer are not code-signed. On first run Windows SmartScreen may show "Windows protected your
PC": click **More info**, then **Run anyway**. Managed PCs may block unsigned programs: IT has to allow the
file (by the SHA-256 in SHA256SUMS.txt, or by path), or the build has to be signed later. The zip variant needs
no install and no admin rights; unblock it first (file Properties -> Unblock) if Windows marks the download.
