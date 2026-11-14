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
