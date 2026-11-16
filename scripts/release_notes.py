"""Fill packaging/RELEASE_NOTES.md.in from the files in a dist folder (no build needed).

    python scripts/release_notes.py <dist dir> [--version V] [--out FILE]

Needs the installer (``RobotCellOptimizer-*-setup.exe``) and ``SHA256SUMS.txt`` in the folder; the zip
is optional (its line is dropped when absent). Without ``--out`` the text goes to stdout. The
workflow appends GitHub's generated change list after the "Changes" heading.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TEMPLATE = ROOT / "packaging" / "RELEASE_NOTES.md.in"


def fill(dist: Path, version: str | None = None) -> str:
    installers = sorted(dist.glob("RobotCellOptimizer-*-setup.exe"))
    if not installers:
        raise SystemExit(f"release_notes: no RobotCellOptimizer-*-setup.exe in {dist}")
    sums = dist / "SHA256SUMS.txt"
    if not sums.is_file():
        raise SystemExit(f"release_notes: {sums} is missing")
    installer = installers[0].name
    zips = sorted(dist.glob("RobotCellOptimizer-*-win64.zip"))
    names = [installer] + ([zips[0].name] if zips else [])
    lines = [ln.strip() for ln in sums.read_text(encoding="utf-8").splitlines() if ln.strip()]
    shown = [ln for ln in lines if any(ln.endswith(n) for n in names)]
    if not any(ln.endswith(installer) for ln in shown):
        raise SystemExit(f"release_notes: {sums.name} has no line for {installer}")
    ver = version or installer.removeprefix("RobotCellOptimizer-").removesuffix("-setup.exe")
    text = TEMPLATE.read_text(encoding="utf-8")
    if not zips:  # drop the zip bullet
        text = "".join(ln for ln in text.splitlines(True) if "{{ZIP}}" not in ln)
    return (
        text.replace("{{VERSION}}", ver)
        .replace("{{INSTALLER}}", installer)
        .replace("{{ZIP}}", zips[0].name if zips else "")
        .replace("{{SHA256}}", "\n".join(shown))
    )


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dist", type=Path)
    ap.add_argument("--version", default=None)
    ap.add_argument("--out", type=Path, default=None)
    a = ap.parse_args()
    text = fill(a.dist, a.version)
    if a.out:
        a.out.write_text(text, encoding="utf-8")
    else:
        sys.stdout.write(text)


if __name__ == "__main__":
    main()
