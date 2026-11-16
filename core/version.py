"""The version string shown to users (Settings panel, window label, ``--version``, selftest).

Frozen build: from the ``_version.py`` that scripts/build_exe.py writes (VERSION = git describe,
COMMIT, DATE), e.g. ``v0.9.0 (16de6e1, 2026-11-15)``. Source checkout: ``git describe --tags
--always --dirty``, or ``source (unknown)`` without git. Never raises; cached.
"""

from __future__ import annotations

import subprocess
import sys
from functools import lru_cache

from core.provenance import ROOT, frozen_version

UNKNOWN_SOURCE = "source (unknown)"


def numeric_version(text: str) -> str:
    """``v1.2.3-4-gabc`` -> ``1.2.3.4`` for Windows file version info (``0.0.0.0`` if no number)."""
    import re

    m = re.match(r"v?(\d+)(?:\.(\d+))?(?:\.(\d+))?(?:-(\d+)-g)?", text.strip())
    if not m:
        return "0.0.0.0"
    return ".".join(g or "0" for g in m.groups())


@lru_cache(maxsize=1)
def app_version() -> str:
    try:
        if getattr(sys, "frozen", False):
            info = frozen_version()
            if not info.get("VERSION"):
                return "unknown"
            extra = [x for x in (info.get("COMMIT"), (info.get("DATE") or "")[:10]) if x]
            return info["VERSION"] + (f" ({', '.join(extra)})" if extra else "")
        out = subprocess.run(
            ["git", "describe", "--tags", "--always", "--dirty"],
            cwd=ROOT,
            capture_output=True,
            text=True,
            timeout=5,
        ).stdout.strip()
        return out or UNKNOWN_SOURCE
    except Exception:
        return UNKNOWN_SOURCE
