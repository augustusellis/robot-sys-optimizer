"""Folders of the batch runs (v13): the parts folder, the output folder, the last batch viewed.

They belong to the user rather than to one project (one parts folder serves many projects), so
they live in ``<workspace>/batch_settings.json`` next to ``last_project.txt``. Reading never fails:
a missing or broken file gives the defaults.
"""

from __future__ import annotations

import json

from ui.session import WORKSPACE

FILE = WORKSPACE / "batch_settings.json"


def _all() -> dict:
    try:
        data = json.loads(FILE.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}
    return data if isinstance(data, dict) else {}


def get(key: str, default: str = "") -> str:
    v = _all().get(key, default)
    return v if isinstance(v, str) else default


def put(key: str, value: str) -> None:
    data = _all()
    data[key] = str(value)
    try:
        FILE.parent.mkdir(parents=True, exist_ok=True)
        FILE.write_text(json.dumps(data, indent=2), encoding="utf-8")
    except OSError:
        pass
