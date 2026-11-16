"""v12 item 23: the layering rule (CLAUDE.md "Architecture rules"): ``core/`` and ``opt/`` never
import viser or anything in ``ui/``, so the math runs headless and in worker processes.

Checked two ways: statically (every import statement, including ones inside functions, read with
ast), and at run time (importing every core / opt module in a fresh interpreter loads neither).
"""

import ast
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
FORBIDDEN = ("viser", "ui", "app")


def _modules():
    for pkg in ("core", "opt"):
        yield from sorted((REPO / pkg).rglob("*.py"))


def test_no_gui_imports_in_the_source():
    bad = []
    for p in _modules():
        tree = ast.parse(p.read_text(encoding="utf-8"), str(p))
        for node in ast.walk(tree):
            names = []
            if isinstance(node, ast.Import):
                names = [a.name for a in node.names]
            elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
                names = [node.module]
            for n in names:
                if n.split(".")[0] in FORBIDDEN:
                    bad.append(f"{p.relative_to(REPO)}:{node.lineno} imports {n}")
    assert not bad, "\n".join(bad)


def test_importing_core_and_opt_loads_no_gui():
    mods = [".".join(p.relative_to(REPO).with_suffix("").parts) for p in _modules()]
    mods = [m.removesuffix(".__init__") for m in mods]
    code = (
        "import importlib, sys\n"
        f"for m in {mods!r}:\n"
        "    importlib.import_module(m)\n"
        "bad = sorted(m for m in sys.modules if m.split('.')[0] in ('viser', 'ui', 'app'))\n"
        "print(','.join(bad))\n"
    )
    out = subprocess.run([sys.executable, "-c", code], cwd=REPO, capture_output=True, text=True, timeout=300)
    assert out.returncode == 0, out.stderr[-2000:]
    assert out.stdout.strip() == "", f"loaded: {out.stdout.strip()}"
