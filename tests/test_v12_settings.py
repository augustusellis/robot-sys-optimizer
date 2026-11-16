"""v12 item 11: saved panel settings are checked against the live app (ui.settings).

Done when every ``SAVED`` key resolves to a widget of the built app, no key is listed twice (the
dict literal and its tuples are read from the source, where a duplicate would otherwise vanish), and
a renamed key (``RENAMED``) still restores the user's value.
"""

import ast
from pathlib import Path

import pytest
import viser

from ui import settings

SRC = Path(settings.__file__)


def _saved_literal() -> ast.Dict:
    tree = ast.parse(SRC.read_text(encoding="utf-8"))
    for node in tree.body:
        if isinstance(node, ast.AnnAssign) and getattr(node.target, "id", "") == "SAVED":
            return node.value
    raise AssertionError("SAVED not found")


def test_no_duplicate_keys_in_the_source():
    d = _saved_literal()
    panels = [k.value for k in d.keys]
    assert len(panels) == len(set(panels)), "a panel listed twice: the first entry is dropped"
    for k, v in zip(panels, d.values, strict=True):
        attrs = [e.value for e in v.elts]
        dup = {a for a in attrs if attrs.count(a) > 1}
        assert not dup, f"{k}: {dup} listed twice"


@pytest.fixture(scope="module")
def app():
    from app import build_app

    server = viser.ViserServer(host="127.0.0.1", port=8984, verbose=False)
    st = build_app(server)
    yield st
    st["builder_window"].stop()
    server.stop()


def test_every_saved_key_resolves_on_the_app(app):
    assert settings.missing(app) == []


def test_renamed_keys_restore(app, monkeypatch):
    h = app["stations"].max_stations
    monkeypatch.setitem(settings.RENAMED, "stations.old_max", "stations.max_stations")
    old = h.value
    try:
        done = settings.apply(app, {"stations.old_max": 7})
        assert "stations.max_stations" in done and h.value == 7
        settings.apply(app, {"stations.old_max": 3, "stations.max_stations": 5})
        assert h.value == 5  # the new key wins
    finally:
        h.value = old
