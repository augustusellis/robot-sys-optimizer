"""Package builder tab bar (v10 #8): an HTML bar that shows which package type is open.

Viser's button group has no "selected" look, so the bar is HTML (``gui.add_html``) redrawn by the
server with the active tab highlighted. A click goes back to Python through the hidden-input bridge
(ui.html_bridge): the button's inline ``onclick`` writes ``tab:<kind>,<n>`` into a text input that the
bar hides itself.
"""

from __future__ import annotations

import html
import re
from collections.abc import Callable

from ui.html_bridge import SETTER, find_js, hide_tag, send_js

PREFIX = "tab:"
_VALUE = re.compile(r"^tab:([a-z]+)(?:,\d+)?$")


def _click_js(kind: str) -> str:
    return (
        "(function(){"
        + find_js(PREFIX)
        + "if(!inp)return;"
        + SETTER
        + send_js(f"'tab:{kind},'+Date.now()")
        + "})()"
    )


def bar_html(tabs: dict[str, str], active: str) -> str:
    """The bar: one button per (label -> kind); ``active`` (a kind) drawn as the selected tab."""
    out = [
        '<div style="display:flex;flex-wrap:wrap;gap:2px;border-bottom:2px solid #3b82f6;margin:2px 0 6px">'
    ]
    for label, kind in tabs.items():
        on = kind == active
        style = (
            "padding:5px 9px;border:1px solid rgba(128,128,128,.45);border-bottom:none;"
            "border-radius:6px 6px 0 0;cursor:pointer;font-size:13px;"
            + (
                "background:#3b82f6;color:#fff;font-weight:600"
                if on
                else "background:transparent;color:inherit"
            )
        )
        out.append(
            f'<button type="button" style="{style}" onclick="{html.escape(_click_js(kind))}">'
            f"{html.escape(label)}</button>"
        )
    out.append("</div>" + hide_tag(PREFIX))
    return "".join(out)


class TabBar:
    """The bar + its bridge input; ``on_pick(kind)`` when a tab is clicked."""

    def __init__(self, gui, tabs: dict[str, str], on_pick: Callable[[str], None], active: str = "tool"):
        self.tabs, self.on_pick = tabs, on_pick
        self.active = active
        self.html = gui.add_html(bar_html(tabs, active))
        self.input = gui.add_text("Package tab", PREFIX + active, hint="set by the tab bar above")
        self.input.on_update(lambda _e: self.command(self.input.value))

    def command(self, text: str) -> None:
        m = _VALUE.match(text.strip())
        if m is None or m.group(1) not in self.tabs.values():
            return
        if m.group(1) != self.active:
            self.on_pick(m.group(1))

    def show(self, kind: str) -> None:
        """Highlight ``kind`` (called when the package type changes, by a click or in code)."""
        if kind != self.active:
            self.active = kind
            self.html.content = bar_html(self.tabs, kind)
