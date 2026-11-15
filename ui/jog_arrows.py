"""Jog arrows (v11 #6): an HTML pad of ◀ ▶ arrow buttons per axis that jog on a click and keep
jogging while held (ui.robot_jog.JogPanel).

Viser buttons only report clicks, so the arrows are HTML (``gui.add_html``) with an inline
``onpointerdown``: it sends one step at once and, when the button is still held after ``HOLD_MS``,
one step every ``REPEAT_MS`` until the pointer is released or leaves the window. A step travels to
Python through the hidden-input bridge of ui.joystick / ui.package_tabs: ``jog:<axis>,<sign>,<n>``
written into a text input (hidden by the pad itself). Steps arriving while one is being applied
collapse into the newest (ui.widgets.Latest), so a slow IK or collision check never queues up.
"""

from __future__ import annotations

import html
import re

HOLD_MS, REPEAT_MS = 350, 120
MOVE, TURN = ("X", "Y", "Z"), ("Rx", "Ry", "Rz")
_VALUE = re.compile(r"^jog:([A-Za-z0-9]+),(-?1),[\d.]+$")
_FIND = (
    "var inp=[...document.querySelectorAll('input')].find(function(i){return i.value.indexOf('jog:')==0});"
)
_HIDE = (
    "(function(){" + _FIND + "if(!inp)return;var r=inp.closest('.mantine-InputWrapper-root')"
    "||inp.parentElement&&inp.parentElement.parentElement;if(r)r.style.display='none'})()"
)


def _down_js(axis: str, sign: int) -> str:
    return (
        "(function(ev){" + _FIND + "if(!inp)return;"
        "var set=Object.getOwnPropertyDescriptor(HTMLInputElement.prototype,'value').set;var n=0;"
        f"function send(){{set.call(inp,'jog:{axis},{sign},'+Date.now()+'.'+(n++));"
        "inp.dispatchEvent(new Event('input',{bubbles:true}))}"
        f"send();var t=null;var d=setTimeout(function(){{t=setInterval(send,{REPEAT_MS})}},{HOLD_MS});"
        "function up(){clearTimeout(d);if(t)clearInterval(t);"
        "window.removeEventListener('pointerup',up);window.removeEventListener('pointercancel',up);"
        "window.removeEventListener('blur',up)}"
        "window.addEventListener('pointerup',up);window.addEventListener('pointercancel',up);"
        "window.addEventListener('blur',up);ev.preventDefault()})(event)"
    )


def _btn(axis: str, sign: int, text: str) -> str:
    style = (
        "min-width:34px;padding:4px 6px;border:1px solid rgba(128,128,128,.5);border-radius:6px;"
        "background:transparent;color:inherit;cursor:pointer;font-size:14px;touch-action:none;user-select:none"
    )
    return (
        f'<button type="button" title="{axis} {"+" if sign > 0 else "−"} (hold to keep moving)" '
        f'style="{style}" onpointerdown="{html.escape(_down_js(axis, sign))}">{text}</button>'
    )


def pad_html(joints: int = 6, cartesian: bool = True) -> str:
    """The pad: Move X Y Z and Turn Rx Ry Rz (``cartesian``: not in the Joint frame) and J1..Jn,
    each with a − and a + arrow."""

    def row(label: str, axes) -> str:
        cells = "".join(
            f'<span style="display:inline-flex;align-items:center;gap:2px;margin:2px 8px 2px 0">'
            f'{_btn(a, -1, "◀")}<b style="min-width:22px;text-align:center">{a}</b>{_btn(a, 1, "▶")}</span>'
            for a in axes
        )
        return f'<div style="margin:2px 0"><small>{label}</small><br>{cells}</div>'

    return (
        '<div class="rso-jog">'
        + (row("Move (the frame's axes)", MOVE) + row("Turn", TURN) if cartesian else "")
        + row("Joints", [f"J{k + 1}" for k in range(joints)])
        + f'</div><img src="x:" style="display:none" onerror="{html.escape(_HIDE)}">'
    )


def parse(text: str) -> tuple[str, int] | None:
    """('X', -1) from 'jog:X,-1,<n>' (None for anything else)."""
    m = _VALUE.match(text.strip())
    return (m.group(1), int(m.group(2))) if m else None


class JogArrows:
    """The pad + its bridge input; ``jog(axis, sign)`` per step."""

    def __init__(self, gui, jog, joints: int = 6):
        from ui.widgets import Latest

        self.jog = jog
        self._shown = (joints, True)
        self.html = gui.add_html(pad_html(joints))
        self.input = gui.add_text("Jog arrows", "jog:", hint="set by the arrows above")
        self._latest = Latest(self.command)
        self.input.on_update(lambda _e: self._latest(self.input.value))

    def show(self, joints: int, cartesian: bool) -> None:
        """Redraw for another robot (joint count) or frame (Joint: joint arrows only)."""
        if (joints, cartesian) != self._shown:
            self._shown = (joints, cartesian)
            self.html.content = pad_html(joints, cartesian)

    def command(self, text: str) -> bool:
        cmd = parse(text)
        if cmd is None:
            return False
        self.jog(*cmd)
        return True
