"""The hidden-input bridge from HTML widgets back to Python (v12 #13).

Viser has no joystick, hold-to-repeat button or tab bar, so those are HTML (``gui.add_html``) with
inline event handlers (inline handlers run; a ``<script>`` in innerHTML would not). They talk to
the server through a viser text input: the handler finds the input whose value starts with the
widget's ``prefix`` (``joy:``, ``jog:``, ``tab:``), writes a new value with the native value setter
and fires an ``input`` event, which viser sends like typing; the input's ``on_update`` parses it.
The input's row is hidden by the widget itself: an ``<img onerror>`` runs when the HTML is drawn.

Used by ui.joystick, ui.jog_arrows and ui.package_tabs: a change to the trick (say a viser
upgrade renames the input's wrapper class) is made here once.
"""

from __future__ import annotations

import html

SETTER = "var set=Object.getOwnPropertyDescriptor(HTMLInputElement.prototype,'value').set;"


def find_js(prefix: str) -> str:
    """JS that sets ``inp`` to the bridge input of ``prefix`` (undefined when there is none)."""
    return (
        "var inp=[...document.querySelectorAll('input')]"
        f".find(function(i){{return i.value.indexOf('{prefix}')==0}});"
    )


def send_js(value_expr: str) -> str:
    """JS that writes the JS expression ``value_expr`` into ``inp`` and tells React (needs ``set``)."""
    return f"set.call(inp,{value_expr});inp.dispatchEvent(new Event('input',{{bubbles:true}}))"


def hide_js(prefix: str) -> str:
    """JS (an IIFE) that hides the bridge input's row."""
    return (
        "(function(){" + find_js(prefix) + "if(!inp)return;var r=inp.closest('.mantine-InputWrapper-root')"
        "||inp.parentElement&&inp.parentElement.parentElement;if(r)r.style.display='none'})()"
    )


def hide_tag(prefix: str) -> str:
    """The ``<img onerror>`` that runs ``hide_js`` when the widget's HTML is drawn."""
    return f'<img src="x:" style="display:none" onerror="{html.escape(hide_js(prefix))}">'
