"""*Locate missing files* (v12 #7): after a project opens without some of its files.

The part, carrier bodies and stay-out zones whose mesh is missing are kept in ``session.missing``
(ui.project_assets.tolerate_missing). This dialog has one upload per item: the uploaded file is
written into the project folder under the name the project refers to, the project is saved (it still
names the missing items, ``keep_missing``) and opened again, which finds the file there.
"""

from __future__ import annotations

from pathlib import Path

from ui import project_assets

LABEL = {"part": "part mesh", "carrier": "carrier body", "stayout": "stay-out zone"}


def items(session) -> list[str]:
    """One line per missing item: 'part mesh my_part.stl'."""
    return [f"{LABEL[m['kind']]} {Path(str(m['path']).replace(chr(92), '/')).name}" for m in session.missing]


def locate(panel, index: int, data: bytes, e=None) -> bool:
    """Put the file for ``session.missing[index]`` into the project, save, reopen."""
    s = panel.s
    project_assets.locate(s, index, data)
    panel.save_now()
    return panel._load(e, s.project_dir)


def ask(panel, e) -> None:
    """The dialog (needs a browser client; without one the warnings in the log stay)."""
    client = getattr(e, "client", None)
    s = panel.s
    if client is None or not s.missing:
        return
    with client.gui.add_modal("Locate missing files") as modal:
        client.gui.add_markdown(
            "_The project opened without these files. Give each one to restore the item "
            "(it is copied into the project folder), or close to go on without them._"
        )
        for i, line in enumerate(items(s)):
            name = Path(str(s.missing[i]["path"]).replace("\\", "/")).name
            up = client.gui.add_upload_button(f"Locate {line}", hint=f"the file {name}")

            def picked(ev, i=i, up=up) -> None:
                modal.close()
                locate(panel, i, up.value.content, ev)

            up.on_upload(picked)
        client.gui.add_button("Close").on_click(lambda _e: modal.close())
