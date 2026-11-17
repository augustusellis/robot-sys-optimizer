"""Results -> *Batch results* (v13 #7): look through the parts of a batch run.

*Open batch* reads the summary of an output folder (``ui.batch_summary``) and shows its table. The
part dropdown and *Open this part* load that part's solved project (as Project -> Open does), so
its plans can be viewed in the Cell, inspected and simulated; with *Build paths when opened* its
paths are built straight away (paths are not saved in a project). *Download report* sends the
part's PDF (or HTML) report. The last batch is remembered for the next start (``ui.batch_prefs``:
opening a part replaces the project, so the project's own state could not keep it).
"""

from __future__ import annotations

from pathlib import Path

import viser

from core import part_package as pp
from ui import batch_prefs, batch_summary
from ui.widgets import notify

NONE = "(none)"
ICON = {"done": "✔", "failed": "✘", "cancelled": "–"}


def table(data: dict) -> str:
    rows = data.get("rows", [])
    done = sum(r.get("status") == "done" for r in rows)
    lines = [
        f"**{done} of {len(rows)} parts planned** ({data.get('updated', '')}).",
        "",
        "| Part | | Reached | Stops | Cycle (s) | Cost |",
        "|---|---|---|---|---|---|",
    ]
    for r in rows:
        reached = f"{r['reached']}/{r['points']}" if r.get("points") is not None else "—"
        cyc = r.get("cycle_s")
        lines.append(
            f"| {r.get('part', '?')} | {ICON.get(r.get('status'), '?')} | {reached} | "
            f"{r.get('stops', '—')} | {'—' if cyc is None else cyc} | {r.get('cost', '—')} |"
        )
    failed = [r for r in rows if r.get("status") == "failed"]
    if failed:
        lines.append("")
        lines += [f"✘ **{r.get('part')}**: {r.get('reason', '')}  " for r in failed]
    return "\n".join(lines)


class BatchView:
    def __init__(self, server: viser.ViserServer, session, project_panel, build_paths=None):
        self.s, self.project, self.build_paths = session, project_panel, build_paths
        self.data: dict | None = None
        self.out: Path | None = None
        gui = server.gui
        self.folder = gui.add_folder("Batch results", expand_by_default=False)
        with self.folder:
            self.path = gui.add_text("Batch output folder", batch_prefs.get("last_batch"))
            self.open_btn = gui.add_button("Open batch")
            self.summary = gui.add_markdown("")
            self.part = gui.add_dropdown("Part", (NONE,))
            self.auto_paths = gui.add_checkbox("Build paths when opened", True)
            self.open_part_btn = gui.add_button("Open this part", color="green")
            self.report_btn = gui.add_button("Download report")
            self.info = gui.add_markdown("")
        self.open_btn.on_click(lambda e: self.open_batch(Path(self.path.value.strip()), e))
        self.open_part_btn.on_click(lambda e: self.ask_open_part(e))
        self.report_btn.on_click(lambda e: self.download_report(e))
        self._enable()
        if self.path.value.strip():
            try:
                self.open_batch(Path(self.path.value.strip()), quiet=True)
            except Exception:
                pass

    def _enable(self) -> None:
        has = self.data is not None and bool(self.done_parts())
        self.part.disabled = self.open_part_btn.disabled = self.report_btn.disabled = not has

    def done_parts(self) -> list[dict]:
        return [r for r in (self.data or {}).get("rows", []) if r.get("status") == "done"]

    def open_batch(self, out: Path, e=None, quiet: bool = False) -> bool:
        try:
            self.data = batch_summary.read(out)
        except ValueError as err:
            if not quiet:
                notify(e, "Not a batch", str(err))
            return False
        self.out = Path(out)
        self.path.value = str(out)
        batch_prefs.put("last_batch", str(out))
        self.summary.content = table(self.data)
        names = [r["part"] for r in self.done_parts()]
        self.part.options = tuple(names) or (NONE,)
        self.part.value = names[0] if names else NONE
        self._enable()
        return True

    def row(self, name: str | None = None) -> dict | None:
        name = name or self.part.value
        return next((r for r in self.done_parts() if r["part"] == name), None)

    def _unsaved(self) -> bool:
        from ui.project_panel import _dump

        try:
            return getattr(self.project, "_saved_json", None) not in (None, _dump(self.s.snapshot()))
        except Exception:
            return False

    def ask_open_part(self, e=None) -> None:
        client = getattr(e, "client", None)
        if client is None or not self._unsaved():
            self.open_part(e)
            return
        with client.gui.add_modal("Open this part?") as modal:
            client.gui.add_markdown(
                "The open project has **unsaved changes**. Opening the part replaces it (save it first in "
                "the Project panel to keep them)."
            )
            go = client.gui.add_button("Open the part", color="green")
            stop = client.gui.add_button("Cancel")

        @go.on_click
        def _(ev) -> None:
            modal.close()
            self.open_part(ev)

        stop.on_click(lambda _ev: modal.close())

    def part_dir(self, r: dict) -> Path:
        """The part's project folder (the summary's path, or beside the summary when it moved)."""
        d = Path(r.get("folder", ""))
        if (d / "project.json").is_file() or self.out is None:
            return d
        return self.out / pp.safe(r["part"])

    def open_part(self, e=None, name: str | None = None) -> bool:
        r = self.row(name)
        if r is None:
            notify(e, "Nothing to open", "Open a batch and pick a planned part")
            return False
        if not self.project._load(e, self.part_dir(r)):
            return False
        self.info.content = (
            f"Opened **{r['part']}**: {r.get('reached')}/{r.get('points')} points, {r.get('stops')} stop(s). "
            "Its plans are in the list above."
        )
        if self.auto_paths.value and self.build_paths is not None:
            self.build_paths()
        return True

    def report_path(self, name: str | None = None) -> Path | None:
        r = self.row(name)
        if r is None:
            return None
        rep = Path(r.get("report", ""))
        if rep.is_file():
            return rep
        d = self.part_dir(r)
        for cand in (d / "report.pdf", d / "export" / "report.html"):
            if cand.is_file():
                return cand
        return None

    def download_report(self, e=None) -> None:
        rep = self.report_path()
        if rep is None:
            return notify(e, "No report", "This part has no report file")
        client = getattr(e, "client", None)
        if client is not None:
            client.send_file_download(f"{pp.safe(self.part.value)}_{rep.name}", rep.read_bytes())
        self.info.content = f"Report: `{rep}`"
