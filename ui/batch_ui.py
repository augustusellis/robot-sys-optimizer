"""Optimize -> *Several parts (batch)* (v13 #6): the station optimizer on a folder of part packages
with this project's robot, end effector, external axis, stay-outs and settings.

*Scan* lists the packages of the parts folder with a checkbox each (all ticked; unusable ones are
listed with the reason). *Run batch* takes the open project as it is now (``_setup/`` in the output
folder: no need to save it first) and runs ``ui.batch_run.run_batch`` in a background thread;
each part gets a line with its stage, and the overall bar follows. When it ends the Results tab's
*Batch results* viewer opens the output folder (``on_done``).
"""

from __future__ import annotations

import shutil
import threading
import time
from pathlib import Path

import viser

from core import part_package as pp
from core import schema
from ui import batch_prefs
from ui.log_panel import log
from ui.widgets import notify

SETUP = "_setup"


def default_out(parts_folder: str) -> str:
    return str(Path(parts_folder) / "batch_results") if parts_folder.strip() else ""


class BatchPanel:
    def __init__(self, server: viser.ViserServer, session, on_done=None):
        self.server, self.s, self.on_done = server, session, on_done
        self.found: list[tuple[Path, pp.PartPackage]] = []
        self.checks: dict[str, object] = {}
        self.lines: dict[str, tuple[str, float]] = {}
        self.rows: list[dict] = []
        self._parts: list[tuple[str, Path]] = []
        self._cancel = threading.Event()
        self._thread: threading.Thread | None = None
        gui = server.gui
        self.folder = gui.add_folder("Several parts (batch)", expand_by_default=True)
        with self.folder:
            gui.add_markdown(
                "_Plans every ticked part package (made in the package builder's **Part** tab) with this "
                "project's robot, end effector, external axis, stay-outs and settings: stations, paths, a "
                "saved project and a PDF report per part._"
            )
            self.parts_folder = gui.add_text("Parts folder", batch_prefs.get("parts_folder"))
            self.scan_btn = gui.add_button("Scan")
            self.list = gui.add_folder("Parts", expand_by_default=True)
            with self.list:
                self.bad_md = gui.add_markdown("")
            self.out = gui.add_text(
                "Output folder",
                batch_prefs.get("output_folder") or default_out(self.parts_folder.value),
                hint="one folder per part, plus summary.pdf / .csv",
            )
            self.frame_search = gui.add_checkbox(
                "Search the axis frame per part",
                False,
                hint="Rail / gantry: place the frame for each part first (Gantry placement); off = the "
                "project's frame",
            )
            self.workers_md = gui.add_markdown("")
            self.reason_md = gui.add_markdown("")
            self.run = gui.add_button("Run batch", color="green")
            self.stop = gui.add_button("Cancel", color="red", visible=False)
            self.progress = gui.add_progress_bar(0.0, visible=False)
            self.status = gui.add_markdown("")
        self.scan_btn.on_click(lambda e: self.scan(e))
        self.run.on_click(lambda e: self.start(e))
        self.stop.on_click(lambda e: self._cancel.set())
        self.parts_folder.on_update(lambda e: self._folder_changed())
        self.out.on_update(lambda e: batch_prefs.put("output_folder", self.out.value.strip()))

    # --- the list ---------------------------------------------------------------------------------

    def _folder_changed(self) -> None:
        old_default = batch_prefs.get("output_folder") in ("", default_out(batch_prefs.get("parts_folder")))
        batch_prefs.put("parts_folder", self.parts_folder.value.strip())
        if old_default:
            self.out.value = default_out(self.parts_folder.value)

    def scan(self, e=None) -> list[str]:
        for cb in self.checks.values():
            cb.remove()
        self.checks = {}
        text = self.parts_folder.value.strip()
        try:
            ok, bad = pp.scan_folder(Path(text)) if text else ([], [])
        except ValueError as err:
            notify(e, "Cannot scan", str(err))
            ok, bad = [], []
        self.found = ok
        with self.list:
            for d, pkg in ok:
                cb = self.server.gui.add_checkbox(
                    d.name, True, hint=f"{pkg.name}: {len(pkg.targets.points)} targets"
                )
                cb.on_update(lambda _e: self.refresh())
                self.checks[d.name] = cb
        lines = [f"✘ `{d.name}`: {why}" for d, why in bad]
        if not ok:
            lines.insert(0, "_No part packages in this folder._" if text else "_Type the parts folder._")
        self.bad_md.content = "  \n".join(lines)
        self.refresh()
        return list(self.checks)

    def ticked(self) -> list[tuple[str, Path]]:
        return [(d.name, d) for d, _ in self.found if d.name in self.checks and self.checks[d.name].value]

    def why_not(self) -> str:
        s = self.s
        if s.robot is None:
            return "Choose a robot first (tab 1)."
        if s.tool_model is None:
            return "Set up the end effector first (tab 3)."
        if not self.found:
            return "Scan a parts folder first."
        if not self.ticked():
            return "Tick at least one part."
        if not self.out.value.strip():
            return "Type the output folder."
        return ""

    def refresh(self) -> None:
        from ui.batch_run import plan_workers

        n = len(self.ticked())
        if n:
            _, why = plan_workers(n)
            self.workers_md.content = f"_{n} part(s): {why}._"
        else:
            self.workers_md.content = ""
        reason = self.why_not()
        self.reason_md.content = f"_{reason}_" if reason else ""
        th = self._thread  # the batch thread's own last refresh is not busy
        busy = th is not None and th.is_alive() and th is not threading.current_thread()
        self.run.disabled = bool(reason) or busy

    # --- running ----------------------------------------------------------------------------------

    def write_setup(self, out: Path) -> Path:
        """The open project as it is now, as a project folder ``<out>/_setup`` for the workers."""
        setup = out / SETUP
        if setup.exists():
            shutil.rmtree(setup)
        setup.mkdir(parents=True)
        src = Path(self.s.project_dir) / "assets"
        if src.is_dir():
            shutil.copytree(src, setup / "assets", ignore=shutil.ignore_patterns("_builder"))
        project = self.s.snapshot()
        project = project.model_copy(update={"results": [], "compared": []})
        schema.save_project(project, setup / "project.json")
        return setup

    def _progress(self, name: str, stage: str, frac: float) -> None:
        self.lines[name] = (stage, frac)
        parts = [n for n, _ in self._parts]
        self.progress.value = 100.0 * sum(self.lines.get(n, ("", 0.0))[1] for n in parts) / max(len(parts), 1)
        icon = {"done": "✔", "failed": "✘", "cancelled": "–"}
        self.status.content = "  \n".join(
            f"{icon.get(st, '…')} **{n}**: {st}"
            for n in parts
            for st, _ in [self.lines.get(n, ("queued", 0))]
        )

    def start(self, e=None, block: bool = False) -> None:
        if self._thread is not None and self._thread.is_alive():
            return notify(e, "Busy", "A batch is already running")
        reason = self.why_not()
        if reason:
            return notify(e, "Cannot run the batch", reason)
        from ui import jobs
        from ui.batch_run import run_batch

        out = Path(self.out.value.strip())
        try:
            setup = self.write_setup(out)
        except (OSError, ValueError) as err:
            return notify(e, "Cannot write to the output folder", str(err))
        self._parts = self.ticked()
        self.lines = {}
        self._cancel.clear()
        self.run.visible, self.stop.visible, self.progress.visible = False, True, True
        self.progress.value = 0.0
        self._progress(self._parts[0][0], "queued", 0.0)
        t0 = time.monotonic()
        frame = bool(self.frame_search.value)
        log(f"Batch started: {len(self._parts)} part(s) into `{out}`.", "info")

        def work() -> None:
            try:
                self.rows = run_batch(
                    setup,
                    self._parts,
                    out,
                    frame_search=frame,
                    check_workers=jobs.CHECK_WORKERS,
                    on_progress=self._progress,
                    cancelled=self._cancel.is_set,
                    parts_folder=self.parts_folder.value.strip(),
                )
                for r in self.rows:
                    self._progress(
                        r["part"], r["status"] + (f": {r['reason']}" if r.get("reason") else ""), 1.0
                    )
                done = sum(r["status"] == "done" for r in self.rows)
                for r in self.rows:
                    if r["status"] == "failed":
                        log(f"Batch: **{r['part']}** failed: {r.get('reason', '')}", "error")
                log(f"Batch finished: {done} of {len(self.rows)} part(s) planned; `{out}`.", "done", t0)
                if self.on_done is not None:
                    self.on_done(out)
            except Exception as err:
                self.status.content = f"⚠ batch failed: {err}"
                log(f"Batch failed: {err}", "error")
            finally:
                self.run.visible, self.stop.visible, self.progress.visible = True, False, False
                self.refresh()

        self._thread = threading.Thread(target=work, daemon=True)
        self._thread.start()
        if block:
            self._thread.join()
