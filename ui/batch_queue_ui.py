"""Optimize -> Optimizer: Batch queue (plan Part 2, step B5): queue part + selection jobs, plan later.

*Add to queue* snapshots the current part, face selection, point-generator settings and the
Station planner panel into ``<workspace>/batch_<time>/batch.json`` (``core.batch``);
*Run batch* plans the pending jobs one after another (``ui.batch_job.run_batch``) and
writes a study folder per job plus ``summary.html``. The manifest on disk is the truth:
every edit reloads it first, and the runner updates single jobs (``bt.update_job``).
"""

from __future__ import annotations

import shutil
import threading
from datetime import datetime
from pathlib import Path

import viser

from core import batch as bt
from core import data_paths
from ui.log_panel import log
from ui.widgets import notify

MODES = {"Plan stations": False, "Quick plan": True}  # label -> planner_snapshot(quick=...)
REDO = ("failed", "partial", "cancelled")  # statuses that can be reset to pending


def _cell(text: str) -> str:
    return str(text).replace("|", "/").replace("\n", " ")[:120]


def job_table(m: bt.BatchManifest) -> str:
    if not m.jobs:
        return "_The queue is empty._"
    lines = []
    for j in m.jobs:
        s = j.summary or {}
        n = s.get("total", s.get("targets_preview", ""))
        got = f"{s.get('reached', '')}/{s.get('total', '')}, {s['cost']:.2f}" if s.get("stations") else ""
        mode = "quick" if j.planner.get("mode") == "quick" else "plan"
        name = f" {_cell(j.name)}" if j.name != j.id else ""
        lines.append(
            f"| {j.id}{name} | {_cell(Path(j.part).name)} | {len(j.selection)} | {n} | {mode} | "
            f"{j.status} | {got} | {_cell(j.error)} |"
        )
    return (
        "| job | part | faces | targets | mode | status | reached, cost | error |\n"
        "|---|---|---|---|---|---|---|---|\n" + "\n".join(lines)
    )


class BatchPanel:
    def __init__(self, server: viser.ViserServer, session, project, targets, stations):
        self.s, self.project, self.targets, self.stations = session, project, targets, stations
        self.batch_dir: Path | None = None
        self._thread: threading.Thread | None = None
        self._cancel = threading.Event()
        self.last_summary: Path | None = None
        gui = server.gui
        self.folder = gui.add_folder("Batch queue", expand_by_default=False)  # Optimizer -> Batch queue
        with self.folder:
            gui.add_markdown(
                "_Select faces on the part as usual, then **Add to queue**; do this for more parts. "
                "**Run batch** plans every queued job in this cell and saves one study folder per job._"
            )
            self.mode = gui.add_dropdown("Mode", tuple(MODES), hint="the planner used for the job")
            self.add = gui.add_button("Add to queue", color="green")
            self.gate_md = gui.add_markdown("", visible=False)
            self.batch_md = gui.add_markdown("_No batch yet: it is created on the first Add to queue._")
            self.existing = gui.add_dropdown("Open batch", ("—",))
            self.open_batch_btn = gui.add_button("Open batch…")
            self.table = gui.add_markdown("")
            self.pick = gui.add_dropdown("Job", ("—",))
            self.remove_btn = gui.add_button("Remove")
            self.up_btn = gui.add_button("Move up")
            self.down_btn = gui.add_button("Move down")
            self.reset_btn = gui.add_button(
                "Reset to pending", hint="run a failed, partial or cancelled job again"
            )
            self.open_btn = gui.add_button("Open result", hint="open the finished job's study as the project")
            self.run_btn = gui.add_button("Run batch", color="green")
            self.stop = gui.add_button("Cancel", visible=False)
            self.progress = gui.add_progress_bar(0.0, visible=False)
            self.status = gui.add_markdown("")
        self.add.on_click(self._guard(self.add_to_queue))
        self.open_batch_btn.on_click(self._guard(lambda e: self.open_batch(self.existing.value, e)))
        self.remove_btn.on_click(self._guard(lambda e: self.remove(e)))
        self.up_btn.on_click(self._guard(lambda e: self.move(-1, e)))
        self.down_btn.on_click(self._guard(lambda e: self.move(1, e)))
        self.reset_btn.on_click(self._guard(lambda e: self.reset(e)))
        self.open_btn.on_click(self._guard(lambda e: self.open_result(e)))
        self.run_btn.on_click(self._guard(lambda e: self.run(e)))
        self.stop.on_click(lambda e: self._cancel.set())
        self._lock_stations()
        self.list_batches()

    # --- helpers ------------------------------------------------------------------------------

    @staticmethod
    def _guard(fn):
        def inner(event=None):
            try:
                fn(event)
            except Exception as err:  # never die silently in a callback
                notify(event, "Batch queue", f"{type(err).__name__}: {err}")

        return inner

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def _lock_stations(self) -> None:
        """While a batch runs, the Station planner says so instead of queuing behind it."""
        st, orig = self.stations, self.stations.start

        def start(event=None, *a, **kw):
            if self.running:
                return notify(event, "Batch running", "A batch is running: wait for it or cancel it first")
            return orig(event, *a, **kw)

        st.start = start

    def list_batches(self) -> list[str]:
        ws = data_paths.workspace_dir()
        names = sorted((p.name for p in ws.glob("batch_*") if (p / bt.MANIFEST_NAME).is_file()), reverse=True)
        self.existing.options = tuple(names) or ("—",)
        return names

    def manifest(self) -> bt.BatchManifest:
        if self.batch_dir is None:
            raise ValueError("no batch is open: use Add to queue or Open batch…")
        return bt.load_manifest(self.batch_dir / bt.MANIFEST_NAME)

    def _save(self, m: bt.BatchManifest) -> None:
        bt.save_manifest(m, self.batch_dir / bt.MANIFEST_NAME)

    def refresh(self) -> None:
        if self.batch_dir is None:
            return
        try:
            m = self.manifest()
        except ValueError as err:
            self.table.content = f"⚠ {err}"
            return
        self.batch_md.content = f"Batch `{self.batch_dir.name}` (cell study `{m.cell_project}`)"
        self.table.content = job_table(m)
        keep = self.pick.value.split()[0]
        opts = tuple(f"{j.id}  {j.name}" if j.name != j.id else j.id for j in m.jobs)
        self.pick.options = opts or ("—",)
        self.pick.value = next((o for o in opts if o.split()[0] == keep), opts[0]) if opts else "—"

    def _job_id(self) -> str:
        v = self.pick.value
        if v == "—":
            raise ValueError("the queue has no job selected")
        return v.split()[0]

    # --- batch folder -------------------------------------------------------------------------

    def open_batch(self, name: str, event=None) -> bool:
        d = data_paths.workspace_dir() / name
        if name == "—" or not (d / bt.MANIFEST_NAME).is_file():
            notify(event, "Open batch", "Pick a batch folder from the list (workspace/batch_*)")
            return False
        bt.load_manifest(d / bt.MANIFEST_NAME)  # a bad file raises, reported by the guard
        self.batch_dir = d
        self.status.content = ""
        self.refresh()
        return True

    # --- add ----------------------------------------------------------------------------------

    def add_reason(self) -> str:
        from ui.workflow import setup_reason

        s = self.s
        if s.part is None:
            return "Load a part first"
        if len(self.targets.selection) == 0:
            return "Select some faces of the part first (Targets tab)"
        return setup_reason(s)

    def add_to_queue(self, event=None) -> bool:
        from core.target_gen import generate_targets
        from ui.batch_planner import planner_snapshot

        if self.running:
            notify(event, "Batch running", "Wait for the batch to finish before adding jobs")
            return False
        if reason := self.add_reason():
            notify(event, "Cannot queue", reason)
            return False
        s, sel, gen = self.s, [int(f) for f in self.targets.selection], self.targets.gen_settings()
        mesh = self.targets.mesh()
        if mesh is None:
            notify(event, "Cannot queue", "The part mesh could not be read")
            return False
        P, _N, _g = generate_targets(mesh, sel, gen)
        if len(P) == 0:
            notify(
                event, "No targets", "These settings generate 0 targets on the selection; change them first"
            )
            return False
        try:
            self.project.save_now()  # the cell project must be on disk
        except (OSError, ValueError) as err:
            notify(event, "Cannot queue", f"save the project first ({err})")
            return False
        if self.batch_dir is None:
            self.batch_dir = data_paths.workspace_dir() / f"batch_{datetime.now():%Y%m%d_%H%M%S}"
            self.batch_dir.mkdir(parents=True, exist_ok=True)
            self._save(bt.BatchManifest(cell_project=bt.rel_cell(s.project_dir, self.batch_dir)))
        m = self.manifest()
        src = s.resolve(s.part.mesh_path)
        parts = self.batch_dir / "parts"
        parts.mkdir(exist_ok=True)
        dest, k = parts / src.name, 1
        while dest.exists() and bt.mesh_sha1(dest) != bt.mesh_sha1(
            src
        ):  # unique name; reuse identical copies
            dest, k = parts / f"{src.stem}_{k}{src.suffix}", k + 1
        if not dest.exists():
            shutil.copyfile(src, dest)
        job = bt.add_job(
            m,
            part=dest.relative_to(self.batch_dir).as_posix(),
            selection=sel,
            gen=gen,
            units=s.part.units,
            pose=[[float(x) for x in row] for row in s.part.T_world_part],
            mesh_sha1=bt.mesh_sha1(dest),
            planner=planner_snapshot(self.stations, quick=MODES[self.mode.value]),
            summary={"targets_preview": len(P)},
        )
        self._save(m)
        self.list_batches()
        self.refresh()
        notify(event, "Queued", f"{job.id}: {len(sel)} faces, {len(P)} targets", ok=True)
        return True

    # --- edit ---------------------------------------------------------------------------------

    def _editable(self, m: bt.BatchManifest, jid: str, event) -> bool:
        job = next((j for j in m.jobs if j.id == jid), None)
        if job is None:
            notify(event, "Batch queue", f"{jid} is no longer in the queue")
            return False
        if self.running and job.status != "pending":
            notify(
                event, "Batch running", f"{jid} is {job.status}; only pending jobs can change while it runs"
            )
            return False
        return True

    def remove(self, event=None) -> bool:
        m, jid = self.manifest(), self._job_id()
        if not self._editable(m, jid, event):
            return False
        m.jobs = [j for j in m.jobs if j.id != jid]
        self._save(m)
        self.refresh()
        return True

    def move(self, delta: int, event=None) -> bool:
        m, jid = self.manifest(), self._job_id()
        i = next((n for n, j in enumerate(m.jobs) if j.id == jid), -1)
        k = i + delta
        if i < 0 or not 0 <= k < len(m.jobs):
            return False
        if not (self._editable(m, jid, event) and self._editable(m, m.jobs[k].id, event)):
            return False
        m.jobs[i], m.jobs[k] = m.jobs[k], m.jobs[i]
        self._save(m)
        self.refresh()
        return True

    def reset(self, event=None) -> bool:
        m, jid = self.manifest(), self._job_id()
        job = next((j for j in m.jobs if j.id == jid), None)
        if job is None or job.status not in REDO:
            notify(event, "Reset", "Only failed, partial or cancelled jobs can be reset to pending")
            return False
        job.status, job.error = "pending", ""
        job.summary = {k: v for k, v in job.summary.items() if k == "targets_preview"}
        bt.update_job(self.batch_dir / bt.MANIFEST_NAME, job)
        self.refresh()
        return True

    # --- run ----------------------------------------------------------------------------------

    def run(self, event=None, block: bool = False) -> None:
        from ui import jobs
        from ui.batch_job import run_batch

        if self.running:
            return notify(event, "Busy", "The batch is already running")
        if self.stations._thread is not None and self.stations._thread.is_alive():
            return notify(event, "Busy", "The Station planner is running: wait for it first")
        m = self.manifest()
        if not bt.cell_path(m, self.batch_dir).is_dir():
            return notify(event, "Cannot run", f"the cell study {m.cell_project} is missing")
        if not any(j.status in ("pending", "running") for j in m.jobs):
            return notify(event, "Nothing to run", "No pending jobs: add some or reset a failed one")
        self._cancel.clear()
        self.run_btn.visible, self.stop.visible, self.progress.visible = False, True, True
        self.progress.value = 0.0
        t0 = datetime.now()
        warns: list[str] = []

        def on_msg(kind: str, data) -> None:
            try:
                if kind == "progress":
                    self.status.content = f"_{data[0]}_"
                    self.progress.value = 100.0 * data[1]
                elif kind == "warn":
                    warns.append(str(data))
                    log(f"Batch: {data}.", "warn")
                elif kind == "job":
                    self.refresh()
                elif kind == "finished":
                    self._finished(data, warns, t0)
            except Exception as err:  # a display problem must not kill the runner thread
                log(f"Batch panel: {type(err).__name__}: {err}", "error")

        self._thread = run_batch(self.batch_dir, on_msg, self._cancel.is_set, in_process=not jobs.USE_WORKER)
        if block:
            self._thread.join()

    def _finished(self, data: dict, warns: list[str], t0) -> None:
        try:
            self.refresh()
            rows = data.get("jobs", [])
            count = {
                k: sum(r["status"] == k for r in rows) for k in ("done", "failed", "partial", "cancelled")
            }
            html = self.batch_dir / "summary.html"
            self.last_summary = html if html.is_file() else None
            ok = not data.get("error")
            head = f"**Batch {data['status']}**" if ok else f"⚠ **Batch failed**: {data['error']}"
            text = f"{head}: " + ", ".join(f"{n} {k}" for k, n in count.items() if n) + "."
            if self.last_summary:
                text += f"\n\nSummary: `{self.last_summary}`"
            if warns:
                text += "\n\n" + "\n\n".join(f"⚠ {w}" for w in warns[:5])
            self.status.content = text
            log(f"Batch finished ({data['status']}).", "done" if data["status"] == "done" else "warn", t0)
        finally:
            self.run_btn.visible, self.stop.visible, self.progress.visible = True, False, False

    def open_result(self, event=None) -> bool:
        m, jid = self.manifest(), self._job_id()
        job = next((j for j in m.jobs if j.id == jid), None)
        if job is None or job.status not in ("done", "partial") or not job.out_dir:
            notify(event, "Open result", "Only a finished job has a study to open")
            return False
        d = self.batch_dir / job.out_dir
        if not (d / "project.json").is_file():
            notify(event, "Open result", f"the study folder {d} is missing")
            return False
        return self.project._load(event, d)
