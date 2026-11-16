"""Results -> Edge check (v10 T10): exact checks of verification points where a sparse plan can
hide gaps (between stops' areas, between configuration zones, along part edges and creases, around
low-margin points; core.edge_check), run as a job like the tolerance check. Gaps go to the log, to
Targets -> Display -> *Colour part by* -> *Edge check* and to the unreached markers; **Densify
edges** adds them as targets (nearest target's group, one undo step)."""

from __future__ import annotations

import threading
import time
from collections.abc import Callable

import numpy as np
import viser

from core import targets as tg
from core.feasibility import CancelledError
from ui.log_panel import log
from ui.plan_ids import key as plan_key
from ui.session_lock import LOCK
from ui.widgets import notify

ROOT = "/cell/part/edge_gaps"


class EdgePanel:
    def __init__(self, server: viser.ViserServer, session, selected: Callable):
        self.server, self.s, self.selected = server, session, selected
        self._thread: threading.Thread | None = None
        self._cancel = threading.Event()
        self.on_done: Callable[[], None] | None = None  # e.g. redraw the targets (markers, colours)
        self.targets = None  # TargetsTab (Densify edges), set by app.py
        gui = server.gui
        with gui.add_folder("Edge check", expand_by_default=False) as self.folder:
            gui.add_markdown(
                "_Checks extra points between the stops' areas, between configuration zones, along "
                "the part's edges and creases and around points that were only just reached. The "
                "plan checks only its target points; gaps show up here._"
            )
            self.auto = gui.add_checkbox(
                "Check after planning", True, hint="run the edge check on the best plan of each planner run"
            )
            self.run = gui.add_button("Run edge check", color="green")
            self.gate_md = gui.add_markdown("", visible=False)
            self.stop = gui.add_button("Cancel", visible=False)
            self.progress = gui.add_progress_bar(0.0, visible=False)
            self.status = gui.add_markdown("")
            self.densify = gui.add_button("Densify edges", disabled=True, hint="add the gaps as targets")
        self.run.on_click(lambda e: self.start(e))
        self.stop.on_click(lambda e: self._cancel.set())
        self.densify.on_click(lambda e: self.add_gaps(e))

    def current(self):
        """The last edge check if it belongs to the selected plan, else None."""
        r, sol = getattr(self.s, "edge_check", None), self.selected()
        return r if r is not None and sol is not None and (r.key or r.label) == plan_key(sol) else None

    def after_plan(self, best) -> None:
        """StationsPanel.on_done (in the planner's thread): check the best plan when *Check after
        planning* is on; the planner run ends after it (nothing else edits the session meanwhile)."""
        if self.auto.value and best is not None and self.s.result_points.get(plan_key(best)):
            self.start(sol=best, block=True)

    def start(self, event=None, block: bool = False, sol=None) -> None:
        s = self.s
        if self._thread is not None and self._thread.is_alive():
            return notify(event, "Busy", "The edge check is already running")
        sol = sol or self.selected()
        res = s.result_points.get(plan_key(sol)) if sol is not None else None
        if sol is None or not res:
            return notify(
                event, "No plan", "Select a plan in the Results list first (with its joint solutions)"
            )
        if s.robot is None or s.tool_model is None or s.part is None:
            return notify(event, "Cannot check", "Need a robot, a tool and a part")
        from ui import jobs
        from ui.plan_job import payload_for

        spec = {"sol": sol, "results": res, "workers": jobs.CHECK_WORKERS}
        in_process = block or not jobs.USE_WORKER
        payload = payload_for(s, spec, live=in_process)
        self._cancel.clear()
        self.run.visible, self.stop.visible, self.progress.visible = False, True, True
        t0 = time.monotonic()

        def on_msg(kind: str, data) -> None:
            if kind == "progress":
                self.status.content = f"_{data[0]}_"
                self.progress.value = 100.0 * data[1]
            elif kind == "warn":
                log(f"Edge check: {data}.", "warn")

        def work() -> None:
            try:
                r = jobs.call("ui.plan_job:edge_job", payload, on_msg, self._cancel.is_set, in_process)
                with LOCK:  # v12 #9
                    s.edge_check = r
                self.status.content = report(r)
                log(
                    f"Edge check of **{sol.label}**: {r.summary()}{where(r)}.",
                    "warn" if r.gaps else "done",
                    t0,
                )
                self.densify.disabled = not r.gaps
                if self.on_done is not None:
                    self.on_done()
            except (CancelledError, jobs.JobCancelledError):
                self.status.content = "**Cancelled.**"
                log("Edge check cancelled.", "warn", t0)
            except Exception as err:
                self.status.content = f"⚠ edge check failed: {err}"
                log(f"Edge check failed: {err}", "error")
                raise
            finally:
                self.run.visible, self.stop.visible, self.progress.visible = True, False, False

        self._thread = threading.Thread(target=work, daemon=True)
        self._thread.start()
        if block:
            self._thread.join()

    def add_gaps(self, event=None) -> int:
        """Densify edges: the gaps become targets in their nearest target's group (one undo step)."""
        r, s = self.current(), self.s
        if r is None or not r.gaps:
            return notify(event, "No gaps", "Run the edge check on the selected plan first") or 0
        group = {p.id: p.group for p in s.targets.points}
        known = {p.id for p in s.targets.points}
        before = set(known)
        by: dict[str, list] = {}
        for e in r.gaps:
            by.setdefault(
                group.get(e.near, s.targets.groups[0].name if s.targets.groups else "default"), []
            ).append(e)
        for g, es in by.items():
            s.targets = tg.append_points(
                s.targets, np.array([e.p for e in es]), np.array([e.n for e in es]), g
            )
        added = [p.id for p in s.targets.points if p.id not in before]
        tools = getattr(self.targets, "tools", None)
        if tools is not None:
            tools.undo_stack.append(added)
        s.edge_check = None
        self.densify.disabled = True
        self.status.content = f"Added **{len(added)}** edge points as targets: plan again to include them."
        log(f"Densify edges: {len(added)} target point(s) added at the gaps.", "done")
        if self.targets is not None:
            self.targets.refresh()
        return len(added)


def where(r) -> str:
    """'; stops 1-2 (5), part edge at stop 3 (2)' (empty without gaps)."""
    from core.edge_check import KINDS

    by: dict[str, int] = {}
    for e in r.gaps:
        stops = "-".join(str(i + 1) for i in e.stops)
        k = f"{KINDS[e.kind]} at stop{'s' if len(e.stops) > 1 else ''} {stops}"
        by[k] = by.get(k, 0) + 1
    return "".join(f"; {k} ({n})" for k, n in sorted(by.items(), key=lambda kv: -kv[1])[:6])


def report(r) -> str:
    from core.edge_check import KINDS
    from ui.unreached_view import SHORT

    head = f"**{r.summary()}** (spacing {1000 * r.spacing:.0f} mm)."
    if not r.gaps:
        return head
    rows = "\n".join(
        f"| {KINDS[e.kind]} | {'-'.join(str(i + 1) for i in e.stops)} | {e.near} | "
        f"{SHORT.get(e.reason, e.reason.replace('_', ' '))} |"
        for e in r.gaps[:15]
    )
    more = f"\n\n_… and {len(r.gaps) - 15} more._" if len(r.gaps) > 15 else ""
    return (
        head
        + " Gaps are marked on the part (✘ edge) and in *Colour part by → Edge check*.\n\n"
        + "| where | stop(s) | near point | why |\n|---|---|---|---|\n"
        + rows
        + more
    )


def gap_arrays(r) -> tuple[np.ndarray, np.ndarray]:
    if r is None or not r.gaps:
        return np.zeros((0, 3)), np.zeros((0, 3))
    return np.array([e.p for e in r.gaps]), np.array([e.n for e in r.gaps])
