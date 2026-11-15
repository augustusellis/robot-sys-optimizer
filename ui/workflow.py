"""Guided setup (v2 phase 2): step headers, the robot + carrier lock, out-of-date banners.

The tabs are steps 1 Cell → 2 External Axis → 3 Tool → 4 Targets → 5 Optimize → 6 Results.
Each setup step has a status line and **Confirm**; any tab can be opened at any time (user
decision: guided, free navigation). Confirming the External Axis step locks the carrier
setup (no editing, the robot rides on it); **Edit setup** unlocks it. In movable mode the
Cell tab never lets the robot base be moved off the carrier.

``refresh()`` recomputes everything from the session (ui.staleness fingerprints) and only
touches widgets when something changed; the app calls it from a 1 s watcher thread and
after every confirm / load.
"""

from __future__ import annotations

import threading
import time

import viser

from ui.session import Session
from ui.staleness import STEP_COMPONENTS, confirm_step, feasibility_changes, solution_changes, step_state

STEPS = tuple(STEP_COMPONENTS)  # setup steps, in tab order
# the drag gizmos (Objects -> Drag gizmo on) of each setup step: off once the step is confirmed
STEP_GIZMOS = {
    "Cell": ("Robot base", "Part", "Stay-out zone", "Reduced-speed zone"),
    "External Axis": ("Carrier body",),
    "Tool Setup": ("Tool mount",),
}
TITLES = {
    "Cell": "Robot & cell",
    "External Axis": "External axis",
    "Tool Setup": "Tool",
    "Targets": "Targets",
}
ICON = {
    "confirmed": "✔ confirmed",
    "changed": "✎ changed since you confirmed it",
    "todo": "○ not confirmed yet",
}
MARK = {"confirmed": "✔", "changed": "✎", "todo": "○"}  # v10 #8: status strip


def strip_text(states: dict, n_results: int, sol_changes: list, feas: list, has_feas: bool) -> str:
    """The status strip: each setup step's state, then whether the plan and the check are up to
    date (v10 #8)."""
    steps = " · ".join(f"{MARK[states[st]]} {i + 1} {TITLES[st]}" for i, st in enumerate(STEPS))
    if not n_results:
        plan = "no plan yet"
    elif sol_changes:
        plan = f"⚠ plan out of date ({', '.join(sol_changes)})"
    else:
        plan = "✔ plan up to date"
    check = (
        f"⚠ check out of date ({', '.join(feas)})"
        if feas
        else "✔ check up to date"
        if has_feas
        else "no check"
    )
    return f"{steps}  \n{plan} · {check}"


def setup_reason(s: Session) -> str:
    """Why the optimizers must wait (markdown), or "" when setup steps 1-4 are confirmed.
    The station planner, the quick planner and the tool optimizer need a confirmed setup; the
    feasibility check and the heatmap stay usable (user decision, v7 round 2)."""
    todo, changed = [], []
    for i, st in enumerate(STEPS):
        state = step_state(s, st)
        name = f"**{i + 1} {TITLES[st]}**"
        if state == "todo":
            todo.append(name)
        elif state == "changed":
            changed.append(name)
    parts = []
    if todo:
        parts.append(f"Confirm {', '.join(todo)} first")
    if changed:
        it = "them" if len(changed) > 1 else "it"
        parts.append(f"{', '.join(changed)} changed since you confirmed {it} — confirm {it} again")
    return "; ".join(parts)


def feasibility_reason(s: Session) -> str:
    """Why *Check all targets* is disabled (T16)."""
    if s.robot is None:
        return "Choose a robot first (**1 Robot & cell**)"
    if s.part is None:
        return "Load a part first (**1 Robot & cell**)"
    if s.tool is None:
        return "Choose a tool first (**3 Tool**)"
    if not s.targets.points:
        return "Add targets first (**4 Targets**)"
    return ""


def result_reason(selected):
    """Reason function: no result selected in Results (Export, T16)."""

    def fn(s: Session) -> str:
        if selected() is None:
            return "No result to export yet: run **5 Optimize → Plan stations** (or the tool optimizer) first"
        return ""

    return fn


def plan_reason(selected):
    """Reason function: no station plan selected (Build paths, T16)."""

    def fn(s: Session) -> str:
        if selected() is None:
            return "No plan yet: run **5 Optimize → Plan stations**, then select it in Results"
        return ""

    return fn


class Workflow:
    def __init__(self, server: viser.ViserServer, session: Session):
        self.server = server
        self.s = session
        self.headers: dict[str, tuple] = {}
        self.opt_md = None
        self.results_md = None
        self.strip_md = None  # v10 #8: status strip above the tabs (app.py)
        self.cell = None  # set by app.py
        self.carrier = None
        self.results = None
        self._last = None
        self._todo: list[str] | None = None  # unconfirmed steps last reported to the log
        self._lock = threading.Lock()
        self.hooks: list = []  # extra callables run by the watcher (e.g. object visibility)
        self.gates: list[tuple] = []  # (run button, reason markdown): disabled until setup is confirmed
        self.listeners: list = []  # () -> None after every refresh (e.g. Tool geometry's Confirm tool)
        self.on_confirmed: list = []  # (step) -> None when a step becomes confirmed (gizmos off)
        self._states: dict = {}  # step -> state at the last refresh

    # --- widgets (created inside each tab, before its panel) ------------------------------------

    def header(self, step: str) -> None:
        gui = self.server.gui
        md = gui.add_markdown("")
        label = "Confirm setup" if step == "External Axis" else "Confirm step"
        ok = gui.add_button(label, color="green", hint="Mark this step as done")
        edit = gui.add_button("Edit setup", visible=False) if step == "External Axis" else None
        ok.on_click(lambda e: self.confirm(step))
        if edit is not None:
            edit.on_click(lambda e: self.edit(step))
        self.headers[step] = (md, ok, edit)

    def gate(self, button, md, reason=None) -> None:
        """Disable ``button`` (a handle or a tuple of them) and say why in ``md`` while
        ``reason(session)`` returns text; default: setup steps 1-4 not all confirmed (T5).
        v7 round 2 T16: other buttons say why too (no result to export, no plan, no targets)."""
        buttons = button if isinstance(button, tuple) else (button,)
        self.gates.append((buttons, md, reason or setup_reason))
        self._apply_gates()

    def gate_reasons(self) -> tuple:
        out = []
        for _, _, fn in self.gates:
            try:
                out.append(fn(self.s))
            except Exception:  # a mid-edit state: say nothing rather than break the watcher
                out.append("")
        return tuple(out)

    def _apply_gates(self, reasons: tuple | None = None) -> None:
        reasons = reasons if reasons is not None else self.gate_reasons()
        for (buttons, md, _), reason in zip(self.gates, reasons, strict=True):
            for b in buttons:
                b.disabled = bool(reason)
            md.content = f"⚠ {reason}." if reason else ""
            md.visible = bool(reason)

    def status_strip(self) -> None:
        self.strip_md = self.server.gui.add_markdown("")

    def optimize_header(self) -> None:
        self.opt_md = self.server.gui.add_markdown("")

    def confirm_all_button(self) -> None:
        """v11 #5: Optimize -> **Confirm all setup steps** (a dialog lists what is not confirmed)."""
        self.confirm_all_btn = self.server.gui.add_button(
            "Confirm all setup steps", hint="steps 1-4 at once, after a dialog listing the unconfirmed ones"
        )
        self.confirm_all_btn.on_click(lambda e: self.confirm_all(e))

    def unconfirmed(self) -> list[str]:
        """'2 External axis (changed since it was confirmed)', ... for the steps not confirmed."""
        out = []
        for i, st in enumerate(STEPS):
            state = step_state(self.s, st)
            if state != "confirmed":
                why = "changed since it was confirmed" if state == "changed" else "not confirmed yet"
                out.append(f"{i + 1} {TITLES[st]} ({why})")
        return out

    def confirm_all(self, event=None, sure: bool = False) -> bool:
        """Confirm every setup step; with a browser client, after an *Are you sure?* dialog."""
        from ui.widgets import notify

        todo = self.unconfirmed()
        if not todo:
            notify(event, "Already confirmed", "Every setup step is confirmed", ok=True)
            return True
        client = getattr(event, "client", None)
        if client is not None and not sure:
            with client.gui.add_modal("Confirm all setup steps?") as modal:
                client.gui.add_markdown(
                    "These setup tabs have not been confirmed:\n\n"
                    + "\n".join(f"- **{t}**" for t in todo)
                    + "\n\nConfirm them all as they are now?"
                )
                yes = client.gui.add_button("Confirm all", color="green")
                no = client.gui.add_button("Cancel")

            def go(_e) -> None:
                modal.close()
                self.confirm_all(event, sure=True)

            yes.on_click(go)
            no.on_click(lambda _e: modal.close())
            return False
        for st in STEPS:
            confirm_step(self.s, st)
        self.refresh(force=True)
        notify(event, "Setup confirmed", ", ".join(todo), ok=True)
        return True

    def results_header(self) -> None:
        self.results_md = self.server.gui.add_markdown("")

    # --- actions ------------------------------------------------------------------------------

    def confirm(self, step: str) -> None:
        confirm_step(self.s, step)
        self.refresh(force=True)

    def edit(self, step: str) -> None:
        self.s.steps.pop(step, None)
        self.refresh(force=True)

    # --- state -> widgets ---------------------------------------------------------------------

    def refresh(self, force: bool = False) -> None:
        with self._lock:
            states = {st: step_state(self.s, st) for st in STEPS}
            became = [st for st in STEPS if states[st] == "confirmed" and self._states.get(st) != "confirmed"]
            self._states = states
        for st in became:  # outside the lock: the hooks change widgets
            for fn in self.on_confirmed:
                fn(st)
        self._refresh(force)

    def forget_states(self) -> None:
        """After a project load: its confirmed steps count as newly confirmed (gizmos off)."""
        self._states = {}

    def _refresh(self, force: bool = False) -> None:
        with self._lock:
            s = self.s
            states = {st: step_state(s, st) for st in STEPS}
            self._log_setup(
                [f"{i + 1} {TITLES[st]}" for i, st in enumerate(STEPS) if states[st] != "confirmed"]
            )
            locked = states["External Axis"] != "todo"
            mounted = s.base.mode == "movable"
            sol = self.results.selected() if self.results is not None else None
            sol_changes = solution_changes(s, sol) if sol is not None else []
            feas = feasibility_changes(s) if s.feasibility else []
            reasons = self.gate_reasons()
            key = (
                reasons,
                tuple(states.items()),
                locked,
                mounted,
                sol.label if sol else None,
                tuple(sol_changes),
                tuple(feas),
                len(s.results),
                bool(s.feasibility),
            )
            if key == self._last and not force:
                return
            self._last = key
            self._apply_gates(reasons)
            for i, step in enumerate(STEPS):
                if step not in self.headers:
                    continue
                md, ok, edit = self.headers[step]
                nxt = (
                    f" · next: **{i + 2} {TITLES[STEPS[i + 1]]}**"
                    if i + 1 < len(STEPS)
                    else " · next: **5 Optimize**"
                )
                md.content = f"**Step {i + 1} of {len(STEPS)}: {TITLES[step]}** · {ICON[states[step]]}{nxt}"
                ok.disabled = states[step] == "confirmed"
                if edit is not None:
                    ok.visible, edit.visible = not locked, locked
            if self.strip_md is not None:
                self.strip_md.content = strip_text(
                    states, len(s.results), sol_changes, feas, bool(s.feasibility)
                )
            if self.carrier is not None:
                self.carrier.set_locked(locked)
            if self.cell is not None:
                self.cell.set_mounted(mounted)
            if self.opt_md is not None:
                todo = [f"{i + 1} {TITLES[st]}" for i, st in enumerate(STEPS) if states[st] != "confirmed"]
                lines = [
                    f"⚠ Setup steps not confirmed: **{', '.join(todo)}**. "
                    "Results can change when you edit them."
                    if todo
                    else "✔ Setup confirmed."
                ]
                if feas:
                    lines.append(
                        f"⚠ The feasibility check is **out of date** ({', '.join(feas)} changed): "
                        "check again."
                    )
                self.opt_md.content = "  \n".join(lines)
            if self.results_md is not None:
                self.results_md.content = (
                    f"⚠ **Out of date:** {', '.join(sol_changes)} changed since this result was computed. "
                    "Re-run it (Optimize) to update; exports ask first."
                    if sol_changes
                    else ""
                )
        for fn in self.listeners:
            fn()

    def _log_setup(self, todo: list[str]) -> None:
        """Unconfirmed setup steps: the log panel's status line, and a message when they change."""
        from ui import log_panel

        if log_panel._panel is not None:
            log_panel._panel.set_setup(todo)
        if todo == self._todo:
            return
        if self._todo is not None or todo:
            if todo:
                log_panel.log(f"Setup steps not confirmed: **{', '.join(todo)}**.", "warn")
            else:
                log_panel.log("All setup steps confirmed.", "done")
        self._todo = todo

    def start_watch(self, period: float = 1.0) -> None:
        """Background refresh (real app only): edits anywhere update the step status and the
        out-of-date banners within about a second."""

        def loop() -> None:
            while True:
                time.sleep(period)
                for fn in (self.refresh, *self.hooks):
                    try:
                        fn()
                    except Exception:  # never let the watcher die (a mid-edit state may not hash)
                        pass

        threading.Thread(target=loop, daemon=True).start()
