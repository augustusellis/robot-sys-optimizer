"""Batch runner (plan Part 2, step B4): plan every pending job of a batch folder, one study each.

Per job (``run_job``, runs in the worker or in-process): materialise the study folder
(``ui.batch_materialize``), plan it with ``plan_job.station_job`` using the planner settings
snapshotted at *Add to queue* (``planner_snapshot`` -> ``BatchJob.planner``), save the
project, export CSV / userframes / HTML report of the best plan into ``<job>/export/``.

- **Failure isolation:** any error marks the job ``failed`` with a one-line ``error`` and the
  full traceback in ``<job>/error.txt``; the batch goes on with the next job.
- **Cancel:** the plans finished so far are saved, the job becomes ``partial`` (``cancelled``
  when none had finished) and the batch stops; the remaining jobs stay ``pending``.
- **Resume:** done jobs are skipped (``core.batch.pending_jobs``).
- **Memory:** ``run_batch`` (the GUI-side driver) runs each job in a fresh worker process: it
  owns a private ``ui.jobs.Runner`` (not the GUI's planner worker) and kills it after every
  job, so a large part never starves the next one and a crash fails only that job.

The manifest is written after every status change, one job entry at a time
(``core.batch.update_job``). Headless: ``python -m ui.batch_job <batch_dir> [--in-process]``.
This module does not import viser (the planner it calls does, inside the job).
"""

from __future__ import annotations

import threading
import time
import traceback
from pathlib import Path

from core import batch as bt
from ui.batch_planner import (  # noqa: F401 (re-exported: the batch panel imports them here)
    COLLISION_KEYS,
    SNAPSHOT_VERSION,
    collision_dict,
    planner_settings_from,
    planner_snapshot,
    settings_dict,
    spec_for,
)
from ui.plan_ids import give_id, give_ids
from ui.plan_ids import key as plan_key

# --- one job --------------------------------------------------------------------------------


def one_line(err: BaseException) -> str:
    summary = getattr(err, "summary", None)
    if summary:
        return str(summary).splitlines()[0]
    lines = str(err).strip().splitlines()
    msg = lines[0] if lines else ""
    if isinstance(err, ValueError) and msg:
        return msg  # our own checks: already a sentence
    return f"{type(err).__name__}: {msg}" if msg else type(err).__name__


def _write_error(job_dir: Path, err: BaseException) -> None:
    tb = getattr(err, "worker_traceback", "") or "".join(
        traceback.format_exception(type(err), err, err.__traceback__)
    )
    try:
        job_dir.mkdir(parents=True, exist_ok=True)
        (job_dir / "error.txt").write_text(f"{one_line(err)}\n\n{tb}", encoding="utf-8")
    except OSError:
        pass  # the one-line error in the manifest still says what happened


def _collision_globals():
    from core.collision_config import current

    return current()


def _restore_collision(saved) -> None:
    from core.collision_config import use

    use(saved)


class _Plans:
    """StationsPanel's streamed bookkeeping on a headless session."""

    def __init__(self, s, quick: bool, job_id: str, emit, save) -> None:
        from ui.staleness import stamp_for

        self.s, self.quick, self.job_id, self.emit, self.save = s, quick, job_id, emit, save
        self.new: set[str] = set()
        self.stamp = stamp_for(s, "plan")

    def _sort(self, plans) -> list:
        return sorted(plans, key=lambda x: (bool(x.violations), x.cost))

    def streamed(self, sol, res) -> None:
        s = self.s
        if self.quick and not sol.label.startswith("Quick: "):
            sol.label = f"Quick: {sol.label}"
        sol.planner = sol.planner or ("quick" if self.quick else "cost")
        same = [r for r in s.results if len(r.stations) == len(sol.stations) and r.tool_values is None]
        for old in same:  # a refined plan replaces the quick one with the same number of stations
            s.result_points.pop(plan_key(old), None)
            s.stamps.pop(plan_key(old), None)
        keep = [r for r in s.results if not any(r is x for x in same)]
        k = give_id(sol, keep)  # v12 #8: plans are keyed by id, as in the Station planner panel
        s.stamps[k] = self.stamp
        s.result_points[k] = res
        s.results = self._sort([*keep, sol])
        self.new.add(k)
        self.save()  # a crash or kill later keeps the plans finished so far

    def replace(self, plans) -> None:
        s = self.s
        for r in [r for r in s.results if r.tool_values is None]:
            s.result_points.pop(plan_key(r), None)
            s.stamps.pop(plan_key(r), None)
        tools = [r for r in s.results if r.tool_values is not None]
        give_ids([sol for sol, _ in plans], tools)  # v12 #8
        for sol, res in plans:
            s.stamps[plan_key(sol)] = self.stamp
            s.result_points[plan_key(sol)] = res
            self.new.add(plan_key(sol))
        s.results = [*(sol for sol, _ in plans), *tools]
        self.save()

    def __call__(self, kind: str, data) -> None:
        if kind == "progress":
            self.emit("progress", data)
        elif kind == "solution":
            self.streamed(*data)
        elif kind == "replace":
            self.replace(data)
        elif kind == "max_manip":
            self.s.max_manip = float(data)
        elif kind == "warn":
            self.emit("warn", f"{self.job_id}: {data}")


def run_job(manifest: bt.BatchManifest, job: bt.BatchJob, batch_dir, emit, cancelled) -> None:
    """Plan one job (in this process). Never raises except ``CancelledError`` (job partial or
    cancelled, plans so far saved); every other error marks the job failed."""
    from core.feasibility import CancelledError

    batch_dir = Path(batch_dir)
    path = batch_dir / bt.MANIFEST_NAME
    job_dir = batch_dir / job.id

    def save() -> None:
        bt.update_job(path, job, fallback=manifest)

    (job_dir / "error.txt").unlink(missing_ok=True)
    job.status, job.error, job.summary, job.run_s = "running", "", {}, 0.0
    save()
    t0 = time.monotonic()
    box: dict = {}
    saved = _collision_globals()  # station_job sets them; in-process runs must not keep them
    try:
        _run(manifest, job, batch_dir, emit, cancelled, box, t0)
        job.status = "done"
    except CancelledError:
        n = _keep(box)
        job.status = "partial" if n else "cancelled"
        job.error = (
            f"cancelled; {n} plan(s) finished before and saved" if n else "cancelled before a plan finished"
        )
        raise
    except Exception as err:  # isolation: this job fails, the batch goes on
        _keep(box)
        job.status, job.error = "failed", one_line(err)
        _write_error(job_dir, err)
    finally:
        job.run_s = round(time.monotonic() - t0, 1)
        _restore_collision(saved)
        save()


def _keep(box: dict) -> int:
    """Save the plans finished so far (cancel / failure); returns how many."""
    plans = box.get("plans")
    if plans is None or not plans.new:
        return 0
    try:
        plans.save()
    except Exception:  # noqa: BLE001 - the job's status still says what happened
        traceback.print_exc()
    return len(plans.new)


def _run(manifest, job, batch_dir: Path, emit, cancelled, box: dict, t0: float) -> None:
    from core.feasibility import CancelledError
    from ui import plan_job, report_make
    from ui.batch_materialize import materialize
    from ui.session import Session, load_project, save_project

    who = job.name or job.id
    emit("progress", (f"{who}: generating targets", 0.0))
    cell = manifest  # cell_project may be relative to the batch folder
    if manifest.cell_project:
        cell = manifest.model_copy(update={"cell_project": str(bt.cell_path(manifest, batch_dir))})
    out = materialize(cell, job, batch_dir)
    s = Session()
    load_project(s, out)
    spec = spec_for(job.planner, s)
    cfg = spec.pop("collision", None)  # v12 #10: the job's collision settings, one value
    if cfg is not None:
        from core.collision_config import use

        use(cfg)  # run_job puts the process's own settings back
    plans = _Plans(s, job.planner.get("mode") == "quick", job.id, emit, lambda: save_project(s, out))
    box["plans"] = plans
    result = plan_job.station_job({"session": s, "spec": spec}, plans, cancelled)
    best = result.get("best")
    if best is not None:  # the streamed object of that label (the run time goes on it)
        best = next((r for r in s.results if r.label in (best.label, f"Quick: {best.label}")), best)
    for r in s.results:
        if plan_key(r) in plans.new and r.run_s is None and r.tool_values is None:
            r.run_s = round(time.monotonic() - t0, 1)
    save_project(s, out)
    if cancelled():  # the planner returned its best so far after Cancel
        raise CancelledError("cancelled")
    if best is None:
        raise ValueError(f"{who}: no station plan found ({len(s.targets.points)} targets)")
    emit("progress", (f"{who}: writing the exports", 1.0))
    files = report_make.export_all(s, best, out / "export")
    exports, failed = {}, []
    for name, v in files.items():
        if isinstance(v, Path):
            exports[name] = v.relative_to(batch_dir).as_posix()
        else:
            exports[name] = f"error: {v}"
            failed.append(f"{name} ({str(v).splitlines()[0] if str(v) else 'failed'})")
    total = len(s.targets.points)
    report = exports.get("report.html", "")
    job.summary = {
        "label": best.label,
        "plans": len(plans.new),
        "stations": len(best.stations),
        "reached": total - len(best.unreached),
        "total": total,
        "cost": float(best.cost),
        "violations": len(best.violations),
        "cycle_s": best.cycle_s,
        "report": "" if report.startswith("error") else report,
        "exports": exports,
    }
    if failed:  # the plan is done; say which files are missing
        job.error = "export failed: " + "; ".join(failed)


def job_row(job: bt.BatchJob, index: int = 0, n: int = 0) -> dict:
    """The ``job`` message: what the batch panel lists per finished job."""
    return {
        "id": job.id,
        "name": job.name,
        "status": job.status,
        "summary": dict(job.summary),
        "error": job.error,
        "run_s": job.run_s,
        "index": index,
        "n": n,
    }


def _scaled(emit, i: int, n: int, who: str):
    def inner(kind: str, data) -> None:
        if kind == "progress":
            msg, f = data
            emit("progress", (f"job {i + 1}/{n} ({who}): {msg}", (i + float(f)) / max(n, 1)))
        else:
            emit(kind, data)

    return inner


# --- worker entry points --------------------------------------------------------------------


def batch_one(payload: dict, emit, cancelled) -> dict:
    """Worker entry (one job per fresh worker): {"batch_dir", "job_id"} -> the job as a dict."""
    bdir = Path(payload["batch_dir"])
    m = bt.load_manifest(bdir / bt.MANIFEST_NAME)
    job = next((j for j in m.jobs if j.id == payload["job_id"]), None)
    if job is None:
        raise ValueError(f"job {payload['job_id']} is no longer in the batch")
    if job.status in ("pending", "running"):
        run_job(m, job, bdir, emit, cancelled)
    return job.model_dump()


def batch_job(payload: dict, emit, cancelled) -> list[dict]:
    """Worker entry (the whole batch in one process, like compare_job): {"batch_dir"}. Emits
    ``progress`` (job i/n + the planner's) and one ``job`` message per finished job."""
    from core.feasibility import CancelledError

    bdir = Path(payload["batch_dir"])
    path = bdir / bt.MANIFEST_NAME
    m = bt.load_manifest(path)
    todo = bt.pending_jobs(m)
    bt.save_manifest(m, path)
    rows: list[dict] = []
    try:
        for i, job in enumerate(todo):
            if cancelled():
                raise CancelledError("batch cancelled")
            try:
                run_job(m, job, bdir, _scaled(emit, i, len(todo), job.name or job.id), cancelled)
            finally:
                rows.append(job_row(job, i, len(todo)))
                emit("job", rows[-1])
    finally:
        bt.write_summary(bt.load_manifest(path), bdir)
    return rows


# --- GUI-side driver ------------------------------------------------------------------------


def _mark_crashed(path: Path, job_id: str, err: BaseException, status: str = "failed") -> None:
    """The worker died or was ended: the job (still 'running' on disk) gets the reason."""
    m = bt.load_manifest(path)
    job = next((j for j in m.jobs if j.id == job_id), None)
    if job is None or job.status not in ("running", "pending"):
        return
    job.status, job.error = status, one_line(err)
    if status == "failed":
        _write_error(path.parent / job_id, err)
    bt.update_job(path, job)


def drive(batch_dir, on_msg, cancelled=lambda: False, in_process: bool = False) -> dict:
    """Run every pending job (blocking): in-process, or each in a fresh worker process.
    Messages: ``progress`` (text, 0..1 over the batch), ``warn``, ``job`` (job_row).
    Returns {"status": done | cancelled, "jobs": [job_row ...]}."""
    from core.feasibility import CancelledError
    from ui import jobs

    bdir = Path(batch_dir)
    path = bdir / bt.MANIFEST_NAME
    m = bt.load_manifest(path)
    todo = [(j.id, j.name or j.id) for j in bt.pending_jobs(m)]
    bt.save_manifest(m, path)  # interrupted 'running' jobs are pending again
    runner = None if in_process else jobs.Runner()  # private: the GUI's planner worker stays
    rows, status = [], "done"
    try:
        for i, (jid, who) in enumerate(todo):
            if cancelled():
                status = "cancelled"
                break
            payload = {"batch_dir": str(bdir), "job_id": jid}
            sub = _scaled(on_msg, i, len(todo), who)
            try:
                if runner is None:
                    batch_one(payload, sub, cancelled)
                else:
                    runner.run("ui.batch_job:batch_one", payload, sub, cancelled)
            except CancelledError:
                status = "cancelled"
            except jobs.JobCancelledError as err:  # ended after Cancel: plans streamed so far are saved
                status = "cancelled"
                _mark_crashed(path, jid, err, "partial")
            except Exception as err:  # the worker died (memory?) or the job could not start
                _mark_crashed(path, jid, err)
            finally:
                if runner is not None:
                    runner.kill()  # a fresh process for the next job frees this one's memory
            job = next((j for j in bt.load_manifest(path).jobs if j.id == jid), None)
            if job is not None:
                rows.append(job_row(job, i, len(todo)))
                on_msg("job", rows[-1])
            if status == "cancelled":
                break
    finally:
        if runner is not None:
            runner.kill()
        try:
            bt.write_summary(bt.load_manifest(path), bdir)
        except Exception as err:  # noqa: BLE001
            on_msg("warn", f"batch summary not written ({one_line(err)})")
    return {"status": status, "jobs": rows}


def run_batch(batch_dir, on_msg, cancelled=lambda: False, in_process: bool = False) -> threading.Thread:
    """Start ``drive`` in a daemon thread. It ends with a ``finished`` message:
    {"status": done | cancelled | failed, "jobs": [...], "error": one line}."""

    def work() -> None:
        try:
            out = drive(batch_dir, on_msg, cancelled, in_process)
            on_msg("finished", {**out, "error": ""})
        except Exception as err:  # e.g. an unreadable batch.json: say so
            on_msg("finished", {"status": "failed", "jobs": [], "error": one_line(err)})

    t = threading.Thread(target=work, daemon=True, name="batch-runner")
    t.start()
    return t


# --- CLI ------------------------------------------------------------------------------------


def _line(row: dict) -> str:
    s = row["summary"]
    what = (
        f"{s['stations']} station(s), {s['reached']}/{s['total']} reached, cost {s['cost']:.2f}"
        if s.get("stations")
        else ""
    )
    tail = f"  {row['error']}" if row["error"] else ""
    return f"{row['id']:<10} {row['status']:<9} {what} ({row['run_s']:.0f} s){tail}"


def main(argv=None) -> int:
    import argparse

    ap = argparse.ArgumentParser(description="Run the pending jobs of a batch folder (batch.json).")
    ap.add_argument("batch_dir", type=Path)
    ap.add_argument("--in-process", action="store_true", help="no worker process per job")
    a = ap.parse_args(argv)
    cancel = threading.Event()
    done: dict = {}

    def on_msg(kind: str, data) -> None:
        if kind == "job":
            print(_line(data), flush=True)
        elif kind == "warn":
            print(f"warning: {data}", flush=True)
        elif kind == "finished":
            done.update(data)

    t = run_batch(a.batch_dir, on_msg, cancel.is_set, a.in_process)
    while t.is_alive():
        try:
            t.join(0.5)
        except KeyboardInterrupt:
            print("cancelling: the current job keeps its finished plans", flush=True)
            cancel.set()
    if done.get("error"):
        print(f"batch failed: {done['error']}", flush=True)
    rows = done.get("jobs", [])
    bad = [r for r in rows if r["status"] != "done"]
    print(f"batch {done.get('status', 'failed')}: {len(rows) - len(bad)}/{len(rows)} job(s) done", flush=True)
    return 0 if done.get("status") == "done" and not bad else 1


if __name__ == "__main__":
    raise SystemExit(main())
