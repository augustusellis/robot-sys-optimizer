"""The summary of a batch run (v13 #5): ``summary.json`` (read by the Batch results viewer),
``summary.csv`` and ``summary.html`` / ``summary.pdf`` in the output folder. Written again after
every part, so a cancelled or crashed run still lists the parts that finished."""

from __future__ import annotations

import csv
import html
import json
import time
from pathlib import Path

COLUMNS = (
    ("part", "Part"),
    ("status", "Status"),
    ("reached", "Reached"),
    ("points", "Points"),
    ("stops", "Stops"),
    ("cycle_s", "Cycle time (s)"),
    ("cost", "Cost"),
    ("plan", "Plan"),
    ("run_s", "Run time (s)"),
    ("reason", "Note"),
)
SUMMARY = "summary.json"
META = (
    ("project", "Project"),
    ("parts_folder", "Parts folder"),
    ("robot", "Robot"),
    ("tool", "End effector"),
    ("axis", "External axis"),
    ("workers", "Ran"),
    ("started", "Started"),
    ("updated", "Updated"),
)


def write(out_dir: Path, meta: dict, rows: list[dict], pdf: bool = False) -> Path:
    """Write the summary files; ``pdf`` also prints summary.pdf (slow: only at the end)."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    data = {**meta, "updated": time.strftime("%Y-%m-%d %H:%M:%S"), "rows": rows}
    tmp = out_dir / (SUMMARY + ".tmp")
    tmp.write_text(json.dumps(data, indent=2), encoding="utf-8")
    tmp.replace(out_dir / SUMMARY)
    with open(out_dir / "summary.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow([*(k for k, _ in COLUMNS), "folder", "report"])
        for r in rows:
            w.writerow(
                [
                    *("" if r.get(k) is None else r.get(k) for k, _ in COLUMNS),
                    r.get("folder", ""),
                    r.get("report", ""),
                ]
            )
    page = out_dir / "summary.html"
    page.write_text(to_html(data, out_dir), encoding="utf-8")
    if pdf:
        from core.report import html_to_pdf

        html_to_pdf(page, out_dir / "summary.pdf")
    return out_dir / SUMMARY


def read(out_dir: Path) -> dict:
    """The summary of a batch output folder (ValueError when there is none)."""
    f = Path(out_dir) / SUMMARY
    try:
        data = json.loads(f.read_text(encoding="utf-8"))
    except (OSError, ValueError) as err:
        raise ValueError(f"no batch summary in {out_dir}") from err
    if not isinstance(data, dict) or not isinstance(data.get("rows"), list):
        raise ValueError(f"{f} is not a batch summary")
    return data


def _cell(r: dict, k: str) -> str:
    v = r.get(k)
    return "" if v is None else html.escape(str(v))


def _link(r: dict, out_dir: Path) -> str:
    rep = r.get("report")
    if not rep:
        return ""
    p = Path(rep)
    try:
        href = p.relative_to(out_dir).as_posix()
    except ValueError:
        href = p.as_uri()
    return f'<a href="{html.escape(href)}">report</a>'


def to_html(data: dict, out_dir: Path) -> str:
    rows = data.get("rows", [])
    head = "".join(f"<th>{t}</th>" for _, t in COLUMNS) + "<th></th>"
    body = "".join(
        f'<tr class="{_cell(r, "status")}">'
        + "".join(f"<td>{_cell(r, k)}</td>" for k, _ in COLUMNS)
        + f"<td>{_link(r, out_dir)}</td></tr>"
        for r in rows
    )
    done = sum(r.get("status") == "done" for r in rows)
    meta = "".join(
        f"<li><b>{t}</b>: {html.escape(str(data[k]))}</li>" for k, t in META if data.get(k) not in (None, "")
    )
    return (
        "<!doctype html><html><head><meta charset='utf-8'><title>Batch summary</title><style>"
        "body{font-family:Segoe UI,Arial,sans-serif;margin:24px;color:#222}"
        "table{border-collapse:collapse;font-size:13px}th,td{border:1px solid #ccc;padding:4px 8px}"
        "th{background:#f0f0f0}tr.failed td{background:#fde2e2}tr.cancelled td{color:#888}"
        "</style></head><body>"
        f"<h1>Batch summary</h1><p>{done} of {len(rows)} parts planned.</p><ul>{meta}</ul>"
        f"<table><tr>{head}</tr>{body}</table></body></html>"
    )
