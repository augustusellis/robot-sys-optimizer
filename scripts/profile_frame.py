"""Time a Gantry placement run on a shipped example (v10 task 7).

    python scripts/profile_frame.py [example] [--top N] [--workers N]

Loads the example (default gantry_overhead) in a headless app, confirms the setup and runs
Optimize -> Gantry placement in this process with a check pool (``--workers``, default
opt.parallel.default_workers()). Prints the time of each phase (reach map, frame scoring, each
frame's full plan) from the job's progress messages, and the total. Uses a temporary workspace.
"""

import argparse
import os
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
os.environ.setdefault("RSO_WORKSPACE", tempfile.mkdtemp())

import viser  # noqa: E402

from app import build_app  # noqa: E402


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("example", nargs="?", default="gantry_overhead")
    ap.add_argument("--top", type=int, default=2)
    ap.add_argument("--workers", type=int, default=None)
    a = ap.parse_args()
    from opt.parallel import default_workers, shutdown_all
    from tests.helpers import confirm_all
    from ui import jobs

    jobs.CHECK_WORKERS = a.workers if a.workers is not None else default_workers()
    server = viser.ViserServer(host="127.0.0.1", port=8793, verbose=False)
    try:
        app = build_app(server)
        assert app["project"].load_example(a.example), a.example
        confirm_all(app["session"])
        frame = app["frame"]
        frame.refresh()
        frame.top.value = a.top
        marks: list[tuple[float, str]] = []
        t0 = time.monotonic()
        orig = frame.status

        class Status:  # records each progress message with its time
            def __setattr__(self, k, v):
                if k == "content":
                    marks.append((time.monotonic() - t0, str(v)))
                setattr(orig, k, v)

        frame.status = Status()
        frame.start(block=True)
        total = time.monotonic() - t0
        frame.status = orig
        last, phase = 0.0, None
        for t, msg in marks:
            key = msg.split(":")[0].strip("_ ")
            if key != phase:
                if phase is not None:
                    print(f"{last:7.1f} s  {phase}")
                phase, last = key, t
        print(
            f"total {total:.1f} s with {jobs.CHECK_WORKERS} check workers; {len(frame.rows)} frame(s) planned"
        )
        for r in frame.rows:
            print(f"  {r['frame']}: {r['stations']} stations, {r['reached']} reached, cost {r['cost']}")
    finally:
        shutdown_all()
        server.stop()


if __name__ == "__main__":
    main()
