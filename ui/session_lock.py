"""One lock for changes to the app's Session from more than one thread (v12 #9).

The Session (ui.session) is read and written by viser's callback thread pool, the 1 s watcher
(ui.workflow), the autosave thread (ui.project_panel) and the threads of the planners and checks.
The planners work on a snapshot (ui.plan_job.payload_for), but the moments where their results go
into the session, the watcher's refresh and hooks, an autosave snapshot, a project load (which
swaps the whole session) and drag callbacks (ui.widgets.Latest) all take ``LOCK``, so none of them
sees a half-made change of another.

Rules: take it around a short change (assign the results, swap the session), never around a long
computation, and never while waiting for another thread (``join``): a planner thread delivering
its result needs it. It is re-entrant, so nested holders in one thread are fine. It is one lock
per process, not per Session object, because loading swaps the session's fields in place.
"""

from __future__ import annotations

import functools
import threading

LOCK = threading.RLock()


def locked(fn):
    """Decorator: run ``fn`` holding ``LOCK``."""

    @functools.wraps(fn)
    def run(*args, **kwargs):
        with LOCK:
            return fn(*args, **kwargs)

    return run
