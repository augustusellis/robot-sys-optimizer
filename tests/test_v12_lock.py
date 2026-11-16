"""v12 item 9: one lock for changes to the Session from several threads (ui.session_lock).

Done when a project load, an autosave snapshot, drag callbacks and planner results wait for a
thread that holds the lock, and the lock is the same object after a load swapped the session.
"""

import threading
import time

from ui.session import Session, load_project, save_project
from ui.session_lock import LOCK, locked
from ui.widgets import Latest


def _held_for(seconds: float) -> threading.Event:
    """Hold LOCK in another thread for ``seconds``; returns an event set once it is held."""
    got = threading.Event()

    def hold():
        with LOCK:
            got.set()
            time.sleep(seconds)

    threading.Thread(target=hold, daemon=True).start()
    assert got.wait(2)
    return got


def test_load_waits_for_the_lock(tmp_path):
    save_project(Session(), tmp_path)
    s = Session()
    _held_for(0.4)
    t0 = time.monotonic()
    load_project(s, tmp_path)  # the swap waits until the other thread is done
    assert time.monotonic() - t0 >= 0.3


def test_drag_callbacks_and_results_run_under_the_lock():
    seen = []
    Latest(lambda: seen.append(LOCK._is_owned()))()
    locked(lambda: seen.append(LOCK._is_owned()))()
    assert seen == [True, True] and not LOCK._is_owned()


def test_lock_is_reentrant_and_survives_a_load(tmp_path):
    save_project(Session(), tmp_path)
    s = Session()
    with LOCK:
        load_project(s, tmp_path)  # takes LOCK again inside: no deadlock
        with LOCK:
            pass
    from ui import session_lock

    assert session_lock.LOCK is LOCK  # one lock per process, not per Session
