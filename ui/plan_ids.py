"""Plans are keyed by an id, not their label (v12 #8).

``Solution.label`` ("2 rail stations: 25/25 reached") is for people. It was also the key of a plan's
joint solutions (``Session.result_points``), its setup stamp (``Session.stamps``), the compare stash
and the built paths, and labels are unique only by convention: two planners writing the same label
overwrote each other's joint solutions. ``give_id`` sets ``Solution.id`` when a plan enters the
session's results; ``key`` is what every dict is keyed by (the label for plans saved before v12,
which have no id: their files keep working unchanged).
"""

from __future__ import annotations

import uuid


def key(sol) -> str:
    """The dict key of a plan's joint solutions, stamp and paths."""
    return sol.id or sol.label


def new_id() -> str:
    return uuid.uuid4().hex[:12]


def give_id(sol, others=()) -> str:
    """Give ``sol`` an id (in place) unless it has one that no other plan in ``others`` uses; a copy
    of a plan (``model_copy``) shares its source's id and gets its own here. Returns the key."""
    taken = {o.id for o in others if o is not sol and o.id}
    if not sol.id or sol.id in taken:
        sol.id = new_id()
    return sol.id


def give_ids(sols, others=()) -> None:
    """``give_id`` for several new plans: unique among themselves and against ``others``."""
    seen = list(others)
    for sol in sols:
        give_id(sol, seen)
        seen.append(sol)
