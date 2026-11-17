"""v13 #1: the gantry frame check sweeps the carriage too: at every X the carriage (and its drop /
lift stage) needs some Y where it clears the part, or the bridge cannot carry it past."""

import json
from pathlib import Path

import numpy as np
import trimesh

from core.carrier import Carrier
from core.schema import Base
from opt.frame_place import sweep_clear

REPO = Path(__file__).resolve().parents[1]


def _overhead() -> Carrier:
    base = json.loads((REPO / "examples/gantry_overhead/project.json").read_text())["base"]
    return Carrier(Base.model_validate(base))


def _wall(width_y: float, height: float) -> trimesh.Trimesh:
    """A 0.3 m thick wall across x = 0, ``width_y`` wide, standing on the floor."""
    return trimesh.creation.box(
        (0.3, width_y, height), trimesh.transformations.translation_matrix((0, 0, height / 2))
    )


def test_carriage_blocked_across_the_whole_y_travel_is_refused():
    # bridge bottom at 4.4 m, drop bottom at 3.55 m: a 3.9 m wall clears the bridge, not the drop
    why = sweep_clear(_overhead(), _wall(6.0, 3.9), np.eye(4))
    assert why.startswith("the carriage cannot pass the part")


def test_carriage_that_can_go_around_the_part_is_fine():
    c = _overhead()
    assert sweep_clear(c, _wall(1.0, 3.9), np.eye(4)) == ""  # around it at y = +-1.4 m
    assert sweep_clear(c, _wall(6.0, 3.0), np.eye(4)) == ""  # under the drop
    assert sweep_clear(c, _wall(6.0, 4.3), np.eye(4)) == "the bridge would travel through the part"
