"""Carrying a tool's contact settings over to a new URDF (v10 #1).

A TCP's ``contact_links`` name URDF links and its ``contact_faces`` index the faces of a link's
visual mesh (as yourdfpy loads it; core.tool.ToolModel). When the package builder rewrites a tool,
those links / faces must still exist: links are kept when the new URDF has a link of that name with
geometry, faces when that link's mesh has the same number of faces as before. Everything else is
dropped and reported.
"""

from __future__ import annotations

from pathlib import Path

from core.schema import TCP, EndEffector


def link_faces(ee: EndEffector, urdf_path: Path) -> dict[str, int]:
    """Link name -> face count of its visual mesh, for the links with geometry."""
    from core.tool import ToolModel

    return {hl.name: len(hl.mesh.faces) for hl in ToolModel(ee, urdf_path).links}


def carry_contacts(
    tcps: list[TCP], old: dict[str, int] | None, new: dict[str, int]
) -> tuple[list[TCP], list[str]]:
    """(the TCPs with the contacts that still map, what was dropped). ``old``: face counts of the
    tool the contacts were made on (None: unknown, faces are kept when their ids fit)."""
    out, lost = [], []
    for t in tcps:
        links = [n for n in t.contact_links if n in new]
        lost += [
            f"TCP {t.name}: contact part '{n}' is not in the new tool"
            for n in t.contact_links
            if n not in new
        ]
        faces = {}
        for n, ids in t.contact_faces.items():
            if n not in new:
                lost.append(f"TCP {t.name}: contact faces on '{n}': the new tool has no such part")
            elif (old is not None and old.get(n) != new[n]) or (ids and max(ids) >= new[n]):
                lost.append(f"TCP {t.name}: contact faces on '{n}': its mesh changed, pick them again")
            else:
                faces[n] = ids
        out.append(t.model_copy(update={"contact_links": links, "contact_faces": faces}))
    return out, lost
