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


# --- v10 #11: contacts picked in the package builder (urdf_package.Frame, by part name) --------


def frames_with_contacts(frames: list, tcps: list[TCP], parts: list) -> list:
    """The builder's TCP frames with the contacts of the project tool's TCPs of the same name
    (link -> the part written to that link)."""
    from core.urdf_package import safe, tool_link

    part_of = {tool_link(x): x.name for x in parts}
    by_name = {safe(t.name): t for t in tcps}
    out = []
    for f in frames:
        t = by_name.get(f.name)
        if t is None:
            out.append(f)
            continue
        out.append(
            f.model_copy(
                update={
                    "contact_parts": [part_of[n] for n in t.contact_links if n in part_of],
                    "contact_faces": {
                        part_of[n]: list(ids) for n, ids in t.contact_faces.items() if n in part_of
                    },
                    "min_radius_mm": t.min_radius_mm,  # v10 T12
                    "radius_applies": t.radius_applies,
                    **{k: getattr(t, k) for k in ("travel", "scan_width_mm", "overlap_pct", "overrun_mm")},
                }
            )
        )
    return out


def rename_part(frames: list, old: str, new: str) -> list:
    """Contacts follow a renamed part."""
    return [
        f.model_copy(
            update={
                "contact_parts": [new if n == old else n for n in f.contact_parts],
                "contact_faces": {(new if n == old else n): ids for n, ids in f.contact_faces.items()},
            }
        )
        for f in frames
    ]


def drop_part(frames: list, name: str, faces_only: bool = False) -> list:
    """Contacts without ``name`` (removed), or without its faces only (its mesh changed)."""
    return [
        f.model_copy(
            update={
                "contact_parts": f.contact_parts if faces_only else [n for n in f.contact_parts if n != name],
                "contact_faces": {n: ids for n, ids in f.contact_faces.items() if n != name},
            }
        )
        for f in frames
    ]
