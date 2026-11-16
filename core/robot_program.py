"""Robot programs from built paths (v10 T21): URScript (UR), TP / .ls (FANUC) and RAPID (ABB).

Surface samples become linear moves of the TCP in a **user frame on the part**: a sample's TCP pose
is stored in the part frame (core.paths), so the points are the same whatever stop the base is at;
each stop gets its user frame (the part seen from that stop's base, core.userframe). Joint moves
(home, retreats, configuration changes, planned moves around the part) become joint moves with the
planned joints. The tool definition is the TCP in the flange frame.

Between stops (stop and go) the program says where the carrier goes: a comment plus a call of a
``rso_carrier`` routine (UR) / a carrier comment and ``WAIT`` placeholder (FANUC) / a ``rso_carrier``
procedure (RAPID) the integrator fills in. Coordinated paths (core.coordinated) use one user frame
(the part seen from the base at carrier zero) and give each point the carrier values as extended
axes (FANUC ``E1..``, RAPID ``extax``, mm); URScript has no external axes: there every point is a
base-frame pose and each move is preceded by ``rso_carrier(...)`` for an external controller.

Configuration data (FANUC CONFIG, ABB confdata) is derived from the planned joints; RAPID programs
also switch ``ConfL`` off so the controller takes the nearest configuration if they differ.
Speeds come from core.cycle_time.CycleSettings. Not run on a controller here: check in a simulator
(URSim, ROBOGUIDE, RobotStudio) before running on a robot.
"""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from core.cycle_time import CycleSettings
from core.userframe import base_to_part

FORMATS = {"urscript": ".script", "fanuc": ".ls", "rapid": ".mod"}


def default_format(robot_name: str, convention: str) -> str | None:
    """The robot's own program language, or None when it is not known (an uploaded URDF, a made-up
    arm): v12 #3, URScript was guessed for those and is wrong for anything that is not a UR."""
    if convention == "ur":
        return "urscript"
    if convention == "fanuc":
        return "fanuc"
    return "rapid" if robot_name.lower().startswith(("gofa", "powa", "irb")) else None


def _wpr(R: np.ndarray) -> tuple[float, float, float]:
    return tuple(float(v) for v in Rotation.from_matrix(R).as_euler("xyz", degrees=True))


def _quat(R: np.ndarray) -> tuple[float, float, float, float]:
    x, y, z, w = Rotation.from_matrix(R).as_quat()
    return float(w), float(x), float(y), float(z)


def _rotvec(R: np.ndarray) -> tuple[float, float, float]:
    return tuple(float(v) for v in Rotation.from_matrix(R).as_rotvec())


class _Step:
    """One program line: kind "lin" (pose in the user frame), "joint" (q) or "carrier" (values)."""

    def __init__(self, kind, frame=0, T=None, q=None, carrier=None, issue="", note=""):
        self.kind, self.frame, self.T, self.q = kind, frame, T, q
        self.carrier, self.issue, self.note = carrier, issue, note


def steps(plan, carrier=None, T_world_part=None) -> tuple[list[_Step], list[np.ndarray], bool]:
    """(program steps, user frames (base -> part, one per stop or one), coordinated)."""
    # coordinated (core.coordinated): one stop whose surface samples carry changing carrier values
    surf = [p for p in plan.samples if p.kind in ("", "lift", "retreat")]
    coord = (
        len(plan.stations) == 1
        and bool(surf)
        and all(p.carrier is not None for p in surf)
        and any(p.carrier != surf[0].carrier for p in surf)
    )
    frames: list[np.ndarray] = []
    if coord:
        zero = carrier.clip({k: 0.0 for k in carrier.dofs}) if carrier is not None else {}
        T0 = carrier.T_world_base(zero) if carrier is not None else plan.stations[0].T_world_base
        frames = [base_to_part(T0, T_world_part)]
    else:
        frames = [base_to_part(st.T_world_base, T_world_part) for st in plan.stations]
    out: list[_Step] = []
    for k, st in enumerate(plan.stations):
        fi = 0 if coord else k
        if not coord and k > 0:
            out.append(_Step("carrier", fi, carrier=dict(st.carrier or {}), note=f"stop {k + 1}"))
        for p in st.samples:
            if p.kind == "carrier":
                continue  # the arm is parked while the carrier drives (the stop's carrier line)
            if p.q is None:  # v11 #12: no joint solution: said in the program, not dropped silently
                out.append(_Step("skip", fi, issue=p.issue or "unreachable", note=f"point {p.a} -> {p.b}"))
                continue
            if p.kind == "home":
                out.append(_Step("joint", fi, q=np.asarray(p.q, float), carrier=p.carrier, issue=p.issue))
                continue
            out.append(
                _Step("lin", fi, T=p.T_part_tcp, q=np.asarray(p.q, float), carrier=p.carrier, issue=p.issue)
            )
    return out, frames, coord


def _fanuc_config(q: np.ndarray) -> str:
    """CONFIG string from the joints: flip (wrist), up / down (elbow), front / back, turns 4 / 6."""
    deg = np.degrees(q)
    flip = "F" if deg[4] < 0 else "N"
    up = "U" if deg[2] > -90 else "D"
    front = "T" if abs(((deg[0] + 180) % 360) - 180) < 90 else "B"
    t = [int(np.floor((d + 180) / 360)) for d in (deg[0], deg[3], deg[5])]
    return f"'{flip} {up} {front}, {t[0]}, {t[1]}, {t[2]}'"


def _abb_conf(q: np.ndarray) -> str:
    deg = np.degrees(q)
    cf = [int(np.floor(deg[i] / 90.0)) for i in (0, 3, 5)]
    cfx = (1 if deg[4] < 0 else 0) + (2 if deg[2] < -90 else 0) + (4 if abs(deg[0]) > 90 else 0)
    return f"[{cf[0]},{cf[1]},{cf[2]},{cfx}]"


def program(
    fmt: str,
    plan,
    T_flange_tcp: np.ndarray,
    T_world_part: np.ndarray,
    carrier=None,
    cs: CycleSettings | None = None,
    name: str = "rso_path",
) -> str:
    """The program text for ``fmt`` (FORMATS). One tool definition per program: a plan whose paths
    use several TCPs is refused (v11 #12; the points would be written for the wrong TCP)."""
    cs = cs or CycleSettings()
    used = sorted({p.tcp for p in plan.samples if p.tcp})
    if len(used) > 1:
        raise ValueError(
            f"the paths use {len(used)} TCPs ({', '.join(used)}): one program per TCP is not supported yet"
        )
    st, frames, coord = steps(plan, carrier, T_world_part)
    if fmt == "urscript":
        return _urscript(st, frames, coord, T_flange_tcp, cs, name)
    if fmt == "fanuc":
        return _fanuc(st, frames, coord, T_flange_tcp, cs, name)
    if fmt == "rapid":
        return _rapid(st, frames, coord, T_flange_tcp, cs, name)
    raise ValueError(f"unknown program format {fmt!r}; have {tuple(FORMATS)}")


def _ur_pose(T: np.ndarray) -> str:
    return "p[" + ", ".join(f"{v:.6f}" for v in (*T[:3, 3], *_rotvec(T[:3, :3]))) + "]"


def _urscript(st, frames, coord, tool, cs, name) -> str:
    lines = [
        f"# {name}: generated by robot-sys-optimizer (v10). Check in URSim before running on a robot.",
        "# TCP poses are in the part's user frame (uf_k = the part seen from the base at stop k).",
    ]
    if coord:
        lines.append(
            "# Coordinated carrier: URScript has no external axes; rso_carrier() is called before every"
        )
        lines.append(
            "# move with the carrier values (m / rad) for an external axis controller, poses are base-frame."
        )
    lines += [f"def {name}():", f"  set_tcp({_ur_pose(tool)})"]
    for k, F in enumerate(frames):
        lines.append(f"  uf_{k + 1} = {_ur_pose(F)}")
    lines += [
        "  def rso_carrier(values):",
        "    # the integrator moves the rail / gantry / mobile base here (blocking until in position)",
        '    textmsg("carrier to ", values)',
        "  end",
    ]
    vj, aj = cs.joint_speed, cs.joint_acc
    for s in st:
        if s.kind == "skip":
            lines.append(f"  # skipped ({s.note}): no joint solution, {s.issue}")
            continue
        if s.issue:
            lines.append(f"  # flagged in the paths: {s.issue}")
        if s.kind == "carrier":
            lines.append(f"  # {s.note}: carrier {_vals(s.carrier, 'm')}")
            lines.append(f"  rso_carrier({_ur_list(s.carrier)})")
        elif s.kind == "joint":
            if coord and s.carrier:
                lines.append(f"  rso_carrier({_ur_list(s.carrier)})")
            lines.append("  movej([" + ", ".join(f"{v:.6f}" for v in s.q) + f"], a={aj:.3f}, v={vj:.3f})")
        else:
            if coord and s.carrier:
                lines.append(f"  rso_carrier({_ur_list(s.carrier)})")
                Tb = frames[0] @ s.T
                lines.append(f"  movel({_ur_pose(Tb)}, a={cs.linear_acc:.3f}, v={cs.process_speed:.3f})")
            else:
                lines.append(
                    f"  movel(pose_trans(uf_{s.frame + 1}, {_ur_pose(s.T)}), a={cs.linear_acc:.3f}, "
                    f"v={cs.process_speed:.3f})"
                )
    lines += ["end", f"{name}()", ""]
    return "\n".join(lines)


def _ur_list(v: dict | None) -> str:
    return "[" + ", ".join(f"{float(x):.6f}" for x in (v or {}).values()) + "]"


def _vals(v: dict | None, unit: str) -> str:
    return ", ".join(
        f"{k} = {float(x) * (1000 if unit == 'mm' and k != 'yaw' else 1):.1f}" for k, x in (v or {}).items()
    )


def _fanuc(st, frames, coord, tool, cs, name) -> str:
    nm = name.upper()[:36]
    body, pos = [], []
    n = 0
    uf = None
    for s in st:
        if s.kind == "skip":
            body.append(f"   ! skipped ({s.note}): no joint solution, {s.issue} ;")
            continue
        if s.kind == "carrier":
            body.append(f"   ! {s.note}: move the carrier {_vals(s.carrier, 'mm')} ;")
            body.append("   WAIT DI[1]=ON ;")
            continue
        if s.frame != uf:
            uf = s.frame
            body.append(f"   UFRAME_NUM={uf + 1} ;")
            body.append("   UTOOL_NUM=1 ;")
        n += 1
        ext = ""
        if coord and s.carrier:
            ext = "\n   " + ",  ".join(
                f"E{i + 1}= {float(v) * 1000:.3f}  mm" for i, v in enumerate(s.carrier.values())
            )
        if s.kind == "joint":
            body.append(f"   J P[{n}] {min(100, int(round(100 * cs.joint_speed / 3.14)))}% FINE ;")
            jj = ",\t".join(f"J{i + 1}= {np.degrees(v):.3f} deg" for i, v in enumerate(s.q))
            pos.append(f"P[{n}]{{\n   GP1:\n\tUF : {uf + 1}, UT : 1,\n\t{jj}{ext}\n}};")
        else:
            body.append(f"   L P[{n}] {cs.process_speed * 1000:.0f}mm/sec CNT50 ;")
            x, y, z = (s.T[:3, 3] * 1000).tolist()
            w, p, r = _wpr(s.T[:3, :3])
            pos.append(
                f"P[{n}]{{\n   GP1:\n\tUF : {uf + 1}, UT : 1,\t\tCONFIG : {_fanuc_config(s.q)},\n"
                f"\tX = {x:.3f}  mm,\tY = {y:.3f}  mm,\tZ = {z:.3f}  mm,\n"
                f"\tW = {w:.3f} deg,\tP = {p:.3f} deg,\tR = {r:.3f} deg{ext}\n}};"
            )
        if s.issue:
            body[-1] = body[-1][:-2] + f" ;  ! flagged: {s.issue}"
    head = [
        f"/PROG  {nm}",
        "/ATTR",
        'COMMENT\t\t= "robot-sys-optimizer";',
        "DEFAULT_GROUP\t= 1,*,*,*,*;",
        "/MN",
        "   ! generated by robot-sys-optimizer (v10); check in ROBOGUIDE first ;",
    ]
    tx, ty, tz = (tool[:3, 3] * 1000).tolist()
    tw, tp, tr = _wpr(tool[:3, :3])
    head.append(f"   ! UTOOL 1 = X {tx:.3f} Y {ty:.3f} Z {tz:.3f} W {tw:.3f} P {tp:.3f} R {tr:.3f} ;")
    for k, F in enumerate(frames):
        x, y, z = (F[:3, 3] * 1000).tolist()
        w, p, r = _wpr(F[:3, :3])
        head.append(f"   ! UFRAME {k + 1} = X {x:.3f} Y {y:.3f} Z {z:.3f} W {w:.3f} P {p:.3f} R {r:.3f} ;")
    lines = head + [
        f"{i + 1:4d}:  {line[3:]}" if line.startswith("   ") else line for i, line in enumerate(body)
    ]
    lines += ["/POS", *pos, "/END", ""]
    return "\n".join(lines)


def _rapid(st, frames, coord, tool, cs, name) -> str:
    def pose(T):
        x, y, z = (T[:3, 3] * 1000).tolist()
        q = _quat(T[:3, :3])
        return f"[{x:.3f},{y:.3f},{z:.3f}],[{q[0]:.6f},{q[1]:.6f},{q[2]:.6f},{q[3]:.6f}]"

    def extax(c):
        v = [float(x) * 1000 for x in (c or {}).values()] if coord else []
        v = (v + [9e9] * 6)[:6]
        return "[" + ",".join("9E9" if x == 9e9 else f"{x:.3f}" for x in v) + "]"

    decl = [
        f"MODULE {name}",
        "  ! generated by robot-sys-optimizer (v10); check in RobotStudio first",
        f"  PERS tooldata rso_tool := [TRUE,[{pose(tool)}],[1,[0,0,0.05],[1,0,0,0],0,0,0]];",
    ]
    for k, F in enumerate(frames):
        decl.append(
            f'  TASK PERS wobjdata rso_wobj{k + 1} := [FALSE,TRUE,"",[{pose(F)}],[[0,0,0],[1,0,0,0]]];'
        )
    body = ["  PROC main()", "    ConfL \\Off;", "    ConfJ \\Off;"]
    n = 0
    vp = f"[{cs.process_speed * 1000:.0f},500,5000,1000]"
    for s in st:
        if s.kind == "skip":
            body.append(f"    ! skipped ({s.note}): no joint solution, {s.issue}")
            continue
        if s.kind == "carrier":
            body.append(f"    ! {s.note}: carrier {_vals(s.carrier, 'mm')}")
            body.append("    rso_carrier;")
            continue
        n += 1
        if s.issue:
            body.append(f"    ! flagged in the paths: {s.issue}")
        if s.kind == "joint":
            jj = ",".join(f"{np.degrees(v):.3f}" for v in s.q)
            decl.append(f"  CONST jointtarget j{n} := [[{jj}],{extax(s.carrier)}];")
            body.append(f"    MoveAbsJ j{n}, v500, fine, rso_tool;")
        else:
            decl.append(f"  CONST robtarget p{n} := [{pose(s.T)},{_abb_conf(s.q)},{extax(s.carrier)}];")
            body.append(f"    MoveL p{n}, {vp}, z5, rso_tool \\WObj:=rso_wobj{s.frame + 1};")
    body += [
        "  ENDPROC",
        "  PROC rso_carrier()",
        "    ! the integrator moves the rail / gantry / mobile base to the next stop here",
        "  ENDPROC",
        "ENDMODULE",
        "",
    ]
    return "\n".join(decl + body)
