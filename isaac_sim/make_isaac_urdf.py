#!/usr/bin/env python3
"""Turn the xacro-generated MiR+UR5 URDF into a self-contained URDF that the
Isaac Sim / Isaac Lab URDF importer can consume.

Mirrors the fixes worked out in ../../mir_ur5/README.md, adapted to the
mir_ur5_humble robot (UR joints carry the ``ur_`` prefix, wheels are already
meshes, gripper passive joints are ``continuous`` without limits):

  * resolve every ``package://`` / ``file://`` mesh path to an absolute path
  * copy meshes whose stem contains characters illegal in a USD prim name
    (e.g. ``sick_lms-100.stl``) to a sanitised filename
  * rename joints whose name contains ``-`` (UR fixed joints) -> ``_``
  * turn the Robotiq passive ``continuous`` mimic joints into ``revolute`` with
    finite limits (PhysX mimic coupling requires finite limits)
  * strip <ros2_control>, <gazebo> and <transmission> blocks (importer chokes
    on / ignores them)

Usage:
  python3 make_isaac_urdf.py INPUT.urdf OUTPUT.urdf [--ws WORKSPACE_INSTALL]
"""
import argparse
import os
import re
import shutil
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))


def _default_ws():
    """Locate the colcon install/ dir. The repo has lived both at the workspace
    root and under <ws>/src/, so walk up until an install/ shows up rather than
    hard-coding one of the two (the old fixed ../install silently resolved to
    nothing, and every package:// mesh was then dropped from the URDF)."""
    d = HERE
    for _ in range(5):
        d = os.path.dirname(d)
        cand = os.path.join(d, "install")
        if os.path.isdir(cand):
            return cand
    return os.path.normpath(os.path.join(HERE, "..", "install"))


DEFAULT_WS = _default_ws()

# Robotiq passive joints range within +-master_upper (0.8). A small margin
# keeps PhysX happy without clipping the coupled motion.
PASSIVE_LIMIT = 0.85


def pkg_roots(ws):
    return {
        "mir_description": os.path.join(ws, "mir_description/share/mir_description"),
        "ur_description": os.path.join(ws, "ur_description/share/ur_description"),
        "robotiq_description": os.path.join(ws, "robotiq_description/share/robotiq_description"),
        "realsense2_description": os.path.join(
            ws, "realsense2_description/share/realsense2_description"),
    }


def resolve(filename, roots):
    """Absolute path for a package:// / file:// / plain filename, else None."""
    if filename.startswith("file://"):
        path = filename[len("file://"):]
        return path if os.path.exists(path) else None
    if filename.startswith("package://"):
        pkg, _, rel = filename[len("package://"):].partition("/")
        base = roots.get(pkg)
        if base is None:
            return None
        path = os.path.join(base, rel)
        return path if os.path.exists(path) else None
    return filename if os.path.exists(filename) else None


def sanitize(abspath):
    """USD prim names are derived from the mesh file stem and cannot contain
    characters like '-'. Copy to a USD-safe filename when needed."""
    d, fn = os.path.split(abspath)
    stem, ext = os.path.splitext(fn)
    safe = re.sub(r"[^A-Za-z0-9_]", "_", stem)
    if safe and safe[0].isdigit():
        safe = "_" + safe
    if safe == stem:
        return abspath
    dst = os.path.join(d, safe + ext)
    if not os.path.exists(dst):
        shutil.copy(abspath, dst)
    return dst


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src")
    ap.add_argument("dst")
    ap.add_argument("--ws", default=DEFAULT_WS,
                    help="colcon install dir holding <pkg>/share/<pkg>/meshes")
    a = ap.parse_args()

    roots = pkg_roots(a.ws)
    tree = ET.parse(a.src)
    root = tree.getroot()

    # 1. strip tags the importer doesn't want
    for tag in ("ros2_control", "gazebo", "transmission"):
        for elem in root.findall(tag):
            root.remove(elem)

    # 2. resolve / sanitise mesh paths, drop the unresolvable ones
    resolved, dropped = 0, []
    for link in root.findall("link"):
        for tag in ("visual", "collision"):
            for elem in list(link.findall(tag)):
                mesh = elem.find("geometry/mesh")
                if mesh is None:
                    continue
                fn = mesh.get("filename")
                abspath = resolve(fn, roots)
                if abspath is None:
                    link.remove(elem)
                    dropped.append((link.get("name"), tag, fn))
                else:
                    mesh.set("filename", sanitize(abspath))
                    resolved += 1

    # 3. rename joints whose name has '-' (illegal in USD prim names)
    renamed = 0
    for joint in root.findall("joint"):
        jn = joint.get("name")
        if jn and "-" in jn:
            joint.set("name", jn.replace("-", "_"))
            renamed += 1

    # 4. continuous mimic joints -> revolute with finite limits
    mimic_fixed = 0
    for joint in root.findall("joint"):
        if joint.get("type") == "continuous" and joint.find("mimic") is not None:
            joint.set("type", "revolute")
            if joint.find("limit") is None:
                ET.SubElement(joint, "limit", {
                    "lower": str(-PASSIVE_LIMIT), "upper": str(PASSIVE_LIMIT),
                    "effort": "50", "velocity": "0.5"})
            mimic_fixed += 1

    tree.write(a.dst, xml_declaration=True, encoding="utf-8")
    print(f"stripped ros2_control/gazebo/transmission tags")
    print(f"resolved meshes        : {resolved}")
    print(f"renamed joints (- -> _): {renamed}")
    print(f"mimic continuous->rev  : {mimic_fixed}")
    print(f"dropped (unresolvable) : {len(dropped)}")
    for name, tag, fn in dropped:
        print(f"  - link={name} {tag} -> {fn}")
    print(f"written: {a.dst}")


if __name__ == "__main__":
    main()
