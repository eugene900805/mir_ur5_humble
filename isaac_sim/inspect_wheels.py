#!/usr/bin/env python3
"""Read the drive-wheel joint limits straight out of the USD.

The USD is a *snapshot* of the URDF, not a live view of it: edit the xacro and
the stage keeps the old numbers with no warning anywhere. This is the check that
tells you whether the stage is current.

`physxJoint:maxJointVelocity` is in **degrees/second**:
    24 rad/s (current MiR100 spec)  -> 1375.1
    20 rad/s (pre-2026-07-31 value) -> 1145.9
If you see 1145.9, the USD predates the spec fix -- rebuild it (README §3).

Also prints the movable joint list, which must stay at 22 and must keep the ROS
joint names, because the bridge matches ROS <-> Isaac by name.

Run with any python that has `pxr` (the Isaac Lab conda env):
    python inspect_wheels.py [path/to/mir_isaac.usd]
"""
import os
import sys

from pxr import Usd, UsdPhysics

HERE = os.path.dirname(os.path.abspath(__file__))
usd = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "usd", "mir_isaac.usd")

stage = Usd.Stage.Open(usd)
movable = []
for prim in stage.Traverse():
    if prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.PrismaticJoint):
        movable.append(prim.GetName())
    name = prim.GetName()
    if "wheel_joint" in name and ("left" in name or "right" in name):
        print(name, prim.GetPath())
        for attr in prim.GetAttributes():
            an = attr.GetName()
            if "axis" in an.lower() or "Vel" in an or "limit" in an.lower() or "physxJoint" in an:
                print("   ", an, "=", attr.Get())
print("movable joints:", len(movable))
print(sorted(movable))
