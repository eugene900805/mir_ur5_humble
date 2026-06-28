#!/usr/bin/env python3
"""Repair the Isaac URDF importer's broken Robotiq mimic output.

For some passive Robotiq joints the importer applies PhysxMimicJointAPI but
leaves the revolute limit at -inf/inf and/or omits the required referenceJoint
relationship -> PhysX rejects the mimic at runtime with
  "needs a finite limit set to be used by the mimic joint feature".

This walks every revolute joint that carries a physxMimicJoint:<axis>:gearing
attribute and, in the USD root layer:
  * sets a finite limit (degrees) where it is non-finite, and
  * points referenceJoint at the master knuckle joint where it is missing.

Usage: python3 fix_mimic_limits.py [USD_PATH]
"""
import math
import os
import sys

from pxr import Sdf, Usd

HERE = os.path.dirname(os.path.abspath(__file__))
USD = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "usd", "mir_isaac.usd")
MASTER = "/mir_100/joints/robotiq_85_left_knuckle_joint"
LIMIT_DEG = 90.0  # generous finite limit (master travels ~0..46 deg)


def mimic_axis(prim):
    """Return the axis token (e.g. 'rotY') of the joint's mimic gearing, or None."""
    for attr in prim.GetAttributes():
        name = attr.GetName()
        if name.startswith("physxMimicJoint:") and name.endswith(":gearing"):
            return name.split(":")[1]
    return None


def main():
    stage = Usd.Stage.Open(USD)
    stage.SetEditTarget(stage.GetRootLayer())

    fixed_limit, fixed_ref = [], []
    for p in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
        if p.GetTypeName() != "PhysicsRevoluteJoint":
            continue
        axis = mimic_axis(p)
        if axis is None:
            continue
        name = p.GetName()

        lo = p.GetAttribute("physics:lowerLimit").Get()
        hi = p.GetAttribute("physics:upperLimit").Get()
        if lo is None or hi is None or not math.isfinite(lo) or not math.isfinite(hi):
            p.GetAttribute("physics:lowerLimit").Set(-LIMIT_DEG)
            p.GetAttribute("physics:upperLimit").Set(LIMIT_DEG)
            fixed_limit.append(name)

        ref_name = f"physxMimicJoint:{axis}:referenceJoint"
        rel = p.GetRelationship(ref_name)
        targets = list(rel.GetTargets()) if rel else []
        if targets != [Sdf.Path(MASTER)]:
            if not rel:
                rel = p.CreateRelationship(ref_name, custom=False)
            rel.SetTargets([Sdf.Path(MASTER)])
            fixed_ref.append(name)

    stage.GetRootLayer().Save()
    print("fixed limits      :", fixed_limit)
    print("fixed referenceRel:", fixed_ref)


if __name__ == "__main__":
    main()
