#!/usr/bin/env python3
"""Correct the Isaac importer's spurious -90deg X rotation on .dae visual meshes,
WITHOUT discarding their per-material colors (which switching to .stl would lose).

Each .dae visual gets a 'Scene' Xform with orient = Rx(-90) = (-0.7071,0.7071,0,0).
The correct orientation (matching the collision .stl) is Rx(+90) = (0.7071,0.7071,0,0).
We flip it in the base USD layer.
"""
import sys
from pxr import Usd, Gf

# pass the base USD path, e.g. usd_mimic/configuration/mir_ur5_base.usd
BASE = sys.argv[1] if len(sys.argv) > 1 else "/mnt/data/mir_ur5/usd_mimic/configuration/mir_ur5_base.usd"
GOOD = Gf.Quatf(0.70710677, 0.70710677, 0.0, 0.0)   # Rx(+90)

stage = Usd.Stage.Open(BASE)
n = 0
for p in stage.Traverse():
    if p.GetName() == "Scene" and "visuals" in p.GetPath().pathString:
        a = p.GetAttribute("xformOp:orient")
        o = a.Get()
        if o is not None and o.GetReal() < 0:  # the broken Rx(-90)
            a.Set(GOOD)
            n += 1
stage.GetRootLayer().Save()
print("corrected Scene nodes:", n)
