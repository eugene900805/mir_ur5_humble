#!/usr/bin/env python3
"""Correct the Isaac importer's spurious -90deg X rotation on .dae visual meshes,
WITHOUT discarding their per-material colors (which switching to .stl would lose).

Each .dae visual gets a 'Scene' Xform with orient = Rx(-90) = (-0.7071,0.7071,0,0).
The correct orientation (matching the collision .stl) is Rx(+90) = (0.7071,0.7071,0,0).
We flip it in the base USD layer.
"""
import os
import sys
from pxr import Usd, Gf

# pass the base USD path; defaults to this checkout's usd/configuration/ layer
_HERE = os.path.dirname(os.path.abspath(__file__))
_DEFAULT_BASE = os.path.join(_HERE, "usd", "configuration", "mir_isaac_base.usd")
BASE = sys.argv[1] if len(sys.argv) > 1 else _DEFAULT_BASE
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
