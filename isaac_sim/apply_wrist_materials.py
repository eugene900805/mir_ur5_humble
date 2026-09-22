"""Apply wrist URDF colors after Isaac's STL importer assigns white materials.

Run with a Python providing pxr, or automatically via convert_to_usd.py.
Author on the root USD layer so regenerated importer layers stay untouched.
"""
import argparse
from pathlib import Path
import xml.etree.ElementTree as ET

from pxr import Gf, Sdf, Usd, UsdShade


WRIST_LINKS = (
    "robotiq_ft300_mounting_plate",
    "robotiq_ft300_sensor",
    "robotiq_wrist_camera_link",
)


def apply_wrist_materials(urdf_path, usd_path):
    robot = ET.parse(urdf_path).getroot()
    stage = Usd.Stage.Open(str(usd_path))
    if not stage or not stage.GetDefaultPrim():
        raise ValueError(f"USD has no default robot prim: {usd_path}")
    stage.SetEditTarget(stage.GetRootLayer())
    root = stage.GetDefaultPrim().GetPath()
    for name in WRIST_LINKS:
        color = robot.find(f"link[@name='{name}']/visual/material/color")
        if color is None:
            raise ValueError(f"Missing URDF visual color for {name}")
        rgba = tuple(float(v) for v in color.get("rgba").split())
        if len(rgba) != 4 or not all(0 <= v <= 1 for v in rgba):
            raise ValueError(f"Invalid RGBA for {name}: {rgba}")
        visuals = stage.GetPrimAtPath(root.AppendPath(name + "/visuals"))
        if not visuals:
            raise ValueError(f"Missing USD visuals for {name}")
        path = root.AppendPath("Looks/URDFWrist/" + name)
        material = UsdShade.Material.Define(stage, path)
        shader = UsdShade.Shader.Define(stage, path.AppendChild("Surface"))
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgba[:3]))
        shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(rgba[3])
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.65)
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        # STL meshes live in instance prototypes with their own white bindings.
        # A stronger binding on the instance root overrides those without
        # de-instancing or changing the source mesh's material.
        UsdShade.MaterialBindingAPI.Apply(visuals).Bind(
            material, bindingStrength=UsdShade.Tokens.strongerThanDescendants
        )
    stage.GetRootLayer().Save()
    print(f"Applied URDF colors to {len(WRIST_LINKS)} wrist links")


if __name__ == "__main__":
    here = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, default=here / "mir_isaac.urdf")
    parser.add_argument("--usd", type=Path, default=here / "usd/mir_isaac.usd")
    args = parser.parse_args()
    apply_wrist_materials(args.urdf, args.usd)
