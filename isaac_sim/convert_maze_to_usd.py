#!/usr/bin/env python3
"""Convert the Gazebo `maze` world (mir_gazebo/worlds/include/maze/model.sdf)
into a USD stage usable as the Isaac Sim environment.

The maze is just a set of static box walls. For each <link> we compose
  world = model_pose * link_pose * geometry_pose
and emit a UsdGeom.Cube (unit cube + matrix that carries the box dimensions as
scale), tagged as a static collider so the robot can drive into the walls.

Usage:
  python convert_maze_to_usd.py [SDF_PATH] [OUT_USD]
Defaults: the repo maze model -> isaac_sim/usd/maze.usd
"""
import math
import os
import sys
import xml.etree.ElementTree as ET

from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade, Usd  # noqa: F401

# PhysxSchema is registered by Isaac's physx extension, so it is only importable
# when a SimulationApp is up. This script is plain pxr (it runs standalone to
# regenerate the stage), so fall back to authoring the physx attributes by name —
# the resulting USD is identical either way.
try:
    from pxr import PhysxSchema
except ImportError:
    PhysxSchema = None

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, ".."))
DEFAULT_SDF = os.path.join(
    REPO, "mir_robot/mir_gazebo/worlds/include/maze/model.sdf")
DEFAULT_OUT = os.path.join(HERE, "usd", "maze.usd")


def parse_pose(text):
    """SDF pose '<x y z roll pitch yaw>' -> Gf.Matrix4d (rigid transform)."""
    if text is None:
        return Gf.Matrix4d(1.0)
    vals = [float(v) for v in text.split()]
    x, y, z, roll, pitch, yaw = (vals + [0.0] * 6)[:6]
    # SDF/Gazebo rotation: R = Rz(yaw) * Ry(pitch) * Rx(roll)
    rot = (
        Gf.Rotation(Gf.Vec3d(0, 0, 1), math.degrees(yaw))
        * Gf.Rotation(Gf.Vec3d(0, 1, 0), math.degrees(pitch))
        * Gf.Rotation(Gf.Vec3d(1, 0, 0), math.degrees(roll))
    )
    m = Gf.Matrix4d(1.0)
    m.SetRotateOnly(rot)
    m.SetTranslateOnly(Gf.Vec3d(x, y, z))
    return m


def main():
    sdf_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_SDF
    out_path = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_OUT
    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    tree = ET.parse(sdf_path)
    model = tree.getroot().find(".//model")
    model_pose = parse_pose(
        model.findtext("pose") if model is not None else None)

    stage = Usd.Stage.CreateNew(out_path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, "/World")
    maze = UsdGeom.Xform.Define(stage, "/World/maze")
    stage.SetDefaultPrim(world.GetPrim())

    # one shared grey material for all the walls
    mat = UsdShade.Material.Define(stage, "/World/maze/WallMaterial")
    shader = UsdShade.Shader.Define(stage, "/World/maze/WallMaterial/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(0.6, 0.6, 0.62))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.8)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

    # A *physics* material. The UsdShade material above is purely visual and has
    # no bearing on contact: without this the walls run on PhysX's default
    # friction, and a base that Nav2 presses into a wall slides along and digs
    # into it instead of being held. Bound under the "physics" purpose, so it
    # coexists with the visual binding rather than replacing it.
    phys_mat = UsdShade.Material.Define(stage, "/World/maze/WallPhysicsMaterial")
    phys_api = UsdPhysics.MaterialAPI.Apply(phys_mat.GetPrim())
    phys_api.CreateStaticFrictionAttr().Set(0.8)
    phys_api.CreateDynamicFrictionAttr().Set(0.8)
    phys_api.CreateRestitutionAttr().Set(0.0)

    n = 0
    for link in model.findall("link"):
        link_pose = parse_pose(link.findtext("pose"))
        # prefer the visual geometry; fall back to collision
        geom_src = link.find("visual") or link.find("collision")
        box = geom_src.find("geometry/box") if geom_src is not None else None
        if box is None:
            continue
        size = [float(v) for v in box.findtext("size").split()]
        geom_pose = parse_pose(geom_src.findtext("pose"))

        world_m = geom_pose * link_pose * model_pose  # USD: row-vector, left-mul

        name = link.get("name", f"Wall_{n}").replace("-", "_")
        cube = UsdGeom.Cube.Define(stage, f"/World/maze/{name}")
        cube.GetSizeAttr().Set(1.0)  # unit cube spanning -0.5..0.5
        # matrix carries translation, rotation AND the box dimensions (scale)
        scale = Gf.Matrix4d(1.0)
        scale.SetScale(Gf.Vec3d(size[0], size[1], size[2]))
        xf = UsdGeom.Xformable(cube)
        xf.MakeMatrixXform().Set(scale * world_m)

        # static collider + material
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        # Author the contact offsets explicitly. These cubes are unit cubes that
        # carry their real dimensions (up to 16 m) as a non-uniform scale in the
        # transform matrix; leaving the offsets unauthored means PhysX derives
        # them from that scale, so a 16 m wall and a 1.5 m wall end up with very
        # different contact skins. A robot pressed into a corner where two such
        # walls meet can then penetrate deeply before contact is generated, and
        # the solver ejects it. Fixed values give every wall the same skin.
        if PhysxSchema is not None:
            px = PhysxSchema.PhysxCollisionAPI.Apply(cube.GetPrim())
            px.CreateContactOffsetAttr().Set(0.02)
            px.CreateRestOffsetAttr().Set(0.0)
        else:
            # ApplyAPI() refuses an unregistered schema, so append the token to
            # apiSchemas by hand and author the attributes. Byte-for-byte the
            # same result as PhysxCollisionAPI.Apply() under Isaac's python.
            p = cube.GetPrim()
            listop = p.GetMetadata("apiSchemas")
            items = list(listop.GetAddedOrExplicitItems()) if listop else []
            if "PhysxCollisionAPI" not in items:
                items.append("PhysxCollisionAPI")
            p.SetMetadata("apiSchemas", Sdf.TokenListOp.CreateExplicit(items))
            p.CreateAttribute("physxCollision:contactOffset",
                              Sdf.ValueTypeNames.Float, False).Set(0.02)
            p.CreateAttribute("physxCollision:restOffset",
                              Sdf.ValueTypeNames.Float, False).Set(0.0)
        UsdShade.MaterialBindingAPI(cube.GetPrim()).Bind(mat)
        UsdShade.MaterialBindingAPI(cube.GetPrim()).Bind(
            phys_mat, UsdShade.Tokens.weakerThanDescendants, "physics")
        n += 1

    stage.GetRootLayer().Save()
    print(f"wrote {n} walls -> {out_path}")


if __name__ == "__main__":
    main()
