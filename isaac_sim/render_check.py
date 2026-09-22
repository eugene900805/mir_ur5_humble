"""Render current USD previews for README, wrist detail and maze overview.

Run with the Isaac Lab Python environment. Poses and lights are authored only
in an in-memory stage; the robot USD and running simulation are not modified.
"""
from pathlib import Path
import xml.etree.ElementTree as ET

from isaaclab.app import AppLauncher

app_launcher = AppLauncher({"headless": True, "enable_cameras": True})
simulation_app = app_launcher.app

import numpy as np
from PIL import Image
import omni.replicator.core as rep
import omni.usd
from pxr import Gf, UsdGeom
import trimesh

HERE = Path(__file__).resolve().parent
USD = HERE / "usd/mir_isaac.usd"
ctx = omni.usd.get_context()
ctx.new_stage()
stage = ctx.get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
robot_path = "/World/Robot"
stage.DefinePrim(robot_path, "Xform").GetReferences().AddReference(str(USD))

# A raised-arm presentation pose, applied to the imported link transforms.
# Static rendering does not step PhysX, so there is no settling or ROS traffic.
pose = {"ur_shoulder_lift_joint": -np.pi / 2, "ur_wrist_1_joint": -np.pi / 2}
urdf = ET.parse(HERE / "mir_isaac.urdf").getroot()
joints = urdf.findall("joint")
transforms = {"base_footprint": np.eye(4)}
pending = list(joints)
while pending:
    progressed = False
    for joint in pending[:]:
        parent = joint.find("parent").get("link")
        child = joint.find("child").get("link")
        if parent not in transforms:
            continue
        origin = joint.find("origin")
        xyz = [0.0] * 3 if origin is None else list(map(float, origin.get("xyz", "0 0 0").split()))
        rpy = [0.0] * 3 if origin is None else list(map(float, origin.get("rpy", "0 0 0").split()))
        matrix = trimesh.transformations.euler_matrix(*rpy)
        matrix[:3, 3] = xyz
        if joint.get("type") in ("revolute", "continuous"):
            angle = pose.get(joint.get("name"), 0.0)
            axis = list(map(float, joint.find("axis").get("xyz").split()))
            matrix = matrix @ trimesh.transformations.rotation_matrix(angle, axis)
        transforms[child] = transforms[parent] @ matrix
        pending.remove(joint)
        progressed = True
    if not progressed:
        raise RuntimeError("Cannot resolve URDF link transforms")
for name, matrix in transforms.items():
    prim = stage.GetPrimAtPath(robot_path + "/" + name)
    if prim:
        UsdGeom.Xformable(prim).MakeMatrixXform().Set(Gf.Matrix4d(matrix.T.tolist()))

rep.create.light(light_type="distant", intensity=1000, rotation=(315, 0, 135))
rep.create.light(light_type="dome", intensity=350)
ground = rep.create.plane(scale=(200, 200, 1), position=(0, 0, -0.002))
with ground:
    rep.modify.material(rep.create.material_omnipbr(diffuse=(0.24, 0.26, 0.29), roughness=0.8))


def capture(camera, resolution, paths):
    product = rep.create.render_product(camera, resolution)
    rgb = rep.AnnotatorRegistry.get_annotator("rgb")
    rgb.attach(product)
    for _ in range(60):
        simulation_app.update()
    array = np.asarray(rgb.get_data())
    if array.shape[:2] != (resolution[1], resolution[0]) or array[..., :3].std() < 2:
        raise RuntimeError(f"Invalid rendered image: {array.shape}")
    for path in paths:
        Image.fromarray(array[..., :3].astype("uint8")).save(path)
        print("saved", path, flush=True)
    rgb.detach(product)
    product.destroy()


camera = rep.create.camera(position=(2.8, 2.8, 2.2), look_at=(0.12, 0.0, 0.95), focal_length=28)
capture(camera, (1440, 1200), [HERE / "mir_isaac_render.png", HERE.parent / "Isaac_Sim_Mir_Ur5.png"])

wrist = np.array(UsdGeom.XformCache().GetLocalToWorldTransform(
    stage.GetPrimAtPath(robot_path + "/robotiq_wrist_camera_link")
).ExtractTranslation())
print("wrist render center:", wrist, "URDF center:", transforms["robotiq_wrist_camera_link"][:3, 3], flush=True)
camera = rep.create.camera(position=tuple(wrist + [0.33, 0.36, 0.22]),
                           look_at=tuple(wrist + [0, 0.025, 0]), focal_length=28,
                           clipping_range=(0.01, 100.0))
capture(camera, (1440, 960), [HERE / "mir_wrist_render.png"])

maze = HERE / "usd/maze.usd"
if maze.exists():
    stage.DefinePrim("/World/Environment", "Xform").GetReferences().AddReference(str(maze))
    camera = rep.create.camera(position=(14, 14, 16), look_at=(0, 0, 0), focal_length=24)
    capture(camera, (1280, 720), [HERE / "maze_render.png"])

simulation_app.close()
