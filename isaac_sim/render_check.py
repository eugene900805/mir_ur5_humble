"""Headless render of the converted USD to a PNG for visual verification."""
import os

from isaaclab.app import AppLauncher

app_launcher = AppLauncher({"headless": True, "enable_cameras": True})
simulation_app = app_launcher.app

import numpy as np
import omni.replicator.core as rep
from pxr import Usd, UsdGeom, Gf
import omni.usd

_HERE = os.path.dirname(os.path.abspath(__file__))
USD = os.path.join(_HERE, "usd", "mir_isaac.usd")
OUT_DIR = _HERE

ctx = omni.usd.get_context()
ctx.open_stage(USD)
stage = ctx.get_stage()

# Reference the robot under a fresh world so we can add lights/camera cleanly
rep.create.light(light_type="distant", intensity=3000)
rep.create.light(light_type="dome", intensity=1000)

# Camera positioned to view the whole robot (bbox ~1.6 x 0.8 x 1.0 m)
cam = rep.create.camera(position=(2.6, 2.2, 1.8), look_at=(0.35, 0.05, 0.5))
rp = rep.create.render_product(cam, (1280, 720))

rgb = rep.AnnotatorRegistry.get_annotator("rgb")
rgb.attach(rp)

# Step the app a number of frames so RTX accumulates / textures load
for _ in range(60):
    simulation_app.update()

data = rgb.get_data()
arr = np.asarray(data)
print("captured array shape:", arr.shape, "dtype:", arr.dtype)
if arr.size:
    nonzero = np.count_nonzero(arr[..., :3])
    print("non-black pixels (rgb):", nonzero, "/", arr[..., :3].size)
    try:
        from PIL import Image
        Image.fromarray(arr[..., :3].astype("uint8")).save(f"{OUT_DIR}/mir_isaac_render.png")
        print("saved", f"{OUT_DIR}/mir_isaac_render.png")
    except Exception as e:
        np.save(f"{OUT_DIR}/mir_isaac_render.npy", arr)
        print("PIL failed, saved npy:", e)

simulation_app.close()
