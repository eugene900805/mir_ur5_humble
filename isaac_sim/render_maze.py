from isaaclab.app import AppLauncher
app = AppLauncher({"headless": True, "enable_cameras": True}).app
import numpy as np, omni.replicator.core as rep, omni.usd
from isaacsim.core.utils import stage as stg
from pxr import UsdLux, Usd
ctx = omni.usd.get_context(); ctx.new_stage(); s = ctx.get_stage()
UsdLux.DomeLight.Define(s, "/World/DomeLight").CreateIntensityAttr(1000.0)
rep.create.light(light_type="distant", intensity=2500)
stg.add_reference_to_stage("/mnt/data/mir_isaac/mir_ur5_humble/isaac_sim/usd/maze.usd", "/World/Env")
stg.add_reference_to_stage("/mnt/data/mir_isaac/mir_ur5_humble/isaac_sim/usd/mir_isaac.usd", "/World/Robot")
for _ in range(5): app.update()
cam = rep.create.camera(position=(14, 14, 16), look_at=(0, 0, 0))
rp = rep.create.render_product(cam, (1280, 720))
rgb = rep.AnnotatorRegistry.get_annotator("rgb"); rgb.attach(rp)
for _ in range(60): app.update()
from PIL import Image
arr = np.asarray(rgb.get_data())[..., :3].astype("uint8")
Image.fromarray(arr).save("/mnt/data/mir_isaac/mir_ur5_humble/isaac_sim/maze_render.png")
print("saved, nonzero:", int((arr>0).sum()))
app.close()
