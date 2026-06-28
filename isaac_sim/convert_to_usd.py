"""Convert the self-contained MiR+UR5 URDF -> USD with Robotiq mimic joints.

Run with the Isaac Lab python environment, e.g.:
  conda activate env_isaaclab_opt
  python isaac_sim/convert_to_usd.py

UrdfConverterCfg.convert_mimic_joints_to_normal_joints maps onto the importer's
set_parse_mimic(): True keeps the Robotiq <mimic> tags as real PhysX coupled
joints (drive only robotiq_85_left_knuckle_joint, the other 5 follow).
fix_base=False because this is a mobile base that must drive on the ground.
"""
import os

from isaaclab.app import AppLauncher

app_launcher = AppLauncher({"headless": True})
simulation_app = app_launcher.app

from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
URDF = os.path.join(HERE, "mir_isaac.urdf")
OUT_DIR = os.path.join(HERE, "usd")
OUT_FILE = "mir_isaac.usd"

cfg = UrdfConverterCfg(
    asset_path=URDF,
    usd_dir=OUT_DIR,
    usd_file_name=OUT_FILE,
    fix_base=False,
    merge_fixed_joints=False,
    force_usd_conversion=True,
    convert_mimic_joints_to_normal_joints=True,
    joint_drive=UrdfConverterCfg.JointDriveCfg(
        gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=100.0, damping=1.0),
        target_type="position",
    ),
)

conv = UrdfConverter(cfg)
print("Generated USD:", conv.usd_path)

simulation_app.close()
