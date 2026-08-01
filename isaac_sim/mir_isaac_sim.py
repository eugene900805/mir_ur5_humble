#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
#
# Isaac Sim back-end for the MiR100 + UR5 + Robotiq85 + D435i robot.
# Replaces the Gazebo simulator. Bridges the robot articulation to ROS2 /
# ros2_control / MoveIt via the topic_based_ros2_control interface:
#
#   Isaac  --(sensor_msgs/JointState)-->  /isaac_joint_states   --> ros2_control
#   ros2_control --(sensor_msgs/JointState)--> /isaac_joint_commands --> Isaac
#   Isaac  --(rosgraph_msgs/Clock)-->     /clock                (use_sim_time)
#
# On the ROS side run, in another terminal:
#   ros2 launch mir_gazebo mir_isaac.launch.py
#
# Run this script with the Isaac Sim python environment, e.g.:
#   cd /mnt/data/IsaacSim/_build/linux-x86_64/release
#   ./python.sh /mnt/data/mir_isaac/mir_ur5_humble/isaac_sim/mir_isaac_sim.py \
#       --usd /mnt/data/mir_isaac/mir_ur5_humble/isaac_sim/usd/mir_isaac.usd

import argparse
import os
import sys

# ----------------------------------------------------------------- CLI args
# (parsed before SimulationApp so we can honour --headless)
parser = argparse.ArgumentParser(description="Isaac Sim ROS2 bridge for MiR+UR5")
_default_usd = os.path.join(os.path.dirname(os.path.abspath(__file__)), "usd", "mir_isaac.usd")
parser.add_argument("--usd", default=_default_usd, help="Path to the robot USD.")
parser.add_argument("--headless", action="store_true", help="Run without a GUI.")
parser.add_argument("--test-walls", action="store_true",
                    help="Diagnostic: build a closed box of walls around the "
                         "robot so every direction has a wall — used to check the "
                         "lidar covers a full 360 (gaps = real sensor blind spot).")
parser.add_argument("--top-down", action="store_true",
                    help="Start the GUI viewport looking straight down (top-down "
                         "/ bird's-eye), like the RViz map view, instead of the "
                         "default 3/4 perspective.")
parser.add_argument("--top-down-height", type=float, default=18.0,
                    help="Camera height (m) for --top-down. Raise to see more of "
                         "the map, lower to zoom in.")
parser.add_argument("--commands-topic", default="isaac_joint_commands",
                    help="(legacy single-topic; kept for the home-pose ramp)")
parser.add_argument("--command-topics", nargs="+",
                    default=["isaac_arm_commands", "isaac_base_commands",
                             "isaac_gripper_commands"],
                    help="One command topic per ros2_control hardware component. "
                         "Separate topics avoid the 3-publisher interleaving that "
                         "starved the arm on a single /isaac_joint_commands.")
parser.add_argument("--states-topic", default="isaac_joint_states")
parser.add_argument("--robot-prim", default="",
                    help="Articulation root prim path. Auto-detected if empty.")
parser.add_argument("--world", default="",
                    help="Path to an environment USD to load under /World/Env "
                         "(e.g. isaac_sim/usd/maze.usd). If omitted, only a "
                         "ground plane is added.")
parser.add_argument("--arm-stiffness", type=float, default=10000.0,
                    help="UR joint position-drive stiffness (tracks MoveIt "
                         "trajectories tightly).")
parser.add_argument("--arm-damping", type=float, default=1000.0,
                    help="UR joint position-drive damping.")
parser.add_argument("--arm-max-force", type=float, default=330.0,
                    help="Per-joint torque cap (N*m) ~ UR5 limits. Enough to "
                         "follow trajectories, gentle enough that the arm cannot "
                         "flip the 67 kg MiR base (verified upright to 500 N*m).")
parser.add_argument("--arm-armature", type=float, default=0.0,
                    help="Rotor inertia (armature) added to the 6 UR joints. The "
                         "wrist links have small inertia, so the stiff drive rings "
                         "(arm jitter) the same way the gripper did; armature "
                         "raises the effective inertia into the integrable range. "
                         "0.0 disables it.")
parser.add_argument("--no-arm-home", action="store_true",
                    help="Skip the startup ramp that eases the arm to its ROS "
                         "home pose (the ramp prevents the base from flipping "
                         "when ros2_control first commands the arm).")
parser.add_argument("--base-linear-damping", type=float, default=0.0,
                    help="Linear damping on the MiR chassis (base_link). Default "
                         "0.0 = original behaviour. WARNING: high values (e.g. 2.0) "
                         "drag the base so hard it barely drives under Nav2. "
                         "Optional idle-jitter knob only.")
parser.add_argument("--base-angular-damping", type=float, default=0.0,
                    help="Angular (yaw) damping on the MiR chassis (base_link). "
                         "Default 0.0 = original behaviour. Optional knob for idle "
                         "yaw wobble; high values also resist commanded turns.")
parser.add_argument("--wheel-drive-damping", type=float, default=1.0e5,
                    help="Velocity-drive damping (gain) on the 2 drive wheels. "
                         "Higher = the wheels track the commanded rad/s more "
                         "stiffly under load (helps in-place rotation, where the "
                         "wheels otherwise lag/slip and the base barely turns).")
parser.add_argument("--wheel-friction", type=float, default=2.0,
                    help="Static/dynamic friction for the drive wheels + ground. "
                         "The URDF's high wheel friction is a <gazebo> tag Isaac "
                         "ignores, so without this the wheels slip on the default "
                         "low-friction ground: the robot drives SLOWER than the "
                         "commanded cmd_vel and turns erratically (sometimes "
                         "over-turns, sometimes not at all). 0.0 disables it.")
parser.add_argument("--caster-swivel-damping", type=float, default=0.0,
                    help="Rotational damping on the 4 passive caster *swivel* "
                         "joints. The rolling caster wheels stay fully free; only "
                         "the swivel gets light damping so the trailing casters "
                         "cannot pump yaw into the chassis (base slowly spinning "
                         "in place when idle). 0.0 = fully free (old behaviour).")
parser.add_argument("--gripper-stiffness", type=float, default=1.0e4,
                    help="Stiffness of the Robotiq master position drive. The "
                         "finger links have tiny inertia, so a very stiff drive "
                         "rings; lower this (e.g. 1e3) together with --gripper-"
                         "armature if the gripper buzzes.")
parser.add_argument("--gripper-damping", type=float, default=1.0e3,
                    help="Damping on the Robotiq master position drive. Raise to "
                         "settle gripper/finger jitter from the PhysX mimic "
                         "coupling.")
parser.add_argument("--gripper-armature", type=float, default=0.05,
                    help="Rotor inertia (armature) added to the 6 Robotiq joints. "
                         "The fingers' inertia is ~1e-5 kg*m^2, so the stiff drive "
                         "+ hard PhysX mimic coupling oscillate faster than the "
                         "sim step can integrate -> buzzing. Armature raises the "
                         "joints' effective inertia into the integrable range and "
                         "is the standard fix. 0.0 disables it.")
parser.add_argument("--solver-position-iterations", type=int, default=32,
                    help="PhysX articulation solver position-iteration count. The "
                         "Robotiq finger linkage is over-constrained by the hard "
                         "mimic joints; more iterations converge those constraints "
                         "each step and cut gripper jitter. (default importer ~4)")
parser.add_argument("--solver-velocity-iterations", type=int, default=4,
                    help="PhysX articulation solver velocity-iteration count.")
parser.add_argument("--max-depenetration-velocity", type=float, default=0.0,
                    help="Cap (m/s) on how fast PhysX may push the robot's bodies "
                         "back out of geometry they have penetrated. 0.0 (default) "
                         "leaves it unlimited. "
                         "EXPERIMENTAL, and off by default for a reason: it was "
                         "added to stop Nav2 wedging the base into a wall corner "
                         "and PhysX then ejecting it out of the map in one step "
                         "(seen at the maze's 0.93 m gap: z 0.001 -> 2.1 m, "
                         "y 3.8 -> 20.7 m in 0.5 s, which takes AMCL with it). "
                         "At 1.0 it does prevent that, but it causes a WORSE "
                         "failure: with depenetration throttled the constraint "
                         "error accumulates until the solver diverges, and the "
                         "robot explodes from a standstill (measured: rest at "
                         "(-0.08,-0.30) -> 53 m away in one 0.43 s step, then "
                         "runaway to 2088 m and NaN). If you want to experiment, "
                         "try values >= 5.0; the real fix for the wedging is "
                         "ObstaclesCritic.consider_footprint: true on the Nav2 "
                         "side, which stops the base entering the corner at all.")
parser.add_argument("--keep-self-collisions", action="store_true",
                    help="Keep intra-robot self-collisions enabled. By default "
                         "they are DISABLED, because the Robotiq fingers grazing "
                         "each other under the PhysX mimic coupling make the "
                         "gripper jitter; external-object collision (grasping) is "
                         "unaffected either way.")
parser.add_argument("--wheel-test", action="store_true",
                    help="Diagnostic: spin the drive wheels directly via the "
                         "articulation API at startup and report rotation, to "
                         "isolate drive vs OmniGraph controller faults.")
parser.add_argument("--publish-odom", action=argparse.BooleanOptionalAction,
                    default=True,
                    help="Publish odom->base_footprint TF + ground-truth "
                         "odometry from Isaac. ON by default, and it has to be: "
                         "the ROS side runs with enable_odom_tf=false "
                         "(diffdrive_controller_isaac.yaml), so with this off "
                         "NOTHING broadcasts odom->base_footprint and Nav2/AMCL "
                         "have no TF chain to the robot. Use --no-publish-odom "
                         "only together with enable_odom_tf:=true.")
parser.add_argument("--base-frame", default="base_footprint")
parser.add_argument("--odom-frame", default="odom")
parser.add_argument("--no-imu", action="store_true",
                    help="Skip the PhysX IMU sensor (/imu_data).")
# Lidars / camera are OPT-IN: standalone-script RTX rendering through ROS2
# in Isaac Sim 5.0 has an SDG annotator-registration race that defeats the
# writer attach in headless mode (NVIDIA's own rtx_lidar.py standalone example
# crashes here too). Sensor prims and OG nodes are still built so the topics
# exist; for actual data flow open the produced USD in the Isaac Sim GUI and
# build the sensor OmniGraphs via the menu shortcuts.
parser.add_argument("--lasers", action="store_true",
                    help="Enable the 2 SICK S300 PhysX lidars (/f_scan, /b_scan). "
                         "Pure OmniGraph ray-cast; works headless. 240° FOV, "
                         "0.05–29 m. Ray count = --laser-rays.")
parser.add_argument("--laser-rays", type=int, default=541,
                    help="Rays per SICK lidar over the 240° FOV. Hardware-exact "
                         "default is 541 (matches the Gazebo sensor, which achieves "
                         "full 360° merged coverage at 541). Raise it only to "
                         "experiment with denser reprojected /scan bins.")
parser.add_argument("--rtx-lasers", action="store_true",
                    help="Enable the 2 SICK S300 RTX lidars (/f_scan, /b_scan) "
                         "using the SICK_S300.json profile (IsaacSensorCreateRtxLidar "
                         "+ ROS2RtxLidarHelper). Requires a viewport (not headless). "
                         "RTX gives physically-based reflectance/noise vs. PhysX "
                         "ideal ray-cast. Use --lasers instead for headless runs.")
parser.add_argument("--camera", action="store_true",
                    help="Enable the D435i RGB-D camera "
                         "(/realsense/{color,depth,camera_info}). Requires viewport.")
args, _ = parser.parse_known_args()
# Convert opt-in to the inverse flags the rest of the script uses.
# --rtx-lasers takes precedence: if both are given, PhysX is skipped.
args.no_lasers = not (args.lasers or args.rtx_lasers)
args.no_camera = not args.camera

if not os.path.isfile(args.usd):
    print(f"[mir_isaac_sim] ERROR: USD not found: {args.usd}\n"
          f"  Generate it first with isaac_sim/convert_mir_to_usd.sh", file=sys.stderr)
    sys.exit(1)

from isaacsim import SimulationApp  # noqa: E402

# enable_cameras=True is required for RTX-lidar / camera ROS publishers to work
# even in headless mode — without it the SDG pipeline does not register the
# `PostProcessDispatch*`/`LdrColor*`/`DistanceToImagePlane*` annotators that the
# laser_scan / image writers depend on. Every official ROS2 sensor sample uses
# this flag (see standalone_examples/api/isaacsim.ros2.bridge/*.py).
simulation_app = SimulationApp({
    "renderer": "RaytracedLighting",
    "headless": args.headless,
    "enable_cameras": True,
})

import carb  # noqa: E402
import numpy as np  # noqa: E402
import omni  # noqa: E402
import omni.graph.core as og  # noqa: E402
import omni.kit.commands  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import usdrt.Sdf  # noqa: E402
from isaacsim.core.api import SimulationContext  # noqa: E402
from isaacsim.core.api.objects.ground_plane import GroundPlane  # noqa: E402
from isaacsim.core.utils import extensions, stage, viewports  # noqa: E402
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade  # noqa: E402

# enable ROS2 bridge extension. omni.syntheticdata is what registers the
# `PostProcessDispatchIsaacSimulationGate` annotator that the RTX lidar
# ROS2 LaserScan writer depends on; enable it explicitly so the writer can
# attach (in some standalone runs it does not come up until first render).
extensions.enable_extension("omni.syntheticdata")
extensions.enable_extension("isaacsim.ros2.bridge")
simulation_app.update()
simulation_app.update()

ROBOT_PRIM_PATH = "/World/Robot"

simulation_context = SimulationContext(stage_units_in_meters=1.0)

# ------------------------------------------------------------------- scene
if args.top_down:
    # straight-down view; tiny Y offset gives the camera a defined "up" so the
    # look-down isn't degenerate (eye exactly above target).
    viewports.set_camera_view(eye=np.array([0.0, -0.01, args.top_down_height]),
                              target=np.array([0.0, 0.0, 0.0]))
else:
    viewports.set_camera_view(eye=np.array([3.0, 3.0, 2.0]),
                              target=np.array([0.0, 0.0, 0.5]))

# ground + light so the robot has something to stand on and be visible
GroundPlane(prim_path="/World/GroundPlane", size=50.0, z_position=0.0)
stage_handle = simulation_context.stage
dome = UsdLux.DomeLight.Define(stage_handle, "/World/DomeLight")
dome.CreateIntensityAttr(1000.0)

# diagnostic: closed box of walls around the robot (every direction has a wall)
if args.test_walls:
    _D = 3.0  # wall distance from origin
    _walls = [(_D, 0.0, 0.1, _D), (-_D, 0.0, 0.1, _D),
              (0.0, _D, _D, 0.1), (0.0, -_D, _D, 0.1)]
    for _i, (_x, _y, _hx, _hy) in enumerate(_walls):
        _p = f"/World/TestWall{_i}"
        _cube = UsdGeom.Cube.Define(stage_handle, _p)
        _cube.CreateSizeAttr(2.0)
        _xf = UsdGeom.XformCommonAPI(_cube.GetPrim())
        _xf.SetTranslate(Gf.Vec3d(_x, _y, 0.5))
        _xf.SetScale(Gf.Vec3f(_hx, _hy, 0.5))  # half-extents (cube size 2)
        UsdPhysics.CollisionAPI.Apply(_cube.GetPrim())
    carb.log_warn(f"[mir_isaac_sim] TEST: {len(_walls)} walls at +-{_D} m around robot")
    simulation_app.update()

# optional environment (e.g. the converted Gazebo maze)
if args.world:
    if not os.path.isfile(args.world):
        carb.log_error(f"[mir_isaac_sim] --world not found: {args.world}")
    else:
        stage.add_reference_to_stage(usd_path=args.world, prim_path="/World/Env")
        carb.log_warn(f"[mir_isaac_sim] environment: {args.world}")
        simulation_app.update()

# load the robot USD as a reference under /World/Robot
stage.add_reference_to_stage(usd_path=args.usd, prim_path=ROBOT_PRIM_PATH)
simulation_app.update()


# --------------------------------------------------- find articulation root
def find_articulation_root(root_path):
    """Return the prim path that carries the Articulation Root API."""
    for prim in stage_handle.Traverse():
        p = prim.GetPath().pathString
        if not p.startswith(root_path):
            continue
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            return p
    # fall back to the reference root (SimulationContext will treat it as the
    # articulation if a root API lives anywhere beneath it)
    return root_path


robot_prim_path = args.robot_prim or find_articulation_root(ROBOT_PRIM_PATH)
carb.log_warn(f"[mir_isaac_sim] articulation root: {robot_prim_path}")


# ------------------------------ articulation tuning -------------------------
# Two articulation-level knobs that both target the gripper jitter:
#   * self-collisions: the Robotiq 2F-85 finger links (knuckle / inner_knuckle /
#     finger_tip on both sides) sit very close and, driven together through the
#     PhysX mimic coupling, keep grazing each other. Those internal contacts
#     fight the position drive and make the gripper buzz. Disabling
#     self-collisions turns off collision *between links of the same
#     articulation* only — external objects are separate bodies, so grasping
#     still works — and MoveIt still does its own SRDF self-collision checks.
#   * solver iteration counts: the Robotiq finger linkage is over-constrained by
#     the hard PhysX mimic joints (knuckle/inner_knuckle/finger_tip all geared to
#     the master); the importer's default ~4 position iterations can't converge
#     them each step, so they jitter. More iterations settle the coupling.
_root_prim = stage_handle.GetPrimAtPath(robot_prim_path)
if _root_prim and _root_prim.IsValid():
    _art_api = PhysxSchema.PhysxArticulationAPI.Apply(_root_prim)
    if not args.keep_self_collisions:
        _art_api.CreateEnabledSelfCollisionsAttr().Set(False)
        carb.log_warn("[mir_isaac_sim] self-collisions DISABLED on articulation "
                      f"{robot_prim_path}")
    _art_api.CreateSolverPositionIterationCountAttr().Set(
        int(args.solver_position_iterations))
    _art_api.CreateSolverVelocityIterationCountAttr().Set(
        int(args.solver_velocity_iterations))
    carb.log_warn(f"[mir_isaac_sim] solver iterations: pos="
                  f"{args.solver_position_iterations} "
                  f"vel={args.solver_velocity_iterations}")
else:
    carb.log_warn("[mir_isaac_sim] could not resolve articulation root prim "
                  "for self-collision / solver tuning")


# ------------------------------ depenetration clamp -------------------------
# Nav2 will occasionally wedge the base against a wall corner (the maze's 0.93 m
# gap is the reliable spot) and keep commanding into it while the progress
# checker counts down. PhysX lets the penetration build up and then resolves it
# in a single step at an unbounded velocity — the robot is fired out of the map
# and every downstream consumer (AMCL, the costmaps) goes with it. Clamping the
# depenetration velocity turns that catastrophic ejection into a slow push-out
# the controller can recover from. Applies to every rigid body in the robot,
# since any link can be the one in contact.
# Two-phase on purpose:
#   * BEFORE play() we only apply the schema and author a permissive value.
#     Applying an API schema while the sim is playing re-parses the physics
#     scene mid-step and segfaults Isaac, so the schema has to exist up front.
#   * The real clamp is set AFTER the startup arm-home teleport has settled.
#     set_joint_positions() is instantaneous and leaves transient penetrations
#     that PhysX needs full speed to resolve; clamped from the start, those
#     penetrations persist and the contact forces flip the base within ~50 s of
#     sim time (measured: 212/308 samples tipped, vs 0/308 unclamped).
#     Writing a new *value* to an existing attribute mid-run is safe.
_DEPEN_ATTRS = []
UNCLAMPED_DEPENETRATION = 1.0e6   # PhysX default is effectively unlimited


def init_depenetration_attrs():
    for prim in stage_handle.Traverse():
        if not prim.GetPath().pathString.startswith(ROBOT_PRIM_PATH):
            continue
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        try:
            api = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
            attr = api.CreateMaxDepenetrationVelocityAttr()
        except Exception:  # noqa: BLE001 — fall back to the raw attribute
            attr = prim.CreateAttribute("physxRigidBody:maxDepenetrationVelocity",
                                        Sdf.ValueTypeNames.Float)
        attr.Set(float(UNCLAMPED_DEPENETRATION))
        _DEPEN_ATTRS.append(attr)


def set_max_depenetration_velocity(v):
    if v <= 0.0:
        print("[mir_isaac_sim] depenetration clamp disabled", flush=True)
        return
    for attr in _DEPEN_ATTRS:
        attr.Set(float(v))
    # print, not carb.log_warn: post-play carb messages do not reach stdout.
    # Read one back so the line is proof the value took, not just that we asked.
    back = _DEPEN_ATTRS[0].Get() if _DEPEN_ATTRS else None
    print(f"[mir_isaac_sim] max depenetration velocity {v} m/s -> "
          f"{len(_DEPEN_ATTRS)} rigid bodies (readback {back})", flush=True)


init_depenetration_attrs()


# ------------------------------ gripper armature ----------------------------
# The Robotiq finger inertias are ~1e-5 kg*m^2. A stiff position drive + the hard
# mimic coupling oscillate far faster than the sim step can integrate -> buzzing.
# Adding armature (rotor inertia) on the 6 gripper joints raises their effective
# inertia into the integrable range — the standard PhysX fix for low-inertia
# joint jitter. Set on the USD joint prims before play().
def set_joint_armature(match, armature):
    """Apply armature to joints. `match` is a substring (str) or an explicit set
    of joint names (any iterable)."""
    if armature <= 0.0:
        return
    names = None if isinstance(match, str) else set(match)
    n = 0
    for prim in stage_handle.Traverse():
        if not prim.GetPath().pathString.startswith(ROBOT_PRIM_PATH):
            continue
        nm = prim.GetName()
        if (nm not in names) if names is not None else (match not in nm):
            continue
        if not (prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.Joint)):
            continue
        try:
            api = PhysxSchema.PhysxJointAPI.Apply(prim)
            api.CreateArmatureAttr().Set(float(armature))
        except Exception:  # noqa: BLE001 — fall back to the raw attribute
            prim.CreateAttribute("physxJoint:armature",
                                 Sdf.ValueTypeNames.Float).Set(float(armature))
        n += 1
    carb.log_warn(f"[mir_isaac_sim] armature {armature} -> {n} joints ({match})")


set_joint_armature("robotiq_85_", args.gripper_armature)


# ------------------------------ make the drive wheels velocity-controlled --
# The arm/gripper joints keep their position drives (set by the URDF
# importer); the two MiR drive wheels must follow velocity commands, so we
# switch their angular drive to velocity mode (stiffness 0, damping high).
def set_velocity_drive(joint_substr, damping=1.0e4, max_force=1.0e6):
    for prim in stage_handle.Traverse():
        if not prim.GetPath().pathString.startswith(ROBOT_PRIM_PATH):
            continue
        name = prim.GetName()
        if joint_substr not in name:
            continue
        if not (prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.Joint)):
            continue
        drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
        drive.CreateTypeAttr().Set("force")
        drive.CreateDampingAttr().Set(damping)
        drive.CreateStiffnessAttr().Set(0.0)
        drive.CreateMaxForceAttr().Set(max_force)
        carb.log_warn(f"[mir_isaac_sim] velocity drive -> {prim.GetPath()}")


for wheel in ("left_wheel_joint", "right_wheel_joint"):
    set_velocity_drive(wheel, damping=args.wheel_drive_damping)


# ------------------------------ wheel-ground friction -----------------------
# The URDF gives the wheels mu=300 only inside <gazebo> tags, which Isaac drops
# on import -> the drive wheels sit on the default low-friction ground and SLIP.
# A velocity-driven wheel then spins at the commanded rad/s but the 67 kg base
# moves SLOWER than commanded (cmd_vel speed mismatch) and differential turning
# becomes erratic (one wheel grips, the other slips -> over-turn or no turn).
# Fix: bind a high-friction PhysicsMaterial to the drive-wheel links AND the
# ground, with combine mode "max" so the contact uses the high value.
def _make_friction_material(path, friction):
    mat = UsdShade.Material.Define(stage_handle, path)
    prim = mat.GetPrim()
    pm = UsdPhysics.MaterialAPI.Apply(prim)
    pm.CreateStaticFrictionAttr().Set(float(friction))
    pm.CreateDynamicFrictionAttr().Set(float(friction))
    pm.CreateRestitutionAttr().Set(0.0)
    pxm = PhysxSchema.PhysxMaterialAPI.Apply(prim)
    pxm.CreateFrictionCombineModeAttr().Set("max")
    return mat


def _bind_physics_material(prim, mat):
    binding = UsdShade.MaterialBindingAPI.Apply(prim)
    binding.Bind(mat, UsdShade.Tokens.weakerThanDescendants, "physics")


if args.wheel_friction > 0.0:
    # Bind high friction ONLY to the 2 drive-wheel links, with combine mode
    # "max" so the contact uses the high value even against the default ground.
    # Do NOT bind it to the ground: a high-friction ground would also make the 4
    # passive casters grip hard, and grippy casters cannot slide/scrub during an
    # in-place pivot -> they block rotation. Leaving the ground at default keeps
    # the casters low-friction (free to scrub) while the drive wheels still grip.
    _fric_mat = _make_friction_material("/World/PhysicsMaterials/drive_wheel",
                                        args.wheel_friction)
    _bound = []
    for _wl in ("left_wheel_link", "right_wheel_link"):
        for prim in stage_handle.Traverse():
            if (prim.GetName() == _wl
                    and prim.GetPath().pathString.startswith(ROBOT_PRIM_PATH)):
                _bind_physics_material(prim, _fric_mat)
                _bound.append(_wl)
                break
    carb.log_warn(f"[mir_isaac_sim] drive-wheel friction {args.wheel_friction} "
                  f"(ground left default so casters can pivot) -> {_bound}")


# ------------------------------ settle the chassis --------------------------
# Idle, the base is not perfectly still: it micro-vibrates / wobbles in yaw on
# its free casters (odom twist shows angular ~0.01-0.04 rad/s with all axes
# non-zero). That wobble is small but the ~0.8 m arm amplifies it into visible
# end-effector motion — which reads as "the arm is moving" even though the UR
# joints are steady. Adding linear+angular damping to the base_link rigid body
# bleeds that energy off. It does NOT block commanded driving: diff_cont drives
# the wheels, and a modest chassis damping is just realistic rolling/turn drag.
def set_body_damping(link_name, linear, angular):
    for prim in stage_handle.Traverse():
        if not prim.GetPath().pathString.startswith(ROBOT_PRIM_PATH):
            continue
        if prim.GetName() != link_name:
            continue
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateLinearDampingAttr().Set(float(linear))
        rb.CreateAngularDampingAttr().Set(float(angular))
        carb.log_warn(f"[mir_isaac_sim] chassis damping lin={linear} "
                      f"ang={angular} -> {prim.GetPath()}")
        return
    carb.log_warn(f"[mir_isaac_sim] chassis link '{link_name}' not found "
                  "for damping")


set_body_damping("base_link", args.base_linear_damping, args.base_angular_damping)


# ------------------------------ free the passive caster joints --------------
# The 4 MiR casters (swivel "rotation" joint + rolling "wheel" joint each, 8
# joints total) carry NO <dynamics> in the URDF, so the importer gives every one
# of them a default position drive (stiffness ~100). That LOCKS each caster at
# angle 0 — 8 joints clamping the chassis to the ground like brakes — so even a
# correctly-commanded drive wheel cannot move the robot. Casters must be passive
# free-wheels: zero stiffness so the importer's default position drive no longer
# clamps the chassis to the ground. The rolling wheel joints get zero
# damping/force (truly free). The swivel "rotation" joints instead get a small
# damping: a real MiR caster trails behind its swivel axis
# (caster_wheel_dx = -0.0382 m), so an UNdamped swivel turns solver noise into a
# yaw torque on the chassis, and because the drive wheels are in velocity mode
# (they hold velocity ~0 but provide no heading restoring force) that torque
# integrates into a slow continuous in-place rotation when idle. Light swivel
# damping bleeds that off without re-locking the casters (that earlier warning
# was about the drive wheels' 1e4 damping). NOTE: maxForce must be > 0 or the
# damping has no effect at all.
def set_passive_drive(joint_substr, damping=0.0, max_force=0.0):
    for prim in stage_handle.Traverse():
        if not prim.GetPath().pathString.startswith(ROBOT_PRIM_PATH):
            continue
        if joint_substr not in prim.GetName():
            continue
        if not (prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.Joint)):
            continue
        drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
        drive.CreateTypeAttr().Set("force")
        drive.CreateStiffnessAttr().Set(0.0)
        drive.CreateDampingAttr().Set(float(damping))
        drive.CreateMaxForceAttr().Set(float(max_force))
        carb.log_warn(f"[mir_isaac_sim] passive drive (damping={damping}) "
                      f"-> {prim.GetPath()}")


# swivel joints: light damping (tunable) to stop the idle in-place rotation
set_passive_drive("caster_rotation_joint",
                  damping=args.caster_swivel_damping,
                  max_force=(1.0e3 if args.caster_swivel_damping > 0.0 else 0.0))
# rolling wheel joints: fully free
set_passive_drive("caster_wheel_joint")


# ------------------------------ stiffen the arm/gripper position drives -----
# The URDF importer gives every joint a soft position drive (stiffness ~100),
# which is far too weak for the UR5 to hold against gravity -> the arm droops
# and never tracks the MoveIt target. Raise stiffness/damping on the 6 UR
# joints (and the gripper master) so they hold and follow position commands.
def set_position_drive(joint_names, stiffness, damping, max_force=1.0e7):
    """Stiffen the angular position drive on the named joints. (Targets are set
    later through the Articulation API, which—unlike the raw USD DriveAPI—handles
    the per-joint axis sign correctly; setting targetPosition here drives some UR
    joints to the mirror pose.)"""
    targets = set(joint_names)
    for prim in stage_handle.Traverse():
        if not prim.GetPath().pathString.startswith(ROBOT_PRIM_PATH):
            continue
        if prim.GetName() not in targets:
            continue
        if not (prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.Joint)):
            continue
        drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
        drive.CreateTypeAttr().Set("force")
        drive.CreateStiffnessAttr().Set(float(stiffness))
        drive.CreateDampingAttr().Set(float(damping))
        drive.CreateMaxForceAttr().Set(max_force)
        carb.log_warn(f"[mir_isaac_sim] position drive ({stiffness}/{damping}) "
                      f"-> {prim.GetName()}")


# UR initial pose = ur_description initial_positions.yaml (the ros2_control
# state initial_value), so Isaac and ros2_control agree on the start state.
UR_HOME = {
    "ur_shoulder_pan_joint": 0.0,
    "ur_shoulder_lift_joint": -1.57,
    "ur_elbow_joint": 0.0,
    "ur_wrist_1_joint": -1.57,
    "ur_wrist_2_joint": 0.0,
    "ur_wrist_3_joint": 0.0,
}
set_position_drive(UR_HOME.keys(), stiffness=args.arm_stiffness,
                   damping=args.arm_damping, max_force=args.arm_max_force)
# armature on the 6 UR joints — same low-inertia jitter fix as the gripper, most
# needed on the small-inertia wrist joints.
set_joint_armature(set(UR_HOME.keys()), args.arm_armature)
# the Robotiq master joint (the 5 mimic joints follow it via PhysX coupling)
set_position_drive(["robotiq_85_left_knuckle_joint"],
                   stiffness=args.gripper_stiffness, damping=args.gripper_damping,
                   max_force=1.0e3)
simulation_app.update()

# ------------------------------------------------ build the ROS2 action graph
graph_keys = og.Controller.Keys
nodes = [
    ("OnImpulseEvent", "omni.graph.action.OnImpulseEvent"),
    ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
    ("Context", "isaacsim.ros2.bridge.ROS2Context"),
    ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
    ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
]
connect = [
    ("OnImpulseEvent.outputs:execOut", "PublishJointState.inputs:execIn"),
    ("OnImpulseEvent.outputs:execOut", "PublishClock.inputs:execIn"),
    ("Context.outputs:context", "PublishJointState.inputs:context"),
    ("Context.outputs:context", "PublishClock.inputs:context"),
    ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
    ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
]
set_values = [
    ("PublishJointState.inputs:topicName", args.states_topic),
    ("PublishJointState.inputs:targetPrim", [usdrt.Sdf.Path(robot_prim_path)]),
]

# One Subscribe + ArticulationController chain PER command topic. The 3
# ros2_control hardware components (UR arm, MiR base, Robotiq gripper) each
# publish to their OWN topic, so a single shared /isaac_joint_commands no longer
# has 3 publishers interleaving — which starved the arm of commands. Each
# ArticulationController applies only the joints in its message to the same
# articulation.
# TopicBasedSystem publishes a JointState whose `name` field lists ALL joints of
# the hardware component but whose value arrays (position/velocity/effort) only
# cover the joints that actually have a *command* interface. The MiR base lists
# 10 joints (2 drive wheels + 8 passive casters) yet only the 2 wheels carry a
# velocity command interface -> name has 10 entries, velocity has 2. The Isaac
# ArticulationController requires len(command) == len(jointNames), so it drops
# the whole message and the wheels never move. For such topics we pin an explicit
# jointNames token array (the actually-commanded joints, in command-interface
# declaration order) instead of forwarding the 10-name list from the message.
EXPLICIT_JOINTS = {
    "isaac_base_commands": ["left_wheel_joint", "right_wheel_joint"],
}

for i, topic in enumerate(args.command_topics):
    sub = f"SubscribeJointState{i}"
    art = f"ArticulationController{i}"
    nodes += [
        (sub, "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
        (art, "isaacsim.core.nodes.IsaacArticulationController"),
    ]
    connect += [
        ("OnImpulseEvent.outputs:execOut", f"{sub}.inputs:execIn"),
        ("OnImpulseEvent.outputs:execOut", f"{art}.inputs:execIn"),
        ("Context.outputs:context", f"{sub}.inputs:context"),
        (f"{sub}.outputs:positionCommand", f"{art}.inputs:positionCommand"),
        (f"{sub}.outputs:velocityCommand", f"{art}.inputs:velocityCommand"),
        (f"{sub}.outputs:effortCommand", f"{art}.inputs:effortCommand"),
    ]
    set_values += [
        (f"{sub}.inputs:topicName", topic),
        (f"{art}.inputs:robotPath", robot_prim_path),
    ]
    explicit = next((v for k, v in EXPLICIT_JOINTS.items() if k in topic), None)
    if explicit is not None:
        # pin the commanded joints so they match the short value arrays
        set_values.append((f"{art}.inputs:jointNames", explicit))
        carb.log_warn(f"[mir_isaac_sim] command chain {i}: {topic} -> "
                      f"pinned jointNames {explicit}")
    else:
        # forward the message's own jointNames (lengths already match)
        connect.append((f"{sub}.outputs:jointNames", f"{art}.inputs:jointNames"))
        carb.log_warn(f"[mir_isaac_sim] command chain {i}: subscribes {topic}")

# optional: ground-truth odometry (odom -> base_footprint) straight from Isaac
if args.publish_odom:
    nodes += [
        ("ComputeOdometry", "isaacsim.core.nodes.IsaacComputeOdometry"),
        ("PublishOdometry", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
        ("PublishRawTF", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
    ]
    connect += [
        ("OnImpulseEvent.outputs:execOut", "ComputeOdometry.inputs:execIn"),
        ("ComputeOdometry.outputs:execOut", "PublishOdometry.inputs:execIn"),
        ("ComputeOdometry.outputs:execOut", "PublishRawTF.inputs:execIn"),
        ("Context.outputs:context", "PublishOdometry.inputs:context"),
        ("Context.outputs:context", "PublishRawTF.inputs:context"),
        ("ReadSimTime.outputs:simulationTime", "PublishOdometry.inputs:timeStamp"),
        ("ReadSimTime.outputs:simulationTime", "PublishRawTF.inputs:timeStamp"),
        ("ComputeOdometry.outputs:position", "PublishOdometry.inputs:position"),
        ("ComputeOdometry.outputs:orientation", "PublishOdometry.inputs:orientation"),
        ("ComputeOdometry.outputs:linearVelocity", "PublishOdometry.inputs:linearVelocity"),
        ("ComputeOdometry.outputs:angularVelocity", "PublishOdometry.inputs:angularVelocity"),
        ("ComputeOdometry.outputs:position", "PublishRawTF.inputs:translation"),
        ("ComputeOdometry.outputs:orientation", "PublishRawTF.inputs:rotation"),
    ]
    set_values += [
        ("ComputeOdometry.inputs:chassisPrim", [usdrt.Sdf.Path(robot_prim_path)]),
        ("PublishOdometry.inputs:odomFrameId", args.odom_frame),
        ("PublishOdometry.inputs:chassisFrameId", args.base_frame),
        ("PublishRawTF.inputs:parentFrameId", args.odom_frame),
        ("PublishRawTF.inputs:childFrameId", args.base_frame),
    ]

try:
    og.Controller.edit(
        {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
        {
            graph_keys.CREATE_NODES: nodes,
            graph_keys.CONNECT: connect,
            graph_keys.SET_VALUES: set_values,
        },
    )
except Exception as e:  # noqa: BLE001
    carb.log_error(f"[mir_isaac_sim] failed to build action graph: {e}")
    simulation_app.close()
    sys.exit(1)

simulation_app.update()

# DIAGNOSTIC: read back what each command chain actually wired, so we can tell
# whether the pinned jointNames took effect (carb.log_warn isn't captured in the
# redirected log, so use print).
for i, topic in enumerate(args.command_topics):
    try:
        jn = og.Controller.get(
            og.Controller.attribute(f"/ActionGraph/ArticulationController{i}.inputs:jointNames"))
        print(f"[graph_check] chain {i} ({topic}) ArticulationController jointNames = {list(jn)}",
              flush=True)
    except Exception as e:  # noqa: BLE001
        print(f"[graph_check] chain {i} ({topic}) readback failed: {e}", flush=True)


# =================================================================== SENSORS
# Lasers, IMU and the D435i RGB-D camera. Each block builds on the same /ROS_Sensors
# action graph; the OG nodes tick on render frames (push evaluator) so they keep
# running without us having to drive impulses for them.

def find_prim(name):
    """First prim under /World/Robot whose leaf name matches."""
    for p in stage_handle.Traverse():
        if p.GetName() == name and p.GetPath().pathString.startswith(ROBOT_PRIM_PATH):
            return p
    return None


def unblock_lidar_self_collision(laser_links):
    """Stop the PhysX lidars from ray-casting the robot's OWN body.

    Root cause of the "/scan is missing a section that rotates with the robot"
    bug (diagnosed 2026-06-27): the two SICK S300 origins land at the chassis
    corners (~0.43,0.24 and -0.36,-0.24, z=0.19) which are INSIDE the base
    collision box. Isaac's PhysX RangeSensor ray-casts against EVERY collider
    (Gazebo's ray sensor, by contrast, ignores its own model), so ~260° of each
    240° FOV hits the body at <1 m and never reaches the walls — the merged
    /scan ends up with two fixed empty sectors (measured: each sensor covered
    only ~100° instead of 240°, 287/541 beams returned <1 m).

    Gazebo's behaviour is reproduced here by disabling collision ONLY on the
    colliders that actually cross the horizontal laser plane (z ≈ 0.19) — i.e.
    the chassis body box. Everything else KEEPS its collision: the wheels/casters
    keep the robot on the floor, and the arm + gripper (which sit ABOVE the laser
    plane and never block a horizontal beam) keep collision so manipulation /
    grasping still work. The robot is therefore NOT collisionless — only the
    chassis stops physically bumping walls, which is fine under Nav2 (costmap
    avoidance + wheels keep floor contact). Reprojection is exact: the lidar
    origins stay on the URDF laser_links, so /f_scan + /b_scan reproject with
    ZERO offset error (unlike nudging the sensors outward).
    """
    # The robot's links are INSTANCEABLE; stage.Traverse() does not descend into
    # instance prototypes, so the collision meshes are hidden and uneditable.
    # De-instance the robot subtree first so the geometry colliders become
    # reachable and we can author collisionEnabled overrides on them.
    deinst = 0
    for p in stage_handle.Traverse():
        if p.GetPath().pathString.startswith(ROBOT_PRIM_PATH) and p.IsInstance():
            p.SetInstanceable(False)
            deinst += 1
    print(f"[mir_isaac_sim] self-collision unblock: de-instanced {deinst} robot prim(s)")

    # Laser plane height (both SICK at z≈0.19); a collider only blocks the
    # horizontal beams if its world bbox spans this z.
    xc = UsdGeom.XformCache()
    zs = [xc.GetLocalToWorldTransform(p).ExtractTranslation()[2] for p in laser_links]
    z_lo, z_hi = (min(zs), max(zs)) if zs else (0.19, 0.19)
    # ignoreVisibility=True is ESSENTIAL: Isaac's collision meshes are marked
    # invisible, and a default BBoxCache returns an EMPTY bound for invisible
    # prims -> every collider would be skipped.
    bbc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                            [UsdGeom.Tokens.default_, UsdGeom.Tokens.render,
                             UsdGeom.Tokens.proxy, UsdGeom.Tokens.guide],
                            useExtentsHint=True, ignoreVisibility=True)

    def is_collider(p):
        # Real collision GEOMETRY only — exclude PhysicsJoints (which also carry a
        # physics:collisionEnabled attr but have no geometry / EMPTY bbox).
        if "/joints/" in p.GetPath().pathString:
            return False
        return (p.HasAPI(UsdPhysics.CollisionAPI)
                or p.HasAPI(PhysxSchema.PhysxCollisionAPI))

    def set_collision(p, enabled):
        attr = p.GetAttribute("physics:collisionEnabled")
        if not attr:
            attr = UsdPhysics.CollisionAPI.Apply(p).CreateCollisionEnabledAttr()
        attr.Set(enabled)

    colliders = [p for p in stage_handle.Traverse()
                 if p.GetPath().pathString.startswith(ROBOT_PRIM_PATH) and is_collider(p)]
    print(f"[mir_isaac_sim] self-collision unblock: {len(colliders)} robot geometry "
          f"colliders found; laser plane z=[{z_lo:.3f},{z_hi:.3f}]")

    # Disable collision ONLY on colliders crossing the laser plane (the chassis).
    # Wheels/casters are kept explicitly too (belt-and-suspenders); the arm,
    # gripper and any other geometry above/below the plane keep their collision.
    KEEP = ("wheel", "caster")
    pad = 0.03
    disabled, kept = [], []
    for p in colliders:
        path = p.GetPath().pathString
        rng = bbc.ComputeWorldBound(p).ComputeAlignedRange()
        if rng.IsEmpty():
            kept.append(path)
            continue
        lo, hi = rng.GetMin()[2], rng.GetMax()[2]
        crosses = (lo - pad <= z_hi) and (hi + pad >= z_lo)
        if crosses and not any(k in path.lower() for k in KEEP):
            set_collision(p, False)
            disabled.append(path)
        else:
            kept.append(path)
    print(f"[mir_isaac_sim] self-collision unblock: disabled {len(disabled)} "
          f"laser-plane (chassis) collider(s), kept {len(kept)} "
          f"(wheels/casters/arm/gripper)")
    print(f"[mir_isaac_sim]   disabled: {[p.split('/')[-1] for p in disabled]}")
    if not disabled:
        print("[mir_isaac_sim] WARNING: 0 disabled — no collider crosses the laser "
              "plane; paste this log so the blocking prim can be targeted directly.")
    return disabled


def add_lidar_safe_chassis_collider():
    """Restore the MiR footprint collision below the horizontal lidar plane.

    The imported chassis mesh surrounds both SICK origins, so PhysX ray casts
    see the robot itself.  ``unblock_lidar_self_collision`` therefore disables
    that mesh collider.  Leaving it at that makes the casters and drive wheels
    the first parts to hit a wall; if Nav2 keeps pushing at a narrow entrance,
    those small curved contacts can wedge the articulation and PhysX can eject
    it.  A low box supplies the same 0.89 x 0.58 m XY footprint as Nav2/Gazebo,
    while its 0.14 m top remains safely below the lidar plane at z=0.1914 m.
    """
    base = find_prim("base_link")
    if base is None:
        carb.log_warn("[mir_isaac_sim] chassis proxy skipped: base_link not found")
        return None
    path = base.GetPath().AppendChild("lidar_safe_chassis_collision")
    cube = UsdGeom.Cube.Define(stage_handle, path)
    cube.CreateSizeAttr(2.0)
    xf = UsdGeom.XformCommonAPI(cube.GetPrim())
    # Nav2 footprint: x=-0.39..0.50, y=-0.29..0.29.  Keep the bottom 2 cm
    # above the floor so this bumper cannot add ground drag.
    xf.SetTranslate(Gf.Vec3d(0.055, 0.0, 0.08))
    xf.SetScale(Gf.Vec3f(0.445, 0.29, 0.06))
    cube.CreateVisibilityAttr(UsdGeom.Tokens.invisible)
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    carb.log_warn("[mir_isaac_sim] chassis collision proxy: "
                  "x=-0.39..0.50 y=-0.29..0.29 z=0.02..0.14 "
                  "(below lidar z=0.1914)")
    return cube.GetPrim()


sensor_nodes, sensor_connect, sensor_set = [], [], []

# ---------------------------------------------------------- 2x SICK S300 lidars
# RTX lidars need a render product each (they're cameras under the hood). The
# Replicator writers attach the ROS2 publishers in the SDG pipeline.
# Mounting orientation matches the URDF macro:
#   the sensor is parented to the laser_link so its Z axis points up after the
#   default Isaac lidar Z-up convention. No extra rotation needed because the
#   link frame already matches the URDF.
LIDARS = []
if not args.no_lasers and not args.rtx_lasers:
    # PhysX (physics ray-cast) lidar. Pure-OmniGraph, no render product or SDG
    # pipeline — works headless. SICK S300: 0.05–29 m, ±120° (240° FOV), 541 rays.
    #
    # OnPlaybackTick runs at the 60 Hz simulation rate.  Publishing every tick
    # used to make /f_scan, /b_scan and the merged /scan run at ~50-60 Hz,
    # unlike the Gazebo SICK plugin's 12.5 Hz.  Besides being an inaccurate
    # sensor model, that needlessly loads AMCL, both costmaps and MPPI at the
    # maze's narrow entrance.  A five-tick simulation gate produces 12 Hz,
    # which is the closest integer divisor of 60 Hz to Gazebo's 12.5 Hz.
    #
    # max_range = 30.0 (NOT 29.0) on purpose: the PhysX RangeSensor returns its
    # max_range for a NO-HIT beam (GenericSensor.h: `linearDepth=maxDepth`), so a
    # no-hit reads as a finite point at max_range — Gazebo instead returns +inf.
    # The scan merger reprojects each beam into virtual_laser_link (the sensors
    # sit ~0.49 m off-centre) and then KEEPS any point with range <= its
    # range_max (29.0) while leaving empty bins at +inf. If the lidar max_range
    # were also 29.0, no-hit beams would reproject to ~28.5–29.5 m: the ones just
    # under 29 get kept as a phantom far ring, the rest become inf — an
    # inconsistent partial ring that does NOT match Gazebo. Setting the lidar
    # max_range to 30.0 (> merger range_max 29.0 + the 0.49 m mount offset)
    # guarantees EVERY no-hit reprojects past 29.0 and is dropped -> the bin
    # stays +inf, exactly like Gazebo. Real hits <=29 m are unaffected; the
    # published /scan still reports range_max 29.0 from the merger.
    #
    # Before creating the lidars, stop them from ray-casting the robot's own
    # chassis (the sensor origins sit inside the base collision box). Without
    # this, ~260° of each FOV hits the body and /scan loses two big sectors.
    _laser_links = [lk for lk in (find_prim("front_laser_link"),
                                  find_prim("back_laser_link")) if lk is not None]
    if _laser_links:
        _disabled_chassis = unblock_lidar_self_collision(_laser_links)
        if _disabled_chassis:
            add_lidar_safe_chassis_collider()
    sensor_nodes += [
        ("PhysXLidarGate", "isaacsim.core.nodes.IsaacSimulationGate"),
    ]
    sensor_connect += [
        ("OnPlaybackTick.outputs:tick", "PhysXLidarGate.inputs:execIn"),
    ]
    sensor_set += [
        ("PhysXLidarGate.inputs:step", 5),
    ]
    for idx, (link, topic) in enumerate((("front_laser_link", "f_scan"),
                                         ("back_laser_link", "b_scan"))):
        parent = find_prim(link)
        if parent is None:
            carb.log_warn(f"[mir_isaac_sim] {link} not in USD, skipping {topic}")
            continue
        _, lidar = omni.kit.commands.execute(
            "RangeSensorCreateLidar",
            path="/sick",
            parent=parent.GetPath().pathString,
            min_range=0.05, max_range=30.0,
            draw_points=False, draw_lines=False,
            horizontal_fov=240.0,
            vertical_fov=1.0,            # must be > 0; 1 row -> 2D scan
            horizontal_resolution=240.0 / max(int(args.laser_rays), 2),
            vertical_resolution=1.0,
            rotation_rate=0.0, high_lod=False, yaw_offset=0.0,
            enable_semantics=False,
        )
        lidar_path = lidar.GetPath().pathString
        rd, pub = f"ReadLidar{idx}", f"LaserPub{idx}"
        sensor_nodes += [
            (rd, "isaacsim.sensors.physx.IsaacReadLidarBeams"),
            (pub, "isaacsim.ros2.bridge.ROS2PublishLaserScan"),
        ]
        sensor_connect += [
            ("PhysXLidarGate.outputs:execOut", f"{rd}.inputs:execIn"),
            (f"{rd}.outputs:execOut", f"{pub}.inputs:execIn"),
            ("ContextSensors.outputs:context", f"{pub}.inputs:context"),
            ("ReadSimTimeSensors.outputs:simulationTime", f"{pub}.inputs:timeStamp"),
            (f"{rd}.outputs:azimuthRange", f"{pub}.inputs:azimuthRange"),
            (f"{rd}.outputs:depthRange", f"{pub}.inputs:depthRange"),
            (f"{rd}.outputs:horizontalFov", f"{pub}.inputs:horizontalFov"),
            (f"{rd}.outputs:horizontalResolution", f"{pub}.inputs:horizontalResolution"),
            (f"{rd}.outputs:intensitiesData", f"{pub}.inputs:intensitiesData"),
            (f"{rd}.outputs:linearDepthData", f"{pub}.inputs:linearDepthData"),
            (f"{rd}.outputs:numCols", f"{pub}.inputs:numCols"),
            (f"{rd}.outputs:numRows", f"{pub}.inputs:numRows"),
            (f"{rd}.outputs:rotationRate", f"{pub}.inputs:rotationRate"),
        ]
        sensor_set += [
            (f"{rd}.inputs:lidarPrim", [usdrt.Sdf.Path(lidar_path)]),
            (f"{pub}.inputs:topicName", topic),
            (f"{pub}.inputs:frameId", link),
        ]
        LIDARS.append((topic, link, lidar_path))
        carb.log_warn(f"[mir_isaac_sim] PhysX lidar -> /{topic} (frame {link})")
    carb.log_warn("[mir_isaac_sim] PhysX lidar publish gate: 60 Hz / 5 = 12 Hz "
                  "(Gazebo SICK update_rate: 12.5 Hz)")

# ----------------------------------------- 2x SICK S300 RTX lidars (--rtx-lasers)
# Uses IsaacSensorCreateRtxLidar with the SICK_S300 JSON profile, then publishes
# via ROS2RtxLidarHelper (type=laser_scan). CRITICAL: the render product must be
# created with the RTX-lidar render vars ["GenericModelOutput","RtxSensorMetadata"]
# — the camera-oriented IsaacCreateRenderProduct OG node does NOT set these, so a
# render product from that node carries no lidar data and the helper publishes
# nothing. We therefore build the render product directly with
# rep.create.render_product(...) and feed its .path to the helper, exactly the
# pattern that passes in isaacsim.ros2.bridge test_rtx_sensor.py on this build.
# Requires a viewport; --lasers (PhysX) is the headless-safe alternative.
_RTX_LIDAR_RPS = []  # keep references alive so the render products aren't GC'd
if not args.no_lasers and args.rtx_lasers:
    # Reference the SICK_S300.usda OmniLidar directly instead of going through
    # IsaacSensorCreateRtxLidar(config=...). That command resolves `config` by
    # NAME against SUPPORTED_LIDAR_CONFIGS (USD assets pulled from the asset
    # server); a file path does NOT match, so it silently falls back to a default
    # 3D rotary lidar (elevationDeg = -15°) and IsaacComputeRTXLidarFlatScan then
    # refuses to run ("Lidar prim is not a 2D Lidar"). SICK_S300.usda is a 2D
    # OmniLidar (elevationDeg all 0) carrying OmniSensorGenericLidarCoreAPI, so a
    # direct reference gives the helper a prim it accepts and FlatScan can read.
    _sick_usda = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              "usd", "SICK_S300.usda")
    if not os.path.isfile(_sick_usda):
        carb.log_error(f"[mir_isaac_sim] SICK_S300.usda not found: {_sick_usda}")
    else:
        for idx, (link, topic) in enumerate((("front_laser_link", "f_scan"),
                                             ("back_laser_link", "b_scan"))):
            parent = find_prim(link)
            if parent is None:
                carb.log_warn(f"[mir_isaac_sim] {link} not in USD, skipping RTX {topic}")
                continue
            lidar_path = parent.GetPath().pathString + "/sick_rtx"
            # prim_type MUST be "OmniLidar": add_reference_to_stage defaults to
            # "Xform", and a local "Xform" type opinion overrides the referenced
            # OmniLidar type, so the helper's `GetTypeName() == "OmniLidar"` check
            # fails ("Render product not attached to RTX Lidar").
            stage.add_reference_to_stage(usd_path=_sick_usda, prim_path=lidar_path,
                                         prim_type="OmniLidar")
            lidar_prim = stage_handle.GetPrimAtPath(lidar_path)
            if not lidar_prim or not lidar_prim.IsValid():
                carb.log_error(f"[mir_isaac_sim] failed to reference SICK_S300.usda "
                               f"at {lidar_path}, skipping RTX {topic}")
                continue
            # Belt-and-suspenders: ensure the lidar-core API is applied so the
            # helper's HasAPI("OmniSensorGenericLidarCoreAPI") check also passes
            # even if the referenced apiSchemas don't compose as expected.
            if not lidar_prim.HasAPI("OmniSensorGenericLidarCoreAPI"):
                lidar_prim.AddAppliedSchema("OmniSensorGenericLidarCoreAPI")
            # RTX lidar render product WITH the sensor render vars (see note above)
            rp = rep.create.render_product(
                lidar_path, resolution=(128, 128),
                render_vars=["GenericModelOutput", "RtxSensorMetadata"],
                force_new=True,
            )
            _RTX_LIDAR_RPS.append(rp)
            helper_node = f"LidarHelper{idx}"
            sensor_nodes += [
                (helper_node, "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
            ]
            sensor_connect += [
                ("OnPlaybackTick.outputs:tick", f"{helper_node}.inputs:execIn"),
                ("ContextSensors.outputs:context", f"{helper_node}.inputs:context"),
            ]
            sensor_set += [
                (f"{helper_node}.inputs:renderProductPath", rp.path),
                (f"{helper_node}.inputs:topicName", topic),
                (f"{helper_node}.inputs:frameId", link),
                (f"{helper_node}.inputs:type", "laser_scan"),
            ]
            LIDARS.append((topic, link, lidar_path))
            carb.log_warn(f"[mir_isaac_sim] RTX lidar (SICK_S300) -> /{topic} "
                          f"(frame {link}, rp {rp.path})")

# NOTE: A single 360° lidar at virtual_laser_link publishing /scan directly is
# DELIBERATELY NOT provided. The real MiR100 has TWO SICK S300 scanners
# (front-left + back-right corners); the simulation must keep both and feed the
# ira_laser_tools merger. Replacing them with one centre lidar is forbidden — see
# README_isaac.md ("Two SICK S300 lidars — do NOT replace with one").

# --------------------------------------------------------------- PhysX IMU
if not args.no_imu:
    imu_parent = find_prim("imu_link") or find_prim("imu_frame")
    if imu_parent is None:
        carb.log_warn("[mir_isaac_sim] imu_link / imu_frame not in USD, skipping IMU")
    else:
        _, imu_prim = omni.kit.commands.execute(
            "IsaacSensorCreateImuSensor",
            path="/imu",
            parent=imu_parent.GetPath().pathString,
            sensor_period=1.0 / 50.0,
            translation=Gf.Vec3d(0, 0, 0),
            orientation=Gf.Quatd(1, 0, 0, 0),
        )
        imu_path = imu_prim.GetPath().pathString
        sensor_nodes += [
            ("ReadIMU", "isaacsim.sensors.physics.IsaacReadIMU"),
            ("PublishIMU", "isaacsim.ros2.bridge.ROS2PublishImu"),
        ]
        sensor_connect += [
            ("OnPlaybackTick.outputs:tick", "ReadIMU.inputs:execIn"),
            ("ReadIMU.outputs:execOut", "PublishIMU.inputs:execIn"),
            ("ContextSensors.outputs:context", "PublishIMU.inputs:context"),
            ("ReadSimTimeSensors.outputs:simulationTime", "PublishIMU.inputs:timeStamp"),
            ("ReadIMU.outputs:linAcc", "PublishIMU.inputs:linearAcceleration"),
            ("ReadIMU.outputs:angVel", "PublishIMU.inputs:angularVelocity"),
            ("ReadIMU.outputs:orientation", "PublishIMU.inputs:orientation"),
        ]
        sensor_set += [
            ("ReadIMU.inputs:imuPrim", [usdrt.Sdf.Path(imu_path)]),
            ("PublishIMU.inputs:topicName", "imu_data"),
            ("PublishIMU.inputs:frameId", "imu_frame"),
        ]
        carb.log_warn(f"[mir_isaac_sim] PhysX IMU -> /imu_data (frame imu_frame)")

# --------------------------------------------------------------- D435i RGB-D
# Two image streams + one camera_info, all sharing the same render product on a
# camera prim placed at /World/Robot/realsense_link. Frames match the URDF.
if not args.no_camera:
    rs_parent = find_prim("realsense_link")
    if rs_parent is None:
        carb.log_warn("[mir_isaac_sim] realsense_link not in USD, skipping camera")
    else:
        cam_path = rs_parent.GetPath().pathString + "/d435i_camera"
        cam_prim = UsdGeom.Camera(stage_handle.DefinePrim(cam_path, "Camera"))
        # D435i optical convention: +Z forward, +X right, +Y down. URDF parents
        # the color/depth optical frames with a -90deg rotation about each axis;
        # here we sit on realsense_link and apply the same color-optical-frame
        # transform so the published image matches the TF tree.
        xform = UsdGeom.XformCommonAPI(cam_prim)
        xform.SetRotate((-90, 0, -90), UsdGeom.XformCommonAPI.RotationOrderXYZ)
        # Intel D435 intrinsics (approx.): 69deg HFOV @ 1920x1080 ~> focal ~1.39mm
        # on a 36mm sensor model. Replicator/Isaac uses a 24mm camera by default;
        # pick aperture so horizontalAperture / focalLength matches tan(HFOV/2)*2.
        cam_prim.GetFocalLengthAttr().Set(1.93)
        cam_prim.GetHorizontalApertureAttr().Set(2.682)  # 69deg HFOV
        cam_prim.GetVerticalApertureAttr().Set(1.509)   # 42.5deg VFOV
        cam_prim.GetClippingRangeAttr().Set((0.1, 100.0))

        # Same shortcut pattern as the lidars: fire IsaacCreateRenderProduct
        # off RunOnce.outputs:step, then the camera helpers consume the produced
        # renderProductPath. CameraInfoHelper attaches to the same render product.
        sensor_nodes += [
            ("CreateRP_RGB", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
            ("CamHelperRGB", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("CamHelperInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ("CamHelperDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
        ]
        sensor_connect += [
            ("RunOnce.outputs:step", "CreateRP_RGB.inputs:execIn"),
            ("CreateRP_RGB.outputs:execOut", "CamHelperRGB.inputs:execIn"),
            ("CreateRP_RGB.outputs:execOut", "CamHelperInfo.inputs:execIn"),
            ("CreateRP_RGB.outputs:execOut", "CamHelperDepth.inputs:execIn"),
            ("CreateRP_RGB.outputs:renderProductPath", "CamHelperRGB.inputs:renderProductPath"),
            ("CreateRP_RGB.outputs:renderProductPath", "CamHelperInfo.inputs:renderProductPath"),
            ("CreateRP_RGB.outputs:renderProductPath", "CamHelperDepth.inputs:renderProductPath"),
            ("ContextSensors.outputs:context", "CamHelperRGB.inputs:context"),
            ("ContextSensors.outputs:context", "CamHelperInfo.inputs:context"),
            ("ContextSensors.outputs:context", "CamHelperDepth.inputs:context"),
        ]
        sensor_set += [
            ("CreateRP_RGB.inputs:cameraPrim", [usdrt.Sdf.Path(cam_path)]),
            ("CreateRP_RGB.inputs:width", 640),
            ("CreateRP_RGB.inputs:height", 480),
            ("CamHelperRGB.inputs:topicName", "realsense/color/image_raw"),
            ("CamHelperRGB.inputs:frameId", "realsense_color_optical_frame"),
            ("CamHelperRGB.inputs:type", "rgb"),
            ("CamHelperInfo.inputs:topicName", "realsense/color/camera_info"),
            ("CamHelperInfo.inputs:frameId", "realsense_color_optical_frame"),
            ("CamHelperDepth.inputs:topicName", "realsense/depth/image_rect_raw"),
            ("CamHelperDepth.inputs:frameId", "realsense_depth_optical_frame"),
            ("CamHelperDepth.inputs:type", "depth"),
        ]
        carb.log_warn("[mir_isaac_sim] D435i camera -> /realsense/{color,depth,camera_info}")

# Build the sensor graph if anything was registered. The IMU & camera ride on a
# push-tick action graph so they fire every frame; the RTX lidars are driven by
# the SDG pipeline through replicator and need no node here.
if sensor_nodes:
    sensor_nodes = [
        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
        ("RunOnce", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
        ("ContextSensors", "isaacsim.ros2.bridge.ROS2Context"),
        ("ReadSimTimeSensors", "isaacsim.core.nodes.IsaacReadSimulationTime"),
    ] + sensor_nodes
    sensor_connect = [
        ("OnPlaybackTick.outputs:tick", "RunOnce.inputs:execIn"),
    ] + sensor_connect
    sensor_graph = None
    # Build render-product nodes with enabled=False so they don't try to attach
    # their replicator writers during evaluate_sync (the SDG annotators are not
    # registered yet at that point — they only come up after play() renders a
    # frame). After play() + warmup frames we'll enable them. Pattern from
    # carter_stereo.py standalone example.
    sensor_set = sensor_set + [
        (name + ".inputs:enabled", False)
        for name, ntype in sensor_nodes
        if ntype == "isaacsim.core.nodes.IsaacCreateRenderProduct"
    ]
    try:
        (sensor_graph, _, _, _) = og.Controller.edit(
            {"graph_path": "/ROS_Sensors", "evaluator_name": "execution"},
            {
                graph_keys.CREATE_NODES: sensor_nodes,
                graph_keys.CONNECT: sensor_connect,
                graph_keys.SET_VALUES: sensor_set,
            },
        )
        carb.log_warn(f"[mir_isaac_sim] sensor graph built ({len(sensor_nodes)} nodes)")
    except Exception as e:  # noqa: BLE001
        carb.log_error(f"[mir_isaac_sim] failed to build sensor graph: {e}")

simulation_app.update()

# ----------------------------------------------------------------- run loop
simulation_context.initialize_physics()
simulation_context.play()


# Start the arm AT its home pose instead of letting it droop. The importer's
# joint state has an offset vs the URDF, so a drooped arm reads ~3 rad away from
# ros2_control's initial_value; when the controllers activate they then yank the
# arm across that gap fast enough to flip the whole MiR base. We TELEPORT the
# articulation to home (set_joint_positions = instantaneous, no momentum, so the
# base can't be kicked) and set the drive targets to hold it. ros2_control then
# reads a steady home pose and holds it -> no lurch. (rclpy isn't usable here:
# Isaac runs Python 3.11, Humble's rclpy C-ext is 3.10.)
if not args.no_arm_home:
    try:
        from isaacsim.core.prims import SingleArticulation
        from isaacsim.core.utils.types import ArticulationAction
        _art = SingleArticulation(prim_path=robot_prim_path)
        _art.initialize()
        _dofs = list(_art.dof_names)
        _home = np.array([UR_HOME.get(n, 0.0) for n in _dofs])
        _pos0, _quat0 = _art.get_world_pose()   # straight spawn orientation
        _art.set_joint_positions(_home)
        _art.apply_action(ArticulationAction(joint_positions=_home))
        for _ in range(30):
            simulation_context.step(render=not args.headless)
        # During this settling the free-caster base can pick up a small yaw and
        # the strong wheel velocity-drive (damping) can snap it -> the robot
        # "jumps crooked". Snap the base orientation back to the straight spawn
        # orientation (keep the settled position), zero velocities, re-settle.
        try:
            _posN, _ = _art.get_world_pose()
            _art.set_world_pose(_posN, _quat0)
            _art.set_joint_velocities(np.zeros(len(_dofs)))
            _art.apply_action(ArticulationAction(joint_positions=_home))
            for _ in range(15):
                simulation_context.step(render=not args.headless)
            carb.log_warn("[mir_isaac_sim] base orientation reset to straight")
        except Exception as _e2:  # noqa: BLE001
            carb.log_warn(f"[mir_isaac_sim] base orient reset skipped: {_e2}")
        carb.log_warn("[mir_isaac_sim] arm teleported to home pose")
    except Exception as e:  # noqa: BLE001
        carb.log_warn(f"[mir_isaac_sim] arm home init skipped: {e}")

# Now that the teleport's transient penetrations have been resolved at full
# speed, clamp depenetration for the rest of the run. See the function's
# definition for why this protects against Nav2 wedging the base into a corner.
set_max_depenetration_velocity(args.max_depenetration_velocity)

# DIAGNOSTIC: drive the wheels directly through the articulation API (the same
# path the arm-home teleport uses, which is known to work) to isolate whether a
# stationary base is a drive problem or an OmniGraph ArticulationController
# problem. Spins both drive wheels at a fixed velocity for ~90 steps and reports
# how far each rotated. If they move here but not via /isaac_base_commands, the
# OmniGraph velocity path is at fault; if they don't move here either, the wheel
# DriveAPI/velocity-drive setup is.
if args.wheel_test:
    try:
        from isaacsim.core.prims import SingleArticulation
        from isaacsim.core.utils.types import ArticulationAction
        _a = SingleArticulation(prim_path=robot_prim_path)
        _a.initialize()
        _dn = list(_a.dof_names)
        _li = _dn.index("left_wheel_joint")
        _ri = _dn.index("right_wheel_joint")
        _p0 = _a.get_joint_positions()
        print(f"[wheel_test] dof idx left={_li} right={_ri}; "
              f"start pos L={_p0[_li]:.3f} R={_p0[_ri]:.3f}", flush=True)
        _act = ArticulationAction(joint_velocities=np.array([5.0, 5.0]),
                                  joint_indices=np.array([_li, _ri]))
        for _ in range(90):
            _a.apply_action(_act)
            simulation_context.step(render=not args.headless)
        _p1 = _a.get_joint_positions()
        print(f"[wheel_test] after 90 steps @5rad/s: "
              f"L {_p0[_li]:.3f}->{_p1[_li]:.3f} (d={_p1[_li]-_p0[_li]:.3f})  "
              f"R {_p0[_ri]:.3f}->{_p1[_ri]:.3f} (d={_p1[_ri]-_p0[_ri]:.3f})", flush=True)
        print("[wheel_test] if d~0 -> velocity DRIVE is broken; "
              "if d large -> OmniGraph ArticulationController is the issue", flush=True)
    except Exception as e:  # noqa: BLE001
        import traceback
        print(f"[wheel_test] failed: {e}\n{traceback.format_exc()}", flush=True)

# Warm up the SDG pipeline for several rendered frames FIRST. Once a real render
# frame has gone through, omni.syntheticdata registers the gate annotators
# (PostProcessDispatchIsaacSimulationGate, LdrColorSDIsaacConvertRGBAToRGB,
# DistanceToImagePlaneSDIsaacPassthroughImagePtr, IsaacComputeRTXLidarFlatScan)
# that the ROS2 writers depend on. Attaching before this fails permanently.
# (Pattern: carter_stereo.py / rtx_lidar.py.) Needs a viewport, i.e. NOT headless.
if sensor_nodes or LIDARS:
    for _ in range(20):
        simulation_context.step(render=True)

    # camera: enable the IsaacCreateRenderProduct OG nodes
    for name, ntype in sensor_nodes:
        if ntype == "isaacsim.core.nodes.IsaacCreateRenderProduct":
            try:
                og.Controller.set(
                    og.Controller.attribute(f"/ROS_Sensors/{name}.inputs:enabled"), True)
                carb.log_warn(f"[mir_isaac_sim] enabled {name}")
            except Exception as e:  # noqa: BLE001
                carb.log_warn(f"[mir_isaac_sim] enable {name}: {e}")

    # (PhysX lidars need no writer attach — they publish via OG nodes.)
    for _ in range(5):
        simulation_context.step(render=not args.headless)

_extra = []
if LIDARS:
    _extra += [f"/{t}" for t, *_ in LIDARS]
if not args.no_imu:
    _extra.append("/imu_data")
if not args.no_camera:
    _extra.append("/realsense/{color,depth}")
print("[mir_isaac_sim] running. Publishing /{0}, /clock{2}; subscribing {1}".format(
    args.states_topic, ", ".join("/" + t for t in args.command_topics),
    (" + " + " ".join(_extra)) if _extra else ""), flush=True)
carb.log_warn("[mir_isaac_sim] entering run loop")

_base_idx = next((i for i, t in enumerate(args.command_topics) if "base" in t), None)
_dbg = 0

while simulation_app.is_running():
    simulation_context.step(render=True)
    # tick the ROS2 publish/subscribe nodes once per frame
    og.Controller.set(
        og.Controller.attribute("/ActionGraph/OnImpulseEvent.state:enableImpulse"), True
    )
    # DIAGNOSTIC: dump what EVERY command chain receives, so the working arm can
    # be compared against the stationary base.
    _dbg += 1
    if args.wheel_test and _dbg % 60 == 0:
        for _i, _t in enumerate(args.command_topics):
            try:
                _s = f"/ActionGraph/SubscribeJointState{_i}"
                pc = list(og.Controller.get(og.Controller.attribute(f"{_s}.outputs:positionCommand")))
                vc = list(og.Controller.get(og.Controller.attribute(f"{_s}.outputs:velocityCommand")))
                jn = list(og.Controller.get(og.Controller.attribute(f"{_s}.outputs:jointNames")))
                print(f"[rt] chain{_i} {_t}: jn={len(jn)} pos={len(pc)} vel={len(vc)} "
                      f"velvals={vc}", flush=True)
            except Exception as e:  # noqa: BLE001
                print(f"[rt] chain{_i} readback failed: {e}", flush=True)

simulation_context.stop()
simulation_app.close()
