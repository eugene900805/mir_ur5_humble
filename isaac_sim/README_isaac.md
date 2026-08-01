# MiR100 + UR5 + Robotiq85 — Isaac Sim back-end (replaces Gazebo)

This integrates **NVIDIA Isaac Sim 5.0** as the physics/rendering simulator for
the `mir_ur5_humble` robot, in place of Gazebo Classic, while keeping the
**ros2_control + MoveIt2** stack unchanged. Isaac Sim drives the robot
articulation; ROS talks to it through the
[`topic_based_ros2_control`](https://github.com/PickNikRobotics/topic_based_ros2_control)
hardware interface — the same pattern NVIDIA uses in its MoveIt sample.

```
        ┌─────────────────────────── Isaac Sim (mir_isaac_sim.py) ───────────────────────────┐
        │  USD articulation /World/Robot   +   ROS2 OmniGraph bridge                          │
        │     publishes  /isaac_joint_states (sensor_msgs/JointState, all joints)             │
        │     publishes  /clock             (rosgraph_msgs/Clock)                             │
        │     subscribes /isaac_arm_commands     ──► ArticulationController  (UR arm)          │
        │     subscribes /isaac_base_commands    ──► ArticulationController  (MiR wheels)      │
        │     subscribes /isaac_gripper_commands ──► ArticulationController  (Robotiq)         │
        └───────▲───────────────────────────────────────────────────────┬────────────────────┘
                │ joint states                                            │ joint commands (3 topics)
                │                                                         ▼
        ┌───────┴─────────────────────────────────────────────────────────────────────────────┐
        │  ros2_control_node (controller_manager)                                              │
        │    hardware = 3x topic_based_ros2_control/TopicBasedSystem (sim_isaac:=true)          │
        │      UR -> /isaac_arm_commands   MiR -> /isaac_base_commands   Robotiq -> /...gripper │
        │    controllers: joint_broadcaster, diff_cont,                                         │
        │                 joint_trajectory_controller, gripper_position_controller             │
        └───────▲───────────────────────────────────────────────────────┬─────────────────────┘
                │ joint_states                                            │ FollowJointTrajectory / GripperCommand
                ▼                                                         │
        robot_state_publisher → /tf                              MoveIt2 move_group + RViz
```

### Why three command topics (not one)

The robot has **three** `ros2_control` hardware components (UR arm, MiR base,
Robotiq gripper), each a `TopicBasedSystem`. If they all publish to a single
`/isaac_joint_commands`, the three C++ publishers at 100 Hz interleave on that
topic and the bridge — which applies the *latest* received message each sim
frame — keeps getting base/gripper messages and **starves the arm of commands**,
so the arm never moves and MoveIt aborts with `PATH_TOLERANCE_VIOLATED`. (This
only shows up in the full stack; publishing the same messages from one node
works fine.) Giving each component its **own** command topic, with one
`SubscribeJointState → ArticulationController` chain per topic in the bridge,
removes the contention. Joint **states** stay on a single `/isaac_joint_states`
(one publisher, many subscribers — no contention).

## What changed vs. the Gazebo setup

| File | Change |
|------|--------|
| `ur_description/urdf/ur.ros2_control.xacro` | added `sim_isaac` → `topic_based_ros2_control/TopicBasedSystem` |
| `ur_description/urdf/ur_macro.xacro` | plumb `sim_isaac` / isaac topics through |
| `mir_description/urdf/include/mir_100_v1.urdf.xacro` | declare `sim_isaac` arg; UR → `/isaac_arm_commands`, Robotiq → `/isaac_gripper_commands` |
| `mir_description/urdf/mir.urdf.xacro` | MiR base `ros2_control` switches Gazebo↔Isaac (→ `/isaac_base_commands`); Gazebo plugin gated by `sim_gazebo` |
| `robotiq_gripper.ros2_control.xacro` | already supported `sim_isaac` (upstream) |
| `mir_description/launch/mir_isaac.launch.py` | **new** — ROS side of the Isaac integration |
| `isaac_sim/` | **new** — USD, converter, Isaac Sim bridge script |

The Gazebo path is untouched: `sim_isaac` defaults to `false`, so
`ros2 launch mir_gazebo mobile_manipulator.launch.py` still runs Gazebo.

## Joint-name match (why it works)

The bridge matches joints **by name** between ROS and Isaac. The USD is
regenerated from *this repo's* xacro, so the articulation joints are exactly the
ROS names: `ur_shoulder_pan_joint … ur_wrist_3_joint`, `left_wheel_joint`,
`right_wheel_joint`, `robotiq_85_left_knuckle_joint` (+ 5 mimic-coupled gripper
joints), and the caster joints. Articulation root: `/World/Robot/base_footprint`.

---

## 1. Prerequisites

- Isaac Sim 5.0 built at `/mnt/data/IsaacSim` (provides `python.sh`).
- Isaac Lab in conda env `env_isaaclab_opt` (only needed to *regenerate* the USD).
- ROS 2 Humble with `ros-humble-topic-based-ros2-control`, `ros-humble-moveit`,
  `ros2_control`, `ros2_controllers`.

## 2. Build the ROS workspace

```bash
cd /mnt/data/mir_isaac
# system python for ROS (not the conda one):
PATH=/usr/bin:/bin:/opt/ros/humble/bin:$PATH PYTHONPATH= \
  colcon build --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

(If you only want the Isaac path, the minimal set is
`mir_description ur_description robotiq_description realsense2_description
ur_moveit_config`.)

## 3. (Optional) Regenerate the USD

A ready-made USD is already at `isaac_sim/usd/mir_isaac.usd`. To rebuild it from
the xacro (e.g. after changing the robot):

```bash
conda activate env_isaaclab_opt
cd /mnt/data/mir_isaac
# 3a. xacro → URDF (Isaac hardware interface selected)
xacro install/mir_description/share/mir_description/urdf/mir.urdf.xacro \
     sim_isaac:=true sim_gazebo:=false ur_type:=ur5 > isaac_sim/mir_isaac_raw.urdf
# 3b. make the URDF self-contained & importer-safe
python isaac_sim/make_isaac_urdf.py isaac_sim/mir_isaac_raw.urdf isaac_sim/mir_isaac.urdf
# 3c. URDF → USD (Robotiq mimic joints preserved)
python isaac_sim/convert_to_usd.py
# 3d. fix the importer's .dae visual orientation (Isaac Sim 5.0 bug)
python isaac_sim/fix_dae_orientation.py isaac_sim/usd/configuration/mir_isaac_base.usd
# 3e. repair the broken Robotiq left-chain mimic joint (inf limit / missing referenceJoint)
python isaac_sim/fix_mimic_limits.py
```

Do re-run this after any change to the shared xacro — the USD is a snapshot, and
a stale one drifts silently. Example: the MiR100 spec fix raised the wheel joint
limit 20 → 24 rad/s, and until the USD was rebuilt PhysX still capped the wheels
at `physxJoint:maxJointVelocity = 1145.9 °/s` (20 rad/s = 1.25 m/s) while ROS
believed it could command 1.5 m/s.

The maze environment is a separate stage built from the Gazebo world, so the two
simulators drive through identical geometry:

```bash
python isaac_sim/convert_maze_to_usd.py     # mir_gazebo/worlds/include/maze/model.sdf -> usd/maze.usd
```

See `../mir_ur5/README.md` for the full background on each fix.

## 4. Run

**Terminal 1 — Isaac Sim** (publishes `/clock`, `/isaac_joint_states`; subscribes `/isaac_{arm,base,gripper}_commands`):

```bash
cd /mnt/data/IsaacSim/_build/linux-x86_64/release
./python.sh /mnt/data/mir_isaac/src/mir_ur5_humble/isaac_sim/mir_isaac_sim.py
#   add --headless to run without a GUI
#   odom→base_footprint + /odom (Isaac ground truth) are published by DEFAULT;
#   --no-publish-odom turns them off, but then you must also flip the ROS side
#   to enable_odom_tf:=true or nothing publishes that transform at all
```

Or bring up both halves with one command (`launch_isaac:=true` starts
`mir_isaac_sim.py` as a child process of the launch, matching the Gazebo
workflow):

```bash
ros2 launch mir_description mir_isaac.launch.py launch_isaac:=true world:=maze
```

**Terminal 2 — ROS control + MoveIt + RViz:**

```bash
cd /mnt/data/mir_isaac && source install/setup.bash
ros2 launch mir_description mir_isaac.launch.py
#   launch_moveit:=false  launch_rviz:=false   to drop pieces
```

Plan in the RViz MotionPlanning panel and hit **Plan & Execute** — the arm moves
in Isaac Sim. Drive the base with:

```bash
ros2 topic pub /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.3}}"
```

Open/close the gripper (single master joint, 5 joints follow via PhysX mimic):

```bash
ros2 action send_goal /gripper_position_controller/gripper_cmd \
  control_msgs/action/GripperCommand "{command: {position: 0.7}}"
```

## 5. Sensors

| Sensor | ROS topic | Status |
|---|---|---|
| Joint states (all 22 movable joints) | `/isaac_joint_states` | ✅ ~50 Hz |
| Sim clock | `/clock` | ✅ |
| IMU (`imu_link`) | `/imu_data` (`sensor_msgs/Imu`) | ✅ ~50 Hz |
| 2× SICK S300 lidars (PhysX) | `/f_scan`, `/b_scan` (`sensor_msgs/LaserScan`) | ✅ `--lasers`, ~12 Hz (headless OK) |
| 2× SICK S300 lidars (RTX) | `/f_scan`, `/b_scan` (`sensor_msgs/LaserScan`) | ⚠️ `--rtx-lasers`, needs viewport |
| Intel D435i RGB-D (RTX) | `/realsense/{color/image_raw, depth/image_rect_raw, color/camera_info}` | ⚠️ `--camera`, needs viewport |

> ### ⛔ Two SICK S300 lidars — do NOT replace with one
>
> The real MiR100 carries **two** SICK S300 safety scanners, mounted at the
> **front-left** and **back-right** corners, each with a 240° field of view. The
> simulation **must** keep both (`/f_scan` + `/b_scan`) and merge them through
> `ira_laser_tools` into `/scan`, exactly like the Gazebo stack.
>
> **Replacing the two scanners with a single 360° lidar at the robot centre
> (`virtual_laser_link`) is STRICTLY FORBIDDEN.** Even though one centre lidar
> looks like an easy way to get "full coverage", it is not a faithful model of
> the hardware: it removes the real corner mounting parallax, the per-sensor
> blind sectors, and the front/back `/f_scan` `/b_scan` topics the rest of the
> stack (and any safety logic) depends on. If coverage looks incomplete, fix the
> **two-sensor** path (sensor pose/orientation in the USD, or the merger
> parameters) — never substitute a single lidar.

Lidars and the camera are **opt-in**:

```bash
# Headless-safe: PhysX lidar (pure ray-cast, no render product)
python isaac_sim/mir_isaac_sim.py --lasers

# Physically-based RTX lidar (needs viewport, not headless)
python isaac_sim/mir_isaac_sim.py --rtx-lasers

# Camera (needs viewport)
python isaac_sim/mir_isaac_sim.py --rtx-lasers --camera
```

### Two lidar modes

#### PhysX lidar (`--lasers`) — headless OK

Pure-OmniGraph ray-cast: `RangeSensorCreateLidar → IsaacReadLidarBeams →
ROS2PublishLaserScan`. No render product, no SDG pipeline. SICK S300 parameters:
0.05–29 m, ±120° (240° FOV), 541 samples. Works headless and is the default
choice for Nav2 / SLAM runs. A simulation gate publishes every fifth 60 Hz tick
(12 Hz), matching Gazebo's 12.5 Hz SICK update rate without flooding AMCL,
costmaps and MPPI with duplicate scans.

The imported chassis mesh encloses the two corner-mounted ray origins, so its
collider is excluded from lidar queries. The script restores physical wall
contact with an invisible, low-profile `0.89 × 0.58 m` chassis collision proxy
below the laser plane. This keeps the Gazebo/Nav2 footprint while preventing
the scanners from seeing the robot itself and prevents wheel/caster-only wall
contacts from wedging the articulation at the maze entrance.

#### RTX lidar (`--rtx-lasers`) — needs viewport

Uses `IsaacSensorCreateRtxLidar` with the `SICK_S300.json` profile (placed in
`isaac_sim/usd/` and also copied to Isaac Sim's built-in SICK config directory).
The sensor prim is an `OmniLidar` with `OmniSensorGenericLidarCoreAPI`; the
ROS2 `LaserScan` is published by an `isaacsim.ros2.bridge.ROS2RtxLidarHelper`
OG node (`type = laser_scan`).

**Important — render-product render vars.** The helper needs a render product
created with the RTX-lidar render vars
`["GenericModelOutput", "RtxSensorMetadata"]`. The camera-oriented
`IsaacCreateRenderProduct` OG node does **not** set these, so a render product
from that node carries no lidar data and the helper publishes nothing (this was
the original "RTX lidar doesn't work" symptom). We instead build the render
product directly in Python:

```python
rp = rep.create.render_product(
    lidar_path, resolution=(128, 128),
    render_vars=["GenericModelOutput", "RtxSensorMetadata"], force_new=True)
# then: ROS2RtxLidarHelper.inputs:renderProductPath = rp.path
```

This is the exact pattern that passes in
`isaacsim.ros2.bridge` `test_rtx_sensor.py::_test_rtx_lidar_laser_scan` on this
Isaac Sim 5.0 build.

RTX lidar benefits: physically-based reflectance model, configurable range noise
(`rangeAccuracyM = 0.029 m`), realistic timing (13 Hz scan rate, 541 rays fired
sequentially). Drawback: requires a viewport and a GPU capable of ray tracing —
use `--lasers` (PhysX) for headless SLAM/Nav runs.

**SICK S300 mounting on MiR100**: the two sensors are at the *front-left* and
*back-right* corners of the chassis (not purely front/back). Each sensor's
240° scan covers the adjacent sides; together they provide ~300° effective
coverage with a ~60° blind spot at each remaining corner.

### Camera (still RTX → needs a viewport)

The D435i RGB-D still uses the RTX/render path, so `--camera` only produces data
when run with a **viewport** (not headless) on a desktop session. If you only
need navigation, you don't need the camera — `--lasers` alone (headless) is
enough. To get images, drop `--headless`; if attach still fails, build the
camera graph from the Isaac Sim GUI *Tools → ROS2 OmniGraphs → ROS2 Camera Graph*
and save the USD.

### Quick checks

```bash
ros2 topic hz /isaac_joint_states          # ~50 Hz from Isaac
ros2 topic hz /imu_data                    # ~50 Hz, frame_id = imu_frame
ros2 topic echo /clock --once              # sim time present
ros2 control list_controllers              # all 4 active
ros2 control list_hardware_interfaces      # TopicBasedSystem interfaces
```

## 6. Mapping & Navigation (SLAM / Nav2)

The repo's `mir_navigation` stack is simulator-agnostic and works against the
Isaac back-end. With the PhysX lidar (§5) the lasers publish **headless**, so the
whole nav stack runs without a viewport.

| Nav input | source | ready? |
|---|---|---|
| `/clock` | Isaac | ✅ |
| `/odom` + `odom→base_footprint` TF | Isaac ground truth (on by default; `diff_cont` has `enable_odom_tf:false`) | ✅ |
| robot `/tf` | robot_state_publisher | ✅ |
| `/f_scan`, `/b_scan` | Isaac PhysX lidars (`--lasers`) | ✅ |
| `/scan` | `ira_laser_tools` merge of `/f_scan` + `/b_scan` | merger |

One-time setup:

```bash
# scan merger (declared in mir_robot/ros2.repos); needs pcl_ros
sudo apt install -y ros-humble-pcl-ros ros-humble-pcl-conversions
git clone https://github.com/relffok/ira_laser_tools.git    # into the ws root
colcon build --packages-select ira_laser_tools mir_navigation mir_msgs
# nav runtime deps:
sudo apt install -y ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup
```

Run order (each in its own terminal, `source install/setup.bash` first):

```bash
# 1) Isaac with lasers (headless is fine — PhysX lidar)
python isaac_sim/mir_isaac_sim.py --headless --lasers --world isaac_sim/usd/maze.usd

# 2) control + MoveIt
ros2 launch mir_description mir_isaac.launch.py

# 3) merge the two SICK scans -> /scan
ros2 launch mir_description mir_isaac_scan_merger.launch.py

# 4a) build a map (SLAM):
ros2 launch mir_navigation mapping.py use_sim_time:=true \
    slam_params_file:=$(ros2 pkg prefix mir_navigation)/share/mir_navigation/config/mir_mapping_async_sim.yaml
# 4b) ...or localize against the existing maze map + navigate:
ros2 launch mir_navigation amcl.py use_sim_time:=true \
    map:=$(ros2 pkg prefix mir_navigation)/share/mir_navigation/maps/maze.yaml
ros2 launch mir_navigation navigation.py use_sim_time:=true \
    cmd_vel_w_prefix:=/diff_cont/cmd_vel_unstamped
```

Sanity check before nav: `ros2 topic hz /scan` should be ~12 Hz.

## Notes

- **use_sim_time** is `true` everywhere; Isaac is the `/clock` source. Start
  Isaac first and keep it running — restarting it resets `/clock`, which makes
  a live ROS stack throw "jump back in time" and can crash RViz. If you must
  restart Isaac, restart the ROS launches too.
- The drive wheels are switched to **velocity** drive in `mir_isaac_sim.py`;
  the arm/gripper keep position drives. `diff_cont` writes wheel velocities,
  which reach Isaac through `/isaac_base_commands`.
- Odometry: **Isaac** publishes `odom→base_footprint` + `/odom` (ground truth),
  and the ROS side is configured to match — `diffdrive_controller_isaac.yaml`
  sets `enable_odom_tf: false` so the two do not both broadcast the same
  transform. Both halves of that pairing are on by default; `--no-publish-odom`
  turns the Isaac side off and then **you must also set `enable_odom_tf: true`**,
  or nothing broadcasts `odom→base_footprint` at all and Nav2/AMCL have no TF
  chain to the robot (silent failure: the goal is accepted and the robot never
  moves). Verify with
  `ros2 topic echo /tf | grep -c base_footprint` or `tf2_echo odom base_footprint
  --ros-args -p use_sim_time:=true`.
- MoveIt arm execution: `trajectory_execution.allowed_start_tolerance` is raised
  to 0.1 rad in `mir_isaac.launch.py` because Isaac's PD-driven joints jitter at
  the ~0.01 rad level (the default rejects execution with "start point deviates").
- **Arm drive gains** (`mir_isaac_sim.py`, tunable via `--arm-stiffness` /
  `--arm-damping` / `--arm-max-force`): stiffness 10000, damping 1000,
  max-force 330 N·m. The force cap is deliberately near the UR5 joint limit so
  the arm tracks MoveIt trajectories but **cannot generate enough reaction
  torque to tip the 67 kg MiR base** (verified upright; the base flips only at
  absurd force ~8000 N·m). At startup the arm is **teleported to its home pose**
  (`--no-arm-home` to disable) so the controllers don't yank it across a gap and
  flip the base.
- **Robotiq mimic joints under Isaac** (`robotiq_gripper.ros2_control.xacro`):
  for `sim_isaac` the gripper `ros2_control` block emits **only** the master
  `robotiq_85_left_knuckle_joint`; the 5 follower joints are deliberately NOT
  listed. `topic_based_ros2_control` 0.2.0 (the Humble debian) corrupts the heap
  in `TopicBasedSystem::read()` when a mimic joint is declared with no
  command/state interface — `ros2_control_node` aborts with
  `free(): invalid next size`, so no joint state is ever read. The followers
  still move: PhysX couples them in the USD and `robot_state_publisher` derives
  their TF from the master via the URDF `<mimic>` tags. (This block is otherwise
  the upstream Robotiq file, so the divergence is intentional.) The USD is
  unaffected — `make_isaac_urdf.py` strips all `<ros2_control>` blocks before
  import.
- **Jitter / idle-rotation tuning** (`mir_isaac_sim.py`): the imported
  articulation may need non-default physics knobs to sit still and not buzz.
  These are CLI flags. **Only the gripper ones are on by default**
  (`--gripper-armature 0.05`, `--solver-position-iterations 32`, self-collisions
  off) — the chassis and arm knobs below all default to **0.0**, i.e. off,
  because the damping values are a cure that can be worse than the disease
  (`--base-linear-damping 2.0` drags the base so hard it barely drives under
  Nav2). Measured on the current build: with every chassis knob at 0.0 the idle
  base is **exactly** static (0.000 mm/s linear, 0.0°/h yaw over 58 s of sim
  time), so reach for these only if you actually see movement. Symptom → fix:
  - *Gripper buzzes.* The Robotiq fingers have ~1e-5 kg·m² inertia, so the stiff
    position drive + the hard PhysX mimic coupling (`physxMimicJoint:gearing` on
    an over-constrained 4-bar linkage) oscillate faster than the sim step can
    integrate. Fix = **armature** on the 6 gripper joints (`--gripper-armature`,
    default 0.05) which raises their effective inertia into the integrable range,
    plus a higher articulation **solver position-iteration count**
    (`--solver-position-iterations`, default 32; importer default ~4) to converge
    the coupling each step. Self-collisions are also disabled by default
    (`--keep-self-collisions` to re-enable) so the fingers don't graze each
    other — external-object collision (grasping) is unaffected.
  - *Arm (wrist) jitters.* Same low-inertia ringing on the small wrist joints →
    **armature** on the 6 UR joints (`--arm-armature`, **default 0.0 = off**;
    0.05 is the value to try).
  - *Base slowly rotates in place when idle.* Two causes, two knobs, **both
    default 0.0 = off**: the trailing casters (`caster_wheel_dx = -0.0382 m`)
    pump yaw into the chassis when their swivel is fully free → light **caster
    swivel damping** (`--caster-swivel-damping`, try 2.0; rolling wheels stay
    free); and the chassis itself micro-wobbles in yaw on the casters (visible
    as the long arm appearing to move) → **base_link rigid-body damping**
    (`--base-angular-damping`, try 5.0). Use these sparingly and re-test
    driving afterwards: they are drag on the real motion too, and
    `--base-linear-damping` in particular (try 2.0 only if you must) is strong
    enough to stop Nav2 driving the base at all.
