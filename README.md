# MiR100 + UR5 + D435i + Robotiq85

![MiR100 + UR5 + Robotiq85 in NVIDIA Isaac Sim](Isaac_Sim_Mir_Ur5.png)

This workspace integrates the MiR100 mobile base, the UR5 arm, the Robotiq 85
gripper and a RealSense D435i, and supports Isaac Sim, Gazebo Classic and the
real robot. Isaac and Gazebo share the same ROS 2 Control, Nav2 and MoveIt2
configuration.

# Installation

Follow this README in order: install and build the workspace first, start Isaac
Sim next, and use the Gazebo workflow afterward if needed.

## Assumed directory and Python environment

All commands assume the repository is installed under `~/ros2_ws/src` and
Isaac Sim / Isaac Lab is available in the Anaconda environment
`env_isaaclab`:

```bash
export MIR_WS=~/ros2_ws
export MIR_REPO=~/ros2_ws/src/mir_ur5_humble
export ISAAC_PYTHON=~/anaconda3/envs/env_isaaclab/bin/python
```

Add these exports to `~/.bashrc` if you want them to be available in every new
terminal. The `env_isaaclab` environment is used only for the Isaac Sim Python
process. Build and run the ROS 2, Gazebo and real-robot processes with the
system ROS environment, not from Conda.

## Prerequisites

### ROS 2

If you haven't already installed
[ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
on your PC, install it and add the ROS 2 apt repository first.

Also install ros2-control, ros2-controllers, gazebo-ros-pkgs(usually installed), gazebo-ros2-control

```bash
sudo apt-get install ros-humble-ros2-control
sudo apt-get install ros-humble-ros2-controllers
sudo apt-get install ros-humble-gazebo-ros-pkgs
sudo apt-get install ros-humble-gazebo-ros2-control
sudo apt install ros-humble-gripper-controllers
sudo apt install ros-humble-rmw-cyclonedds-cpp
sudo apt install ros-humble-sensor-msgs
```

For the Isaac Sim back-end and the Nav2 stack also install:

```bash
sudo apt install ros-humble-topic-based-ros2-control
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox
sudo apt install ros-humble-pcl-ros ros-humble-pcl-conversions
sudo apt install ros-humble-moveit
```

Before continuing, make sure the prepared Anaconda environment exists at
`~/anaconda3/envs/env_isaaclab` and contains the Isaac Sim / Isaac Lab Python
packages required by this project.

## Install the source under `ros2_ws/src`

```bash
# Create the ROS 2 workspace.
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone this repository as ~/ros2_ws/src/mir_ur5_humble.
git clone https://github.com/eugene900805/mir_ur5_humble.git mir_ur5_humble

# Fetch the repositories listed by this project.
sudo apt install python3-vcstool
cd ~/ros2_ws
vcs import src < src/mir_ur5_humble/mir_robot/ros2.repos --recursive

# Install package dependencies.
sudo apt update
sudo apt install -y python3-rosdep
rosdep update --rosdistro=humble
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble

# Build with the system Python used by ROS 2. Do not activate Conda here.
source /opt/ros/humble/setup.bash
PATH=/usr/bin:/bin:/opt/ros/humble/bin:$PATH PYTHONPATH= \
  colcon build --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source ~/ros2_ws/install/setup.bash
```

Re-run the build after changing source files. Add the following setup commands
to `~/.bashrc` if desired:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
. /usr/share/gazebo/setup.sh
```

# Isaac Sim startup

The four-terminal workflow keeps Isaac Sim, ROS Control, AMCL and Nav2
separate, making each layer easier to inspect. Build the workspace as described
above before starting these terminals.

## Terminal 1: start the Isaac Sim GUI and maze

Use the Python executable from the Anaconda environment `env_isaaclab`:

```bash
source ~/anaconda3/etc/profile.d/conda.sh
conda activate env_isaaclab
cd ~/ros2_ws/src/mir_ur5_humble

python isaac_sim/mir_isaac_sim.py \
  --lasers \
  --world isaac_sim/usd/maze.usd \
  --top-down
```

Wait until Isaac Sim finishes loading and starts stepping the simulation before
opening the other terminals. Isaac Sim is the `/clock` source. If you restart
it during a run, also restart the following three ROS launches.

## Terminal 2: start ROS Control, MoveIt2, RViz and the dual-laser merger

Open a normal ROS terminal without activating Conda:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch mir_description mir_isaac.launch.py launch_moveit:=true
```

This launch includes the robot state publisher, ROS Control, MoveIt2, RViz and
`/f_scan + /b_scan -> /scan`; do not start a second scan merger. RViz contains
both the navigation tools and the MoveIt **MotionPlanning** panel.

## Terminal 3: localize against the maze map

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch mir_navigation amcl.py use_sim_time:=true \
  map:=$(ros2 pkg prefix mir_navigation)/share/mir_navigation/maps/maze.yaml
```

## Terminal 4: start Nav2

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch mir_navigation navigation.py use_sim_time:=true \
  cmd_vel_w_prefix:=/diff_cont/cmd_vel_unstamped
```

## Drive the robot from RViz

1. Use **2D Pose Estimate** to set the base's initial position and heading.
2. Wait for the laser scan to line up with the map.
3. Drag a goal arrow with **Nav2 Goal**. The arrow direction is the desired
   final heading; navigation is complete when Nav2 reports `Goal succeeded`.
4. Use **Plan & Execute** in the **MotionPlanning** panel to move the UR5 arm.

Quick checks:

```bash
ros2 topic hz /scan                         # about 12 Hz
ros2 control list_controllers               # controllers should be active
ros2 node list | grep move_group            # /move_group
ros2 lifecycle get /amcl                    # active
ros2 lifecycle get /bt_navigator            # active
```

If a goal is accepted but the base does not move, check `/scan` and AMCL first,
then confirm Isaac Sim has not been restarted. Measured results for narrow maze
gaps, turning in place and stress tests are in
[`isaac_debug.md`](isaac_debug.md).

## Isaac Sim options

For the full sensor, USD conversion and physics documentation, see
[`isaac_sim/README_isaac.md`](isaac_sim/README_isaac.md).

### Start Isaac Sim, ROS Control, MoveIt2 and RViz with one command

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ISAAC_PYTHON=~/anaconda3/envs/env_isaaclab/bin/python

ros2 launch mir_description mir_isaac.launch.py \
  launch_isaac:=true world:=maze top_down:=true launch_moveit:=true
```

This is shorter, but the four-terminal flow above keeps the Isaac Sim and ROS
logs separate. `launch_isaac:=true` uses `ISAAC_PYTHON` to start
`mir_isaac_sim.py`.

### Switch to SLAM mapping

Keep Isaac Sim, ROS Control and Nav2 running, but replace the AMCL command in
Terminal 3 with:

```bash
ros2 launch mir_navigation mapping.py use_sim_time:=true \
  slam_params_file:=$(ros2 pkg prefix mir_navigation)/share/mir_navigation/config/mir_mapping_async_sim.yaml
```

### Common launch options

```bash
# No Isaac GUI; the PhysX lasers still work.
ros2 launch mir_description mir_isaac.launch.py \
  launch_isaac:=true world:=maze headless:=true launch_rviz:=false launch_moveit:=false

# ROS Control only, without MoveIt, RViz or the scan merger.
ros2 launch mir_description mir_isaac.launch.py \
  launch_moveit:=false launch_rviz:=false launch_scan_merger:=false
```

Isaac Sim publishes `/f_scan`, `/b_scan`, `/odom` and
`odom -> base_footprint`; the ROS launch merges the two lasers into `/scan` by
default. `diffdrive_controller_isaac.yaml` only overrides
`enable_odom_tf: false`, so ROS Control and Isaac Sim do not publish the same TF
twice. All other controller parameters are shared with Gazebo through
`diffdrive_controller.yaml`.

# Gazebo startup

Gazebo and all of its ROS nodes use the system ROS environment; do not activate
`env_isaaclab`. Run this setup in every Gazebo terminal:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
. /usr/share/gazebo/setup.sh
```

## Navigate with an existing map

Open three terminals in order:

```bash
# Terminal 1: Gazebo maze + ROS Control + MoveIt2 + RViz
ros2 launch mir_gazebo mobile_manipulator.launch.py \
  world:=maze launch_moveit:=true

# Terminal 2: AMCL localization
ros2 launch mir_navigation amcl.py use_sim_time:=true \
  map:=$(ros2 pkg prefix mir_navigation)/share/mir_navigation/maps/maze.yaml

# Terminal 3: Nav2
ros2 launch mir_navigation navigation.py use_sim_time:=true \
  cmd_vel_w_prefix:=/diff_cont/cmd_vel_unstamped
```

In RViz press **2D Pose Estimate** first, and only press **Nav2 Goal** once the
laser scan lines up with the map. Use **Plan & Execute** in the
**MotionPlanning** panel to move the UR5 arm.

## Map with SLAM

Terminal 1 still starts Gazebo; replace Terminal 2 with:

```bash
ros2 launch mir_navigation mapping.py use_sim_time:=true \
  slam_params_file:=$(ros2 pkg prefix mir_navigation)/share/mir_navigation/config/mir_mapping_async_sim.yaml
```

If you want to navigate automatically while mapping, also start Terminal 3 from
above.

MoveIt2 is already started by Gazebo's Terminal 1 command. Pass
`launch_moveit:=false` only when you intentionally want to disable it.

# Real robot demo (MiR100 + UR5 hardware)

> Full debugging notes (WiFi keepalive, controller choice, shared localization,
> all symptoms → root causes → fixes, in Chinese) are in
> **[`claude_debug.md`](claude_debug.md)**.

The MiR's address depends on which WiFi the PC is on, so don't hardcode it —
auto-detect it (tries `mir.com`, then known addresses, then scans the subnet
for rosbridge port 9090):

```
export MIR_HOST=$($MIR_REPO/tools/find_mir.sh)
ros2 launch mir_navigation mir_nav_launch.py mir_hostname:=$MIR_HOST
```

The two cases the script covers, with our lab's values as an example — yours
will differ, and the UR5 control box has its own fixed address on the same
subnet (`192.168.0.101` here, set it as `UR_HOST`):

| WiFi network | MiR address | Note |
|---|---|---|
| MiR's own hotspot | `192.168.12.20` | the robot's DNS resolves `mir.com` to itself |
| a lab router the MiR joins as a client | DHCP, e.g. `192.168.0.104` | found by the subnet scan |

The PC's own IP never needs to be typed anywhere: the UR driver auto-detects
its reverse-connection IP (headless mode) and the MiR bridge connects outward.
After switching WiFi: restart the UR driver (its reverse IP was baked in at
startup), restart the nav stack with the new `mir_hostname`, and disable WiFi
power-save on the **new** connection profile
(`nmcli connection modify "<SSID>" 802-11-wireless.powersave 2`, then
down/up) — power-save causes the periodic outages that break External Control.

Run each block in its own terminal; every terminal needs
`cd $MIR_WS && source install/setup.bash` first (system python, **not** conda),
plus the `UR_HOST` / `MIR_HOST` from above.

```
### Terminal 1 — UR5 driver (headless External Control, no teach pendant needed)
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5 robot_ip:=$UR_HOST tf_prefix:=ur_ \
    launch_rviz:=false launch_rsp:=false headless_mode:=true \
    initial_joint_controller:=passthrough_trajectory_controller \
    activate_joint_controller:=true controller_spawner_timeout:=60

### Terminal 2 — MoveIt (drag the goal ball in RViz -> Plan & Execute)
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5 prefix:=ur_ launch_rviz:=true

### Terminal 3 — MiR navigation (bridge + Nav2 + RViz, shares the robot's own
###              localization/map with the MiR web UI -> no 2D Pose Estimate needed)
ros2 launch mir_navigation mir_nav_launch.py mir_hostname:=$MIR_HOST

### Terminal 4 — laser scan merger (required; QoS flags matter on hardware)
ros2 launch mir_description mir_isaac_scan_merger.launch.py use_sim_time:=false best_effort:=true
```

Then send **Nav2 Goal** in the navigation RViz to drive the base, and
**Plan & Execute** in the MoveIt RViz to move the arm.

Key points (details in `claude_debug.md`):

- `launch_rsp:=false` — the MiR combined model already publishes the arm TF;
  a second robot_state_publisher from the driver corrupts the TF tree.
- `headless_mode:=true` — the driver injects the External Control URScript
  itself; the pendant program (`test.urp`) is irrelevant and must NOT be
  played. If the script stops, resend with
  `ros2 service call /io_and_status_controller/resend_robot_program std_srvs/srv/Trigger`
  (takes ~6 s). Occasional drops are caused by an **unstable router/WiFi** —
  if your network is solid you can ignore this entirely. On a flaky link (our
  2.4 GHz MiR hotspot drops every ~10-15 min even at the 5 s watchdog maximum)
  keep `$MIR_REPO/tools/ur_program_watchdog.sh` running in a 5th terminal; it
  auto-resends within seconds.
- `initial_joint_controller` selects the trajectory controller:
  `passthrough_trajectory_controller` (whole trajectory interpolated on the
  robot — smooth over WiFi, default) vs `scaled_joint_trajectory_controller`
  (125 Hz streaming — stutters over WiFi). MoveIt follows whichever is active.
- WiFi keepalive: workspace-vendored `ur_client_library` raises
  `MAX_RT_RECEIVE_TIMEOUT_MS` 200 ms → 5000 ms and `keep_alive_count`
  (xacro arg, default 250 = 5 s) sets the watchdog; WiFi power-save is
  disabled on the AP connection. Without these the External Control
  connection drops within seconds.
- `use_mir_localization:=true` (default) — RViz always shows the same pose as
  the MiR web UI (`mir_localization_tf` converts `/robot_pose` into the
  map→odom TF); no local AMCL/map_server. Old behavior:
  `use_mir_localization:=false map:=maze.yaml`.
- Speed limits are launch args: real-robot defaults `max_vel_x:=0.3`,
  `max_vel_theta:=0.5` (simulation defaults: 1.2 m/s, 1.0 rad/s). Override:
  `max_vel_x:=0.5 max_vel_theta:=0.8`.
- Before relaunching, kill leftovers or duplicate action servers break Nav2
  (`unknown goal response`):
  `pkill -f mir_nav_launch.py; pkill -f mir_launch.py; pkill -f twist_stamper; pkill -f teleop_twist_keyboard; pkill xterm`

# Using a different installation path

This README assumes `~/ros2_ws/src/mir_ur5_humble` and the Anaconda environment
`env_isaaclab`, as described in
[Assumed directory and Python environment](#assumed-directory-and-python-environment).
If your installation is elsewhere, update `MIR_WS`, `MIR_REPO` and
`ISAAC_PYTHON`. No absolute path is baked into the launch files:

| Launch arg | Where its default comes from |
|---|---|
| `isaac_python` | `$ISAAC_PYTHON`; the launch aborts with a clear message if it is unset |
| `isaac_dir` | `$MIR_ISAAC_DIR`, else `isaac_sim/` is derived from the launch file's own location in this checkout |

Both are only read when `launch_isaac:=true`; the four-terminal workflow starts
Isaac Sim directly and never touches them. The `isaac_dir` derivation needs the
`--symlink-install` build used throughout this README (a plain copying build
breaks the link back to the source tree — then pass `isaac_dir:=` explicitly).

Other portability notes:

- `mir_isaac_sim.py` is meant to be run directly; its `--usd` default is computed
  relative to the script, so it is path-independent. Only `--world` takes a full
  path.
- Run the script with Python from the `env_isaaclab` environment. Plain system
  Python cannot load the Isaac Sim modules.
- Isaac's shipped lidar-config dir may contain a symlink to an Isaac Sim
  location that does not exist on your machine; this only affects *custom RTX*
  lidar profiles — the default PhysX `--lasers` path is unaffected.
- After editing any `mir_description/config/*.yaml` or launch file, rebuild
  (`colcon build --packages-select mir_description`) so the `install/` copy used
  at runtime is refreshed (or edit the `install/` copy too).

# Notes

1. If you get an error with respect to Gazebo Classic: Cannot launch gzclient on a launch file - results in shared_ptr assertion error, 

    All you have to do is, source the gazebo classic by`. /usr/share/gazebo/setup.sh`
    and try again

2. If you encounter an error with respect to launching the ur_moveit launch:  Exception caught while processing action 'loadRobotModel': parameter 'robot_description_planning.joint_limits.panda_joint1.max_velocity' has invalid type: Wrong parameter type, parameter {robot_description_planning.joint_limits.panda_joint1.max_velocity} is of type {double}, setting it to {string} is not allowed,

    All you have to do is set `LC_NUMERIC=en_US.UTF-8` in your terminal and try again

3. If you get an error about gazebo already open elsewhere, run this -

    `killall gzserver `
    
## Acknowledgement

The 3d files for MiR 100 is from [DFKI](https://github.com/DFKI-NI/mir_robot).
The 3d model and plugins for UR5 are from [Universal_Robots_ROS2_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver).
The Realsense plugin is from [PAL Robotics](https://github.com/pal-robotics/realsense_gazebo_plugin/tree/foxy-devel) and description from [Intel](https://github.com/IntelRealSense/realsense-ros).
