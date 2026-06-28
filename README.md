# mir100+ur5+d435i+robotiq85

![MiR100 + UR5 + Robotiq85 in NVIDIA Isaac Sim](Isaac_Sim_Mir_Ur5.png)

> **Isaac Sim back-end available.** This robot can be simulated in NVIDIA
> Isaac Sim 5.0 (instead of Gazebo) while keeping the same ros2_control + MoveIt2
> stack. See **[`isaac_sim/README_isaac.md`](isaac_sim/README_isaac.md)**.
> Quick start:
> ```
> # terminal 1 (Isaac Sim):
> /mnt/data/IsaacSim/_build/linux-x86_64/release/python.sh isaac_sim/mir_isaac_sim.py
> # terminal 2 (ROS control + MoveIt):
> ros2 launch mir_description mir_isaac.launch.py
> ```

# Installation

## Preliminaries
## ROS2
If you haven't already installed [ROS2](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) on your PC, you need to add the ROS2 apt repository.

Also install ros2-control, ros2-controllers, gazebo-ros-pkgs(usually installed), gazebo-ros2-control

```
sudo apt-get install ros-humble-ros2-control
sudo apt-get install ros-humble-ros2-controllers
sudo apt-get install ros-humble-gazebo-ros-pkgs
sudo apt-get install ros-humble-gazebo-ros2-control
sudo apt install ros-humble-gripper-controllers
sudo apt install ros-humble-rmw-cyclonedds-cpp
sudo apt install ros-humble-sensor-msgs
```
## Source install
```
# create a ros2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/

# clone mir_robot into the ros2 workspace
git clone https://github.com/eugene900805/mir_ur5_humble.git src/mir_robot

# use vcs to fetch linked repos
# $ sudo apt install python3-vcstool
vcs import < src/mir_robot/mir_robot/ros2.repos src --recursive

# use rosdep to install all dependencies (including ROS itself)
sudo apt update
sudo apt install -y python3-rosdep
rosdep update --rosdistro=humble
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble

# build all packages in the workspace
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
echo ". /usr/share/gazebo/setup.sh" >> ~/.bashrc
source ~/.bashrc
cd ~/ros2_ws
colcon build
```

# Gazebo demo (mapping)
```
### gazebo: 
ros2 launch mir_gazebo mobile_manipulator.launch.py world:=maze

### mapping (slam_toolbox)
ros2 launch mir_navigation mapping.py use_sim_time:=true slam_params_file:=$(ros2 pkg prefix mir_navigation)/share/mir_navigation/config/mir_mapping_async_sim.yaml

### navigation (optional)
ros2 launch mir_navigation navigation.py use_sim_time:=true cmd_vel_w_prefix:=/diff_cont/cmd_vel_unstamped
```

# Gazebo demo (Navigation with existing map)
```
### gazebo:
ros2 launch mir_gazebo mobile_manipulator.launch.py world:=maze \
    rviz_config_file:=$(ros2 pkg prefix mir_navigation)/share/mir_navigation/rviz/mir_nav.rviz


### localization (existing map)
ros2 launch mir_navigation amcl.py use_sim_time:=true \
    map:=$(ros2 pkg prefix mir_navigation)/share/mir_navigation/maps/maze.yaml

### navigation
ros2 launch mir_navigation navigation.py use_sim_time:=true

### set initial pose
ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: 'map'
pose:
  pose:
    position:
      x: 0.0
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
" --once

### MoveIt2:
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5 launch_rviz:=true \
    prefix:=ur_ use_fake_hardware:=true use_sim_time:=true
```

# Isaac Sim demo (Navigation with existing map)

> Isaac Sim 5.0 replaces Gazebo as the physics/sensor back-end; the ROS 2
> control + Nav2 + MoveIt stack is identical. Full details in
> [`isaac_sim/README_isaac.md`](isaac_sim/README_isaac.md).

Run each block in its own terminal; every ROS terminal needs
`cd <workspace> && source install/setup.bash` first. **Start Isaac FIRST** — it
is the `/clock` source, so restarting it resets sim time and crashes the live
ROS stack / RViz.

```
### Terminal 1 — Isaac Sim (two SICK S300 lasers + ground-truth odom), then press Play
python src/mir_robot/isaac_sim/mir_isaac_sim.py --lasers --publish-odom \
    --world src/mir_robot/isaac_sim/usd/maze.usd --top-down
#   must run with Isaac's python; e.g. from the Isaac build dir:
#   <IsaacSim>/_build/linux-x86_64/release/python.sh <abs path>/mir_isaac_sim.py --lasers ...
#   add --headless to run without a GUI (PhysX lidar works headless)

### Terminal 2 — robot_state_publisher + ros2_control + MoveIt + RViz + scan merger (all-in-one)
ros2 launch mir_description mir_isaac.launch.py
#   one launch gives the whole TF tree, merges /f_scan + /b_scan -> /scan, opens RViz.
#   drop pieces with: launch_moveit:=false  launch_rviz:=false  launch_scan_merger:=false

### Terminal 3 — localization (existing map)
ros2 launch mir_navigation amcl.py use_sim_time:=true \
    map:=$(ros2 pkg prefix mir_navigation)/share/mir_navigation/maps/maze.yaml
    
### Terminal 4 — navigation
ros2 launch mir_navigation navigation.py use_sim_time:=true \
    cmd_vel_w_prefix:=/diff_cont/cmd_vel_unstamped
```

For SLAM instead of a saved map, replace Terminal 3 with:
```
ros2 launch mir_navigation mapping.py use_sim_time:=true \
    slam_params_file:=$(ros2 pkg prefix mir_navigation)/share/mir_navigation/config/mir_mapping_async_sim.yaml
```

In RViz: **2D Pose Estimate** to seed AMCL → **Nav2 Goal** to send a target.
Health checks: `ros2 topic hz /scan` (~12 Hz), and
`ros2 run tf2_ros tf2_echo odom base_footprint` (Isaac's `--publish-odom`
provides this). Unlike Gazebo, Isaac publishes `/f_scan` + `/b_scan` separately,
so the scan merger (bundled in Terminal 2) is required.

> **Isaac vs Gazebo gotcha — `enable_odom_tf`:** they use *different* controller
> param files. Gazebo's `diffdrive_controller.yaml` has `enable_odom_tf: true`
> (it is the only odom→base_footprint source). Isaac uses
> `diffdrive_controller_isaac.yaml` with `enable_odom_tf: false`, because the
> Isaac bridge already broadcasts that TF via `--publish-odom`. `mir_isaac.launch.py`
> picks the Isaac file automatically — do not point Gazebo at it or nav breaks.

# Running on another machine (paths to change)

This tree was set up with the workspace at `/mnt/data/mir_isaac` and Isaac Sim at
`/mnt/data/IsaacSim`. On a different machine, adjust these machine-specific paths:

| What | Hardcoded value | Where | Fix |
|---|---|---|---|
| Isaac Sim install | `/mnt/data/IsaacSim/_build/linux-x86_64/release/python.sh` | Terminal 1 / docs | point at *your* Isaac build's `python.sh` |
| Workspace root | `/mnt/data/mir_isaac` | run commands | use your own ws (the install section above uses `~/ros2_ws`) |
| `isaac_python` arg | `/home/shareduser/anaconda3/envs/env_isaaclab_opt/bin/python` | `mir_description/launch/mir_isaac.launch.py` (~line 98) | a python that has Isaac / `pxr`; override `isaac_python:=...` (only used when `launch_isaac:=true`) |
| `isaac_script_dir` arg | `/mnt/data/mir_isaac/src/mir_robot/isaac_sim` | `mir_isaac.launch.py` (~line 102) | override `isaac_script_dir:=...` (only when `launch_isaac:=true`) |

Other portability notes:
- `mir_isaac_sim.py` is meant to be run directly; its `--usd` default is computed
  relative to the script, so it is path-independent. The `/mnt/data/...` lines in
  its header are example comments only.
- Run the script with **Isaac's own python** (`python.sh`) or an env that can
  import `omni.*` / `pxr`; plain system python cannot load the Isaac modules.
- Isaac's shipped lidar-config dir may contain a symlink to a non-existent
  `/home/shareduser/IsaacSim`; this only affects *custom RTX* lidar profiles —
  the default PhysX `--lasers` path is unaffected.
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



