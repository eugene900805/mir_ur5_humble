# MiR100 + UR5 + D435i + Robotiq85

![MiR100 + UR5 + Robotiq85 in NVIDIA Isaac Sim](Isaac_Sim_Mir_Ur5.png)

這個 workspace 整合 MiR100 移動底盤、UR5 手臂、Robotiq 85 夾爪與
RealSense D435i，支援 Isaac Sim、Gazebo Classic 與實體機器人。Isaac 與
Gazebo 共用同一套 ROS 2 Control、Nav2 和 MoveIt2 設定。

## 先選擇要做什麼

| 目的 | 從哪裡開始 |
|---|---|
| 在 Isaac GUI 裡跑迷宮、用 RViz 下導航目標 | 直接照下一節的 4 個終端機啟動 |
| 用 Gazebo 建圖或導航 | [Gazebo demo](#gazebo-demo-mapping) |
| 操作 MiR100 + UR5 實機 | [Real robot demo](#real-robot-demo-mir100--ur5-hardware) |
| 修改 USD、感測器或物理參數 | [`isaac_sim/README_isaac.md`](isaac_sim/README_isaac.md) |
| 查 Isaac / Nav2 已知問題與實測結果 | [`isaac_debug.md`](isaac_debug.md) |

## 快速啟動：Isaac Sim 迷宮導航（GUI）

這是目前驗證過、最接近 Gazebo 行為的啟動方式。Isaac、ROS Control、AMCL
和 Nav2 分開啟動，發生問題時也最容易看出是哪一層。

### 0. 第一次使用或修改程式後先建置

```bash
cd /mnt/data/mir_isaac
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

以下指令使用 `ROS_DOMAIN_ID=94` 隔離同一網路上的其他 ROS 節點。若要改
數字，**Isaac 和所有 ROS 終端機必須使用同一個 ID**。

### 1. Terminal 1：啟動 Isaac GUI 與迷宮

```bash
env ROS_DOMAIN_ID=94 \
  /home/shareduser/anaconda3/envs/env_isaaclab_opt/bin/python \
  /mnt/data/mir_isaac/src/mir_ur5_humble/isaac_sim/mir_isaac_sim.py \
  --lasers \
  --world /mnt/data/mir_isaac/src/mir_ur5_humble/isaac_sim/usd/maze.usd \
  --top-down
```

等 Isaac 載入完成並開始模擬後再開其餘終端機。Isaac 是 `/clock` 來源；
執行途中若重啟 Isaac，下面三個 ROS launch 也要一起重啟。

### 2. Terminal 2：啟動 ROS Control、RViz 與雙雷射合併

```bash
cd /mnt/data/mir_isaac
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=94

ros2 launch mir_description mir_isaac.launch.py launch_moveit:=false
```

這個 launch 已包含 robot state publisher、ROS Control、RViz，以及
`/f_scan + /b_scan -> /scan`；不要再另外啟動 scan merger。若也要操作
UR5 手臂，移除 `launch_moveit:=false` 即可。

### 3. Terminal 3：使用現有迷宮地圖定位

```bash
cd /mnt/data/mir_isaac
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=94

ros2 launch mir_navigation amcl.py use_sim_time:=true \
  map:=/mnt/data/mir_isaac/src/mir_ur5_humble/mir_robot/mir_navigation/maps/maze.yaml
```

### 4. Terminal 4：啟動 Nav2

```bash
cd /mnt/data/mir_isaac
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=94

ros2 launch mir_navigation navigation.py use_sim_time:=true \
  cmd_vel_w_prefix:=/diff_cont/cmd_vel_unstamped
```

### 5. 在 RViz 操作

1. 用 **2D Pose Estimate** 在地圖上設定車子的初始位置與朝向。
2. 等雷射點雲和地圖對齊後，用 **Nav2 Goal** 拖出目標箭頭。
3. 箭頭方向也是最終朝向。車子到達目標位置後，可能會原地轉到該方向；
   Nav2 顯示 `Goal succeeded` 才代表整個導航完成。

快速檢查：

```bash
ros2 topic hz /scan                         # 約 12 Hz
ros2 control list_controllers               # controllers 應為 active
ros2 lifecycle get /amcl                    # active
ros2 lifecycle get /bt_navigator            # active
```

若目標已接受但車子不動，先檢查 `/scan` 和 AMCL，再確認 Isaac 沒有被重啟。
迷宮窄口、原地轉向與壓力測試的實測紀錄在
[`isaac_debug.md`](isaac_debug.md)。

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

# Gazebo demo

每個終端機先執行：

```bash
cd /mnt/data/mir_isaac
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=94
```

### 使用現有地圖導航

依序開三個終端機：

```bash
# Terminal 1：Gazebo 迷宮 + RViz
ros2 launch mir_gazebo mobile_manipulator.launch.py world:=maze \
  rviz_config_file:=$(ros2 pkg prefix mir_navigation)/share/mir_navigation/rviz/mir_nav.rviz

# Terminal 2：AMCL 定位
ros2 launch mir_navigation amcl.py use_sim_time:=true \
  map:=$(ros2 pkg prefix mir_navigation)/share/mir_navigation/maps/maze.yaml

# Terminal 3：Nav2
ros2 launch mir_navigation navigation.py use_sim_time:=true \
  cmd_vel_w_prefix:=/diff_cont/cmd_vel_unstamped
```

在 RViz 先按 **2D Pose Estimate**，雷射與地圖對齊後再按 **Nav2 Goal**。

### 使用 SLAM 建圖

Terminal 1 仍啟動 Gazebo；Terminal 2 改成：

```bash
ros2 launch mir_navigation mapping.py use_sim_time:=true \
  slam_params_file:=$(ros2 pkg prefix mir_navigation)/share/mir_navigation/config/mir_mapping_async_sim.yaml
```

需要一邊建圖一邊自動導航時，再啟動上面的 Terminal 3。

### MoveIt2（選用）

```bash
ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur5 prefix:=ur_ launch_rviz:=true \
  use_fake_hardware:=true use_sim_time:=true
```

# Isaac Sim options

完整的 GUI 迷宮導航流程已放在 README 最上方。以下只列常用變化；感測器、
USD 轉換和物理參數的完整說明請看
[`isaac_sim/README_isaac.md`](isaac_sim/README_isaac.md)。

### 單一指令啟動 Isaac + ROS Control + RViz

```bash
ros2 launch mir_description mir_isaac.launch.py \
  launch_isaac:=true world:=maze top_down:=true launch_moveit:=false
```

這種方式較省指令；需要分別觀察 Isaac 與 ROS 輸出時，仍建議使用上方的
4-terminal 流程。

### 改用 SLAM 建圖

保留快速啟動的 Terminal 1、2、4，並用下面指令取代 AMCL：

```bash
ros2 launch mir_navigation mapping.py use_sim_time:=true \
  slam_params_file:=$(ros2 pkg prefix mir_navigation)/share/mir_navigation/config/mir_mapping_async_sim.yaml
```

### 常用啟動選項

```bash
# 不開 Isaac GUI（PhysX 雷射仍可用）
ros2 launch mir_description mir_isaac.launch.py \
  launch_isaac:=true world:=maze headless:=true launch_rviz:=false launch_moveit:=false

# 只開 ROS Control，不開 MoveIt、RViz 或 scan merger
ros2 launch mir_description mir_isaac.launch.py \
  launch_moveit:=false launch_rviz:=false launch_scan_merger:=false
```

Isaac 會發布 `/f_scan`、`/b_scan`、`/odom` 與
`odom -> base_footprint`；ROS launch 預設合併兩個雷射成 `/scan`。
`diffdrive_controller_isaac.yaml` 只覆寫 `enable_odom_tf: false`，避免 ROS
Control 和 Isaac 重複發布同一段 TF。其餘控制器參數與 Gazebo 共用
`diffdrive_controller.yaml`，避免兩套模擬器的設定逐漸不一致。

# Real robot demo (MiR100 + UR5 hardware)

> Full debugging notes (WiFi keepalive, controller choice, shared localization,
> all symptoms → root causes → fixes, in Chinese) are in
> **[`claude_debug.md`](claude_debug.md)**.

Device addresses depend on which WiFi the PC is on (UR5 control box is
`192.168.0.101` on both):

| WiFi network | MiR address | Note |
|---|---|---|
| `Aislab_mir3` (MiR's own hotspot) | `192.168.12.20` | the robot's DNS resolves `mir.com` to itself |
| `TP-Link_550F_5G` (lab router) | `192.168.0.104` | MiR joins as a WiFi client |

Don't guess — auto-detect (tries `mir.com`, known addresses, then scans the
subnet for rosbridge port 9090):

```
ros2 launch mir_navigation mir_nav_launch.py mir_hostname:=$(./src/mir_robot/tools/find_mir.sh)
```

The PC's own IP never needs to be typed anywhere: the UR driver auto-detects
its reverse-connection IP (headless mode) and the MiR bridge connects outward.
After switching WiFi: restart the UR driver (its reverse IP was baked in at
startup), restart the nav stack with the new `mir_hostname`, and disable WiFi
power-save on the **new** connection profile
(`nmcli connection modify "<SSID>" 802-11-wireless.powersave 2`, then
down/up) — power-save causes the periodic outages that break External Control.

Run each block in its own terminal; every terminal needs
`cd <workspace> && source install/setup.bash` first (system python, **not** conda).

```
### Terminal 1 — UR5 driver (headless External Control, no teach pendant needed)
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5 robot_ip:=192.168.0.101 tf_prefix:=ur_ \
    launch_rviz:=false launch_rsp:=false headless_mode:=true \
    initial_joint_controller:=passthrough_trajectory_controller \
    activate_joint_controller:=true controller_spawner_timeout:=60

### Terminal 2 — MoveIt (drag the goal ball in RViz -> Plan & Execute)
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5 prefix:=ur_ launch_rviz:=true

### Terminal 3 — MiR navigation (bridge + Nav2 + RViz, shares the robot's own
###              localization/map with the MiR web UI -> no 2D Pose Estimate needed)
ros2 launch mir_navigation mir_nav_launch.py mir_hostname:=192.168.0.104

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
  if your network is solid you can ignore this entirely. On a flaky link (e.g.
  our 2.4 GHz `Aislab_mir3` hotspot drops every ~10-15 min even at the 5 s
  watchdog maximum) keep `tools/ur_program_watchdog.sh` running in a 5th
  terminal; it auto-resends within seconds.
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

# Running on another machine (paths to change)

This tree was set up with the workspace at `/mnt/data/mir_isaac` and Isaac Sim at
`/mnt/data/IsaacSim`. On a different machine, adjust these machine-specific paths:

| What | Hardcoded value | Where | Fix |
|---|---|---|---|
| Isaac Sim install | `/mnt/data/IsaacSim/_build/linux-x86_64/release/python.sh` | Terminal 1 / docs | point at *your* Isaac build's `python.sh` |
| Workspace root | `/mnt/data/mir_isaac` | run commands | use your own ws (the install section above uses `~/ros2_ws`) |
| `isaac_python` arg | `/home/shareduser/anaconda3/envs/env_isaaclab_opt/bin/python` | `mir_description/launch/mir_isaac.launch.py` (~line 98) | a python that has Isaac / `pxr`; override `isaac_python:=...` (only used when `launch_isaac:=true`) |
| `isaac_dir` arg | `/mnt/data/mir_isaac/src/mir_ur5_humble/isaac_sim` | `mir_isaac.launch.py` | override `isaac_dir:=...` (only when `launch_isaac:=true`) |

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
