# MiR100 + UR5 + Robotiq85 + D435i — Isaac Sim back-end (replaces Gazebo).
#
# This launch file brings up the *ROS side* of the Isaac Sim integration:
#   - robot_state_publisher (robot_description built with sim_isaac:=true)
#   - controller_manager (ros2_control_node) using the topic_based hardware
#     interface, which talks to Isaac Sim over /isaac_joint_commands and
#     /isaac_joint_states
#   - controller spawners: joint_broadcaster, diff_cont,
#     joint_trajectory_controller, gripper_position_controller
#   - MoveIt2 move_group + RViz (optional)
#
# Isaac Sim itself is started separately with:
#   <isaac>/python.sh isaac_sim/mir_isaac_sim.py
# (see isaac_sim/README_isaac.md). Isaac publishes /clock, so use_sim_time:=true.
#
# Example:
#   ros2 launch mir_description mir_isaac.launch.py
#   ros2 launch mir_description mir_isaac.launch.py launch_moveit:=false launch_rviz:=false

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def default_isaac_dir():
    """Locate <repo>/isaac_sim without hardcoding a machine-specific path.

    $MIR_ISAAC_DIR wins if set. Otherwise derive it from this file's real
    location: <repo>/mir_robot/mir_description/launch/mir_isaac.launch.py ->
    <repo>/isaac_sim. With `colcon build --symlink-install` the installed copy
    is a symlink into the source tree, so realpath() lands in the repo either
    way; with a plain (copying) build it does not, hence the isdir() check.
    Returns "" when it cannot be found, so `isaac_dir:=` must be passed.
    """
    env = os.environ.get("MIR_ISAAC_DIR")
    if env:
        return env
    launch_dir = os.path.dirname(os.path.realpath(__file__))
    repo = os.path.abspath(os.path.join(launch_dir, "..", "..", ".."))
    candidate = os.path.join(repo, "isaac_sim")
    return candidate if os.path.isdir(candidate) else ""


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except (EnvironmentError, yaml.YAMLError):
        return None


def generate_launch_description():
    ld = LaunchDescription()

    # ----------------------------------------------------------------- args
    declared_args = [
        DeclareLaunchArgument("ur_type", default_value="ur5",
                              description="UR model: ur3/ur3e/ur5/ur5e/ur10/..."),
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="Use Isaac Sim /clock."),
        DeclareLaunchArgument("launch_moveit", default_value="true"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument("isaac_joint_commands",
                              default_value="/isaac_joint_commands"),
        DeclareLaunchArgument("isaac_joint_states",
                              default_value="/isaac_joint_states"),
        # ---- single-command Isaac: also start Isaac Sim from this launch ----
        DeclareLaunchArgument("launch_isaac", default_value="false",
                              description="Also start Isaac Sim (mir_isaac_sim.py) "
                                          "from this launch, so one command brings "
                                          "up sim + control like the Gazebo one."),
        DeclareLaunchArgument("world", default_value="",
                              description="Environment name under isaac_sim/usd "
                                          "(e.g. 'maze' -> usd/maze.usd). Empty = "
                                          "ground plane only."),
        DeclareLaunchArgument("headless", default_value="false",
                              description="Run Isaac Sim without a GUI."),
        DeclareLaunchArgument("top_down", default_value="false",
                              description="Isaac Sim top-down camera view."),
        DeclareLaunchArgument("lasers", default_value="true",
                              description="Enable the SICK lidars in Isaac Sim."),
        DeclareLaunchArgument("launch_scan_merger", default_value="true",
                              description="Merge Isaac's /f_scan + /b_scan into a "
                                          "single /scan for Nav2 (Gazebo emits "
                                          "/scan directly; Isaac needs this)."),
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("mir_navigation"), "rviz", "mir_nav.rviz"]),
            description="RViz config (defaults to the mir_navigation nav view, "
                        "matching the Gazebo workflow)."),
        DeclareLaunchArgument(
            "isaac_python",
            default_value=os.environ.get("ISAAC_PYTHON", ""),
            description="Python interpreter that can import isaacsim/omni "
                        "(Isaac Sim's python.sh, or an Isaac Sim conda env). "
                        "Defaults to $ISAAC_PYTHON; only used with "
                        "launch_isaac:=true."),
        DeclareLaunchArgument(
            "isaac_dir",
            default_value=default_isaac_dir(),
            description="Directory containing mir_isaac_sim.py and usd/. "
                        "Defaults to $MIR_ISAAC_DIR, else the isaac_sim/ dir of "
                        "this checkout; only used with launch_isaac:=true."),
    ]
    for a in declared_args:
        ld.add_action(a)

    # When launch_isaac:=true, start Isaac Sim as a child process. Built with an
    # OpaqueFunction so optional flags (--world/--headless/--top-down/--lasers)
    # are only appended when requested (you can't conditionally add a single
    # argv entry with plain substitutions).
    def _start_isaac(context):
        if context.launch_configurations.get("launch_isaac", "false") != "true":
            return []
        py = context.launch_configurations["isaac_python"]
        idir = context.launch_configurations["isaac_dir"]
        if not py:
            raise RuntimeError(
                "launch_isaac:=true needs an Isaac Sim python. Set $ISAAC_PYTHON "
                "(e.g. <isaac-sim>/python.sh) or pass isaac_python:=<path>.")
        if not idir:
            raise RuntimeError(
                "launch_isaac:=true could not locate the isaac_sim/ directory. "
                "Set $MIR_ISAAC_DIR or pass isaac_dir:=<repo>/isaac_sim.")
        cmd = [py, os.path.join(idir, "mir_isaac_sim.py"), "--publish-odom"]
        world = context.launch_configurations.get("world", "")
        if world:
            cmd += ["--world", os.path.join(idir, "usd", world + ".usd")]
        if context.launch_configurations.get("lasers", "true") == "true":
            cmd.append("--lasers")
        if context.launch_configurations.get("headless", "false") == "true":
            cmd.append("--headless")
        if context.launch_configurations.get("top_down", "false") == "true":
            cmd.append("--top-down")
        return [ExecuteProcess(cmd=cmd, output="screen", name="isaac_sim")]

    ld.add_action(OpaqueFunction(function=_start_isaac))

    ur_type = LaunchConfiguration("ur_type")
    use_sim_time = LaunchConfiguration("use_sim_time")
    isaac_joint_commands = LaunchConfiguration("isaac_joint_commands")
    isaac_joint_states = LaunchConfiguration("isaac_joint_states")

    mir_description_share = FindPackageShare("mir_description")
    moveit_config_share = FindPackageShare("ur_moveit_config")
    ur_description_share = FindPackageShare("ur_description")

    # ------------------------------------------------------ robot_description
    # Full robot (MiR base + UR + Robotiq + D435i) with the Isaac hardware
    # interface selected. This exact description is shared by RSP, the
    # controller_manager and move_group so everything agrees on the model.
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([mir_description_share, "urdf", "mir.urdf.xacro"]), " ",
        "sim_isaac:=true", " ",
        "sim_gazebo:=false", " ",
        "ur_type:=", ur_type, " ",
        "isaac_joint_commands:=", isaac_joint_commands, " ",
        "isaac_joint_states:=", isaac_joint_states,
    ])
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # Controller params: the shared file both simulators use, then the Isaac
    # deltas on top (later params files win). The only delta today is
    # enable_odom_tf=false, because the Isaac bridge broadcasts
    # odom->base_footprint itself. Everything else — wheel geometry, the MiR100
    # speed limits, the arm/gripper controllers — stays in the shared file so it
    # cannot drift between the two sims.
    controllers_file = PathJoinSubstitution(
        [mir_description_share, "config", "diffdrive_controller.yaml"]
    )
    controllers_file_isaac = PathJoinSubstitution(
        [mir_description_share, "config", "diffdrive_controller_isaac.yaml"]
    )

    # ----------------------------------------------------- robot_state_publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )
    ld.add_action(robot_state_publisher)

    # --------------------------------------------------------- controller_manager
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_file, controllers_file_isaac,
                    {"use_sim_time": use_sim_time}],
        output="both",
    )
    ld.add_action(control_node)

    # ------------------------------------------------------------ spawners
    def spawner(name, extra=None):
        args = [name, "--controller-manager", "/controller_manager"]
        if extra:
            args += extra
        return Node(package="controller_manager", executable="spawner",
                    arguments=args, parameters=[{"use_sim_time": use_sim_time}])

    joint_broadcaster = spawner("joint_broadcaster")
    diff_cont = spawner("diff_cont")
    arm_controller = spawner("joint_trajectory_controller")
    gripper_controller = spawner("gripper_position_controller")

    # Start broadcaster first, then the command controllers, to avoid races.
    ld.add_action(joint_broadcaster)
    ld.add_action(RegisterEventHandler(OnProcessExit(
        target_action=joint_broadcaster,
        on_exit=[diff_cont, arm_controller, gripper_controller])))

    # --------------------------------------------------------------- MoveIt2
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            Command([
                PathJoinSubstitution([FindExecutable(name="cat")]), " ",
                PathJoinSubstitution([moveit_config_share, "srdf", "mir_100.srdf"]),
            ]),
            value_type=str,
        )
    }

    # kinematics.yaml uses the ROS2 param-file format (/**:ros__parameters:...).
    # Unwrap it so move_group receives robot_description_kinematics.ur_manipulator.*
    # rather than robot_description_kinematics./**:ros__parameters:...
    _kin_raw = load_yaml("ur_moveit_config", "config/kinematics.yaml") or {}
    if "/**" in _kin_raw:
        robot_description_kinematics = _kin_raw["/**"]["ros__parameters"]
    else:
        robot_description_kinematics = _kin_raw

    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization "
                                "default_planner_request_adapters/ResolveConstraintFrames "
                                "default_planner_request_adapters/FixWorkspaceBounds "
                                "default_planner_request_adapters/FixStartStateBounds "
                                "default_planner_request_adapters/FixStartStateCollision "
                                "default_planner_request_adapters/FixStartStatePathConstraints",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_yaml = load_yaml("ur_moveit_config", "config/ompl_planning.yaml")
    if ompl_yaml:
        ompl_planning_pipeline_config["move_group"].update(ompl_yaml)

    controllers_yaml = load_yaml("ur_moveit_config", "config/controllers.yaml")
    # Both simulators spawn joint_trajectory_controller. The shared controller
    # file defaults to the hardware-only passthrough controller, so select the
    # simulator controller explicitly for MoveIt trajectory execution.
    controllers_yaml["passthrough_trajectory_controller"]["default"] = False
    controllers_yaml["scaled_joint_trajectory_controller"]["default"] = False
    controllers_yaml["joint_trajectory_controller"]["default"] = True
    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager":
            "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        # Isaac drives the arm with a finite-stiffness PD position drive, so the
        # live joint state has a small steady-state error vs. the commanded
        # value and keeps jittering at the ~0.01 rad level. The default 0.01 rad
        # start tolerance rejects execution ("start point deviates from current
        # robot state"); 0.1 rad is the value the UR MoveIt config uses for sim.
        "trajectory_execution.allowed_start_tolerance": 0.1,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_moveit")),
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim_time},
        ],
    )
    ld.add_action(move_group_node)

    # ---------------------------------------------------------------- RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
        arguments=["-d", LaunchConfiguration("rviz_config_file")],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {"use_sim_time": use_sim_time},
        ],
    )
    ld.add_action(rviz_node)

    # -------------------------------------------------------- scan merger
    # Isaac publishes /f_scan + /b_scan; Nav2/AMCL want a single /scan. Gazebo
    # emits /scan directly, so this is only needed on the Isaac side.
    scan_merger = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([mir_description_share, "launch",
                                  "mir_isaac_scan_merger.launch.py"])]),
        condition=IfCondition(LaunchConfiguration("launch_scan_merger")),
    )
    ld.add_action(scan_merger)

    return ld
