# Copyright (c) 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Denis Stogl

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, \
                           SetLaunchConfiguration, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.event_handlers import OnProcessExit
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    mir_description_dir = get_package_share_directory('mir_description')
    mir_gazebo_dir = get_package_share_directory('mir_gazebo')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    rviz_config_file = LaunchConfiguration('rviz_config_file')

    ld = LaunchDescription()

    declare_namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace to push all topics into.')

    declare_robot_x_arg = DeclareLaunchArgument(
        'robot_x',
        default_value='0.0',
        description='Spawning position of robot (x)')

    declare_robot_y_arg = DeclareLaunchArgument(
        'robot_y',
        default_value='0.0',
        description='Spawning position of robot (y)')

    declare_robot_yaw_arg = DeclareLaunchArgument(
        'robot_yaw',
        default_value='0.0',
        description='Spawning position of robot (yaw)')

    declare_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty',
        description='Choose simulation world. Available worlds: empty, maze')

    declare_verbose_arg = DeclareLaunchArgument(
        'verbose',
        default_value='false',
        description='Set to true to enable verbose mode for Gazebo.')

    declare_teleop_arg = DeclareLaunchArgument(
        'teleop_enabled',
        default_value='true',
        description='Set to true to enable teleop to manually move MiR around.')

    declare_rviz_arg = DeclareLaunchArgument(
        'rviz_enabled',
        default_value='true',
        description='Set to true to launch rviz.')

    declare_rviz_config_arg = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(
            mir_description_dir, 'rviz', 'mir_visu_full.rviz'),
        description='Define rviz config file to be used.')

    declare_gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Set to "false" to run headless.')
    

    launch_gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'verbose': LaunchConfiguration('verbose'),
            'gui': LaunchConfiguration('gui'),
            'world': [mir_gazebo_dir, '/worlds/', LaunchConfiguration('world'), '.world']
        }.items()
    )

    launch_ur_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ur_simulation_gazebo"), "/launch", "/ur_sim_control.launch.py"]
        ),
        launch_arguments={
            "ur_type": 'ur5e',
            "safety_limits": 'true',
            "runtime_config_package": 'ur_simulation_gazebo',
            "controllers_file": 'ur_controllers.yaml',
            "description_package": 'ur_description',
            "description_file": 'ur.urdf.xacro',
            "tf_prefix": 'ur_',
            "launch_rviz": "false",
        }.items(),
    )

    '''launch_mir_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mir_description_dir, 'launch', 'mir_launch.py')
        )
    )'''

    # robot_description = ParameterValue(
    #     Command(
    #     [
    #         "xacro ", 
    #         os.path.join(mir_description_dir, "urdf", "mir.urdf.xacro")
    #     ]
    #     ),
    #     value_type=str
    # )


    robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("mir_description"), "urdf", "mir.urdf.xacro"]
            ),
        ]
    )

    robot_description2 = {"robot_description": robot_description}
    
    launch_mir_description = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description2],
        # remappings=[
        #     ("/diff_cont/cmd_vel_unstamped", "/cmd_vel"),],
        namespace=LaunchConfiguration('namespace'),
      )

    launch_mir_gazebo_common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mir_gazebo_dir, 'launch',
                         'include', 'mir_gazebo_common.py')
        )
    )

    def process_namespace(context):
        robot_name = "mir_robot"
        try:
            namespace = context.launch_configurations['namespace']
            robot_name = namespace + '/' + robot_name
        except KeyError:
            pass
        return [SetLaunchConfiguration('robot_name', robot_name)]
    
    # robot_controllers = PathJoinSubstitution(
    #     [
    #         FindPackageShare("mir_description"),
    #         "config",
    #         "diffdrive_controller.yaml",
    #     ]
    # )

    
    # control_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[robot_description2, robot_controllers
    #     ],
    #     output="both",
    # )
    
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', LaunchConfiguration('robot_name'),
                   '-topic', 'robot_description',
                   '-x', LaunchConfiguration('robot_x'),
                   '-y', LaunchConfiguration('robot_y'),
                   '-Y', LaunchConfiguration('robot_yaw'),
                   '-b'],  # bond node to gazebo model,
        namespace=LaunchConfiguration('namespace'),
        output='screen')
    
    # diff_drive_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["diff_cont",
    #                "--controller-manager",
    #                "/controller_manager"],
    # )

    joint_broad_spawner = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_broadcaster'],
        output='screen'
    )
    
    diff_drive_spawner= ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'diff_cont'],
        output='screen'
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
         event_handler=OnProcessExit(
             target_action=joint_broad_spawner,
             on_exit=[diff_drive_spawner],
         )
    )

    # joint_broad_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["joint_broadcaster",
    #                "--controller-manager",
    #                "/controller_manager"],
    # )

    

    delayed_joint_broad_spawner = RegisterEventHandler(
         event_handler=OnProcessExit(
             target_action=spawn_robot,
             on_exit=[joint_broad_spawner],
         )
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_position_controller", "-c", "/controller_manager"],
    )

    # moveit_config = (
    #     MoveItConfigsBuilder("custom_robot", package_name="mir_ur_moveit_config")
    #     .robot_description(file_path="config/mir100.urdf.xacro")
    #     .robot_description_semantic(file_path="config/mir100.srdf")
    #     .trajectory_execution(file_path="config/moveit_controllers.yaml")
    #     .robot_description_kinematics(file_path="config/kinematics.yaml")
    #     .planning_scene_monitor(
    #         publish_robot_description= True, publish_robot_description_semantic=True, publish_planning_scene=True
    #     )
    #     .planning_pipelines(
    #         pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
    #     )
    #     .to_moveit_configs()
    # )
    # use_sim_time={"use_sim_time": LaunchConfiguration('use_sim_time')}
    # config_dict = moveit_config.to_dict()
    # config_dict.update(use_sim_time)

    # move_group_node = Node(
    #     package="moveit_ros_move_group",
    #     executable="move_group",
    #     output="screen",
    #     parameters=[config_dict],
    #     arguments=["--ros-args", "--log-level", "info"],
    # )
    
    launch_rviz = Node(
        condition=IfCondition(LaunchConfiguration('rviz_enabled')),
        package='rviz2',
        executable='rviz2',
        output={'both': 'log'},
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    launch_teleop = Node(
        condition=IfCondition(LaunchConfiguration("teleop_enabled")),
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        arguments=['-r', '/cmd_vel:=/diff_cont/cmd_vel_unstamped'],
        prefix='xterm -e')

    ld.add_action(OpaqueFunction(function=process_namespace))
    ld.add_action(declare_namespace_arg)
    ld.add_action(declare_robot_x_arg)
    ld.add_action(declare_robot_y_arg)
    ld.add_action(declare_robot_yaw_arg)
    ld.add_action(declare_sim_time_arg)
    ld.add_action(declare_world_arg)
    ld.add_action(declare_verbose_arg)
    ld.add_action(declare_teleop_arg)
    ld.add_action(declare_rviz_arg)
    ld.add_action(declare_rviz_config_arg)
    ld.add_action(declare_gui_arg)

    
    ld.add_action(joint_broad_spawner)
    ld.add_action(diff_drive_spawner)
    ld.add_action(launch_gazebo_world)
    #ld.add_action(launch_ur_control)
    ld.add_action(launch_mir_description)
    ld.add_action(spawn_robot)
    ld.add_action(launch_mir_gazebo_common)
    ld.add_action(joint_trajectory_controller_spawner)
    ld.add_action(gripper_controller_spawner)
    # ld.add_action(move_group_node)
    #ld.add_action(control_node)
    #ld.add_action(control_node)
    
    
    
    ld.add_action(launch_rviz)
    ld.add_action(launch_teleop)

    return ld
