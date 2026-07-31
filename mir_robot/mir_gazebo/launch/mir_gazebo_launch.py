import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
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

    # value_type=str is required: without it launch_ros tries to YAML-parse the
    # generated URDF, and any ':' in an XML comment makes that parse fail with
    # "Unable to parse the value of parameter robot_description as yaml".
    robot_description = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [FindPackageShare("mir_description"), "urdf", "mir.urdf.xacro"]
                ),
            ]
        ),
        value_type=str,
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
        # robot_x/robot_y/robot_yaw were declared but never reached
        # spawn_entity, so the robot always appeared at the world origin
        # regardless of what was passed on the command line.
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

    # The UR5 arm has no controller unless we start one, so it goes limp and
    # collapses under gravity: the gripper ends up at z ~= 0.05 m, right in the
    # front laser's plane (z = 0.191 m). The merged /scan then marks the robot's
    # own arm as an obstacle inside its own footprint, DWB's ObstacleFootprint
    # critic rejects every forward trajectory, and Nav2 cannot move the base at
    # all ("Failed to make progress"). Hold the arm in a stowed pose instead.
    arm_controller_spawner = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_trajectory_controller'],
        output='screen'
    )

    # The Robotiq fingers are likewise unactuated without a controller. They
    # flap against their joint limits every physics step, and that vibration is
    # enough to shake the whole base: with the drive wheels commanded to zero
    # the robot still crept and spun across the world at ~0.15 m/s, which wrecks
    # wheel odometry and makes AMCL unable to localise. Holding them still drops
    # the idle drift to ~0.0007 m/s.
    gripper_controller_spawner = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'gripper_position_controller'],
        output='screen'
    )

    stow_arm = ExecuteProcess(
        cmd=['ros2', 'action', 'send_goal',
             '/joint_trajectory_controller/follow_joint_trajectory',
             'control_msgs/action/FollowJointTrajectory',
             '{trajectory: {joint_names: [ur_shoulder_pan_joint,'
             ' ur_shoulder_lift_joint, ur_elbow_joint, ur_wrist_1_joint,'
             ' ur_wrist_2_joint, ur_wrist_3_joint],'
             ' points: [{positions: [0.0, -2.0, 2.4, -2.0, -1.57, 0.0],'
             ' time_from_start: {sec: 6}}]}}'],
        output='screen'
    )

    delayed_arm_controller_spawner = RegisterEventHandler(
         event_handler=OnProcessExit(
             target_action=diff_drive_spawner,
             on_exit=[arm_controller_spawner],
         )
    )

    delayed_gripper_controller_spawner = RegisterEventHandler(
         event_handler=OnProcessExit(
             target_action=arm_controller_spawner,
             on_exit=[gripper_controller_spawner],
         )
    )

    delayed_stow_arm = RegisterEventHandler(
         event_handler=OnProcessExit(
             target_action=gripper_controller_spawner,
             on_exit=[stow_arm],
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

    launch_rviz = Node(
        condition=IfCondition(LaunchConfiguration('rviz_enabled')),
        package='rviz2',
        executable='rviz2',
        output={'both': 'log'},
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
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
    ld.add_action(delayed_arm_controller_spawner)
    ld.add_action(delayed_gripper_controller_spawner)
    ld.add_action(delayed_stow_arm)
    ld.add_action(launch_gazebo_world)
    ld.add_action(launch_mir_description)
    ld.add_action(spawn_robot)
    ld.add_action(launch_mir_gazebo_common)
    #ld.add_action(control_node)
    #ld.add_action(control_node)
    
    
    
    ld.add_action(launch_rviz)
    ld.add_action(launch_teleop)

    return ld
