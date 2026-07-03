import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, \
    SetLaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    mir_driver_dir = get_package_share_directory('mir_driver')
    mir_nav_dir = get_package_share_directory('mir_navigation')

    def find_map_file(context):
        map_arg = context.launch_configurations['map']
        if not map_arg:
            return []
        if(os.path.isfile(os.path.join(mir_nav_dir, 'maps', map_arg))):
            return[SetLaunchConfiguration('map_file', os.path.join(mir_nav_dir, 'maps', map_arg))]
        elif (os.path.isfile(map_arg)):
            return[SetLaunchConfiguration('map_file', map_arg)]

    declare_map_file_argument = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Relative path to map in mir_navigation/maps or full path to map (yaml). '
                    'Only used with use_mir_localization:=false.')

    declare_use_mir_localization_argument = DeclareLaunchArgument(
        'use_mir_localization',
        default_value='true',
        description='Use the MiR onboard localization (same map + pose as the web interface): '
                    'the bridge already relays the robot\'s /map (latched) and its map->odom TF, '
                    'so no local map_server/AMCL is started and no 2D Pose Estimate is needed. '
                    'RViz "2D Pose Estimate" still works: /initialpose is forwarded to the robot. '
                    'Set to false to localize locally with AMCL on the map:= file instead '
                    '(the two map->odom publishers would otherwise fight).')

    declare_rviz_config_file_argument = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(mir_nav_dir, 'rviz', 'mir_nav.rviz'),
        description='Full path to rviz configuration file')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')

    # Real robot: default to gentle speeds (params yaml is 0.8/1.0 which is
    # startling indoors). Override per-run: max_vel_x:=0.5 max_vel_theta:=0.8
    declare_max_vel_x_argument = DeclareLaunchArgument(
        'max_vel_x',
        default_value='0.3',
        description='Max linear speed [m/s] passed to Nav2')

    declare_max_vel_theta_argument = DeclareLaunchArgument(
        'max_vel_theta',
        default_value='0.5',
        description='Max angular speed [rad/s] passed to Nav2')

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("mir_navigation"),
                                   'config', 'mir_mapping_async.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    start_driver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mir_driver_dir, 'launch', 'mir_launch.py')),
        launch_arguments={'rviz_config_file': LaunchConfiguration(
            'rviz_config_file')}.items()
    )

    # map->odom from the robot's own localization (the bridge relays
    # /robot_pose but not the robot's map->odom TF)
    start_mir_localization_tf = Node(
        package='mir_driver',
        executable='mir_localization_tf',
        name='mir_localization_tf',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_mir_localization'))
    )

    launch_amcl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mir_nav_dir, 'launch', 'include', 'amcl.py')),
        launch_arguments={'map': LaunchConfiguration('map_file')}.items(),
        condition=UnlessCondition(LaunchConfiguration('use_mir_localization'))
    )

    launch_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mir_nav_dir, 'launch', 'include', 'navigation.py')),
        # cmd_vel_topic: the real robot listens via twist_stamper on /cmd_vel;
        # navigation.py's default (diff_cont/cmd_vel_unstamped) is for the
        # Gazebo/Isaac diff_drive controller and goes nowhere on hardware.
        launch_arguments={'map_subscribe_transient_local': 'true',
                          'cmd_vel_topic': 'cmd_vel',
                          'max_vel_x': LaunchConfiguration('max_vel_x'),
                          'max_vel_theta': LaunchConfiguration('max_vel_theta')}.items()
    )

    ld = LaunchDescription()

    ld.add_action(declare_map_file_argument)
    ld.add_action(declare_use_mir_localization_argument)
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_max_vel_x_argument)
    ld.add_action(declare_max_vel_theta_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_rviz_config_file_argument)
    ld.add_action(OpaqueFunction(function=find_map_file))

    ld.add_action(start_driver_cmd)
    ld.add_action(start_mir_localization_tf)
    ld.add_action(launch_amcl)
    ld.add_action(launch_navigation)

    return ld
