from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    namespace = LaunchConfiguration('namespace', default='')

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='ira_laser_tools',
            name='mir_laser_scan_merger',
            executable='laserscan_multi_merger',
            parameters=[{'laserscan_topics': "b_scan f_scan",
                         'destination_frame': "virtual_laser_link",
                         'scan_destination_topic': "scan",
                         'cloud_destination_topic': "scan_cloud",
                         'min_height': -0.25,
                         'max_completion_time': 0.05,
                         'max_merge_time_diff': 0.005,
                         # The merger defaults range_max to DBL_MAX, which
                         # serialises into the LaserScan as inf. Beams with no
                         # return are published as range_max + epsilon, and
                         # nav2's costmap uses exactly those to raytrace-clear
                         # stale obstacles -- with inf it never clears any, so
                         # phantom obstacles stay in the local costmap forever.
                         # The SICK S300 reports 29 m, so say 29 m.
                         'range_min': 0.05,
                         'range_max': 29.0,
                         # SICK S300 Gazebo plugins publish at 12.5 Hz.
                         'scan_time': 0.08,
                         'use_sim_time': LaunchConfiguration('use_sim_time'),
                         'best_effort': False}],
            namespace=namespace,    # adds namespace to topic names and frames
            output='screen')
    ])
