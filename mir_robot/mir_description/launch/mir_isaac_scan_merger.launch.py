# Merge the MiR's two SICK laser scans into a single /scan for SLAM / Nav2.
#
# Isaac Sim publishes /f_scan and /b_scan (front + back SICK S300). slam_toolbox
# and AMCL expect a single /scan, so we merge them with ira_laser_tools'
# laserscan_multi_merger — the same node the Gazebo stack used (see
# mir_gazebo/launch/include/mir_gazebo_common.py), just split out so it can run
# against the Isaac back-end.
#
# Requires ira_laser_tools (declared in mir_robot/ros2.repos):
#   vcs import < src/mir_robot/mir_robot/ros2.repos src    # pulls ira_laser_tools
#   colcon build --packages-select ira_laser_tools
#
# Usage:
#   Isaac Sim:    ros2 launch mir_description mir_isaac_scan_merger.launch.py
#   Real MiR:     ros2 launch mir_description mir_isaac_scan_merger.launch.py \
#                     use_sim_time:=false best_effort:=true
#   On the physical robot the mir_driver bridge publishes /f_scan and /b_scan
#   with BEST_EFFORT (sensor_data) QoS, so the merger must subscribe BEST_EFFORT
#   too or it silently receives nothing and /scan stays empty.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    best_effort = LaunchConfiguration("best_effort")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("best_effort", default_value="false"),
        Node(
            package="ira_laser_tools",
            name="mir_laser_scan_merger",
            executable="laserscan_multi_merger",
            parameters=[{
                "laserscan_topics": "b_scan f_scan",
                "destination_frame": "virtual_laser_link",
                "scan_destination_topic": "scan",
                "cloud_destination_topic": "scan_cloud",
                "min_height": -0.25,
                # range_max 29.0 = the real SICK S300 range AND the Gazebo value.
                # Matching Gazebo's no-hit handling: the merger pre-fills every
                # output bin with +inf and only overwrites with points whose
                # reprojected range is <= range_max. Isaac's PhysX lidar returns a
                # FINITE point at its own max_range for no-hit beams (not inf), so
                # mir_isaac_sim.py sets the lidar max_range to 30.0 — above this
                # 29.0 plus the ~0.49 m sensor-to-centre offset — so every no-hit
                # beam reprojects past 29.0 and is dropped, leaving the bin at
                # +inf exactly like Gazebo. range_min 0.1 just drops the rare
                # self-return inside the footprint (Gazebo uses 0.05; the 0.05 m
                # difference is well inside the robot and never seen by Nav2).
                "range_min": 0.1,
                "range_max": 29.0,
                "max_completion_time": 0.05,
                "max_merge_time_diff": 0.005,
                "use_sim_time": use_sim_time,
                "best_effort": best_effort,
            }],
            output="screen",
        ),
    ])
