import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    mir_driver = get_package_share_directory('mir_driver')

    return LaunchDescription([

      DeclareLaunchArgument(
        'mir_hostname',
        default_value='192.168.12.20',
        description='IP/hostname of the physical MiR robot (rosbridge host).'),

      IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
          os.path.join(mir_driver, 'launch', 'mir_headless_launch.py')),
        launch_arguments={
            'mir_hostname': LaunchConfiguration('mir_hostname')
        }.items(),
      ),

      IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
          os.path.join(mir_driver, 'launch', 'additions_launch.py')),
      )

    ])
