from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution


def generate_launch_description():
   return LaunchDescription([
    # Declare arguments with default values
          # Origin at T on driveway ramp from road
          # DeclareLaunchArgument('local_easting_origin',     default_value='692371.60'),
          # DeclareLaunchArgument('local_northing_origin',    default_value='13670689.64'),
          # Origin at east end of Wu-Weng bridge
          DeclareLaunchArgument('local_easting_origin',     default_value='692374.70'),
          DeclareLaunchArgument('local_northing_origin',    default_value='13670628.07'),

    # fuse_nav_data has multiple modes of fusing gps utm data and wheel odometry and IMU/compass data
    # Mode 0: odom-only, no gps
    # Mode 1: gps-only, heading derived from successive gps readings
    # Mode 2: gps for x-y, IMU for heading, calibrated from gps
    Node(
        package='robot_hardware',
        executable='fuse_nav_data',
        name='fuse_nav_data',
        output='screen',
        emulate_tty=True
    ),
    Node(
        package='robot_hardware',
        executable='esp_link',
        name='esp_link',
        output='screen',
        emulate_tty=True
    ),
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('scripted_bot_driver'),
                'launch',
                'servers_launch.py'
            ])
        ]),
    )
   ])
