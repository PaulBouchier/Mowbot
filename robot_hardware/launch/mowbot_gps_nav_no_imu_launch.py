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
          DeclareLaunchArgument('local_easting_origin',     default_value='692371.60'),
          DeclareLaunchArgument('local_northing_origin',    default_value='13670689.64'),

    # lla_utm_local takes gps lat/lon coordinates and converts them to utm offset to some local origin
    # specified by local_easting/northing_orgin
    Node(
        package='lla_utm_local',
        executable='lla_utm_local',
        name='lla_utm_local',
        parameters= [{
                'local_easting_origin':     LaunchConfiguration('local_easting_origin'),
                'local_northing_origin':    LaunchConfiguration('local_northing_origin'),
        }]
    ),
    # fuse_nav_data has multiple modes of fusing gps utm data and wheel odometry and IMU/compass data
    # Mode 0: odom-only, no gps
    # Mode 1: gps-only, heading derived from successive gps readings
    # Mode 2: gps for x-y, IMU for heading, calibrated from gps
    Node(
        package='robot_hardware',
        executable='fuse_nav_data',
        name='fuse_nav_data'
    ),
    Node(
        package='robot_hardware',
        executable='esp_link',
        name='esp_link'
    ),
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('scripted_bot_driver'),
                'launch',
                'servers_launch.py'
            ])
        ]),
    ),
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ublox_dgnss'),
                'launch',
                'ublox_rover_hpposllh_navsatfix.launch.py'
            ])
        ]),
    ),
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ntrip_client'),
                'ntrip_client_remap_launch.py'
            ])
        ]),
        launch_arguments={
            'host': 'rtk2go.com',
            'port': '2101',
            'mountpoint': 'VN1',
            'username': 'paul.bouchier-at-gmail-d-com',
            'password': 'unused',
        }.items()
    )
   ])
