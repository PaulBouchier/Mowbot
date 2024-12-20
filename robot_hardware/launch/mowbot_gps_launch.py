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
          # T-join in ramp on Paul's driveway
          DeclareLaunchArgument('local_easting_origin',     default_value='692371.60'),
          DeclareLaunchArgument('local_northing_origin',    default_value='13670689.64'),

    Node(
        package='lla_utm_local',
        executable='lla_utm_local',
        name='lla_utm_local',
        parameters= [{
                'local_easting_origin':     LaunchConfiguration('local_easting_origin'),
                'local_northing_origin':    LaunchConfiguration('local_northing_origin'),
        }]
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
