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

    Node(
        package='lla_utm_local',
        executable='lla_utm_local',
        name='lla_utm_local',
        parameters= [{
                'local_easting_origin':     LaunchConfiguration('local_easting_origin'),
                'local_northing_origin':    LaunchConfiguration('local_northing_origin'),
        }]
    )
   ])
