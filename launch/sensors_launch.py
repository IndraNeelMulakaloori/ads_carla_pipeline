"""Start sensor_manager with the team's YAML configuration.

    ros2 launch ads_pipeline sensors_launch.py
    ros2 launch ads_pipeline sensors_launch.py autopilot:=true
    ros2 launch ads_pipeline sensors_launch.py config:=/path/to/other.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('ads_development')
    default_config = os.path.join(pkg_share, 'config', 'carla_config.yaml')

    config_arg = DeclareLaunchArgument(
        'config', default_value=default_config,
        description='Path to the sensor configuration YAML.')
    autopilot_arg = DeclareLaunchArgument(
        'autopilot', default_value='true',
        description='Let CARLA drive the ego vehicle. '
                    'Your node must declare the autopilot parameter.')

    sensor_manager = Node(
        package='ads_development',
        executable='sensor_manager',
        name='sensor_manager',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('config'),
            # Overrides the YAML value; a later entry wins.
            {'autopilot': ParameterValue(LaunchConfiguration('autopilot'),
                                         value_type=bool)},
        ],
    )

    return LaunchDescription([config_arg, sensor_manager,autopilot_arg])
