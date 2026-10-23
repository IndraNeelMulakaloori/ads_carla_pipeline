"""Start sensor_manager with the team's YAML configuration.

    ros2 launch stereo_vision sensors_launch.py
    ros2 launch stereo_vision sensors_launch.py autopilot:=true
    ros2 launch stereo_vision sensors_launch.py config:=/path/to/other.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.events import Shutdown

def generate_launch_description():
    pkg_share = get_package_share_directory('stereo_vision')
    default_config = os.path.join(pkg_share, 'config', 'carla_config.yaml')

    
    config_arg = DeclareLaunchArgument(
        'config', default_value=default_config,
        description='Path to the sensor configuration YAML.')
    autopilot_arg = DeclareLaunchArgument(
        'autopilot', default_value='false',
        description='Let CARLA drive the ego vehicle. '
                    'Your node must declare the autopilot parameter.')
    ## Robot urdf
    urdf_path = os.path.join(pkg_share, 'urdf', 'car.urdf')
    with open(urdf_path, 'r') as f:
        robot_description_config = f.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config}]
    )

    sensor_manager = Node(
        package='stereo_vision',
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
    stereo_camera = Node(
        package='stereo_vision',
                executable='stereo_camera',
                name='stereo_camera',
                output='screen',
                arguments=['--ros-args'],
            )
   
    return LaunchDescription([config_arg, autopilot_arg, sensor_manager, robot_state_publisher])
