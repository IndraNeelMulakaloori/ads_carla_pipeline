import os
from datetime import datetime
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource)
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Reuse the normal sensor launch; only override autopilot so the car
    # drives itself during the recording.
    ## The snsor launch already is on autopilort 
    ## so we don't need to override it here.
    pkg_share = get_package_share_directory('stereo_vision')
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'sensors_launch.py')),
            launch_arguments={'autopilot': 'true'}.items()
            )
        
    stereo_camera = Node(
        package='stereo_vision',
        executable='stereo_camera',
        name='stereo_camera',
        output='screen')



    # One bag per run under ~/gp1_data, named by start time so repeated
    # recordings never overwrite each other (ros2 bag refuses existing dirs).
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    bag_path = os.path.join(
        os.path.expanduser('~'), 'ads_development', 'stereo_vision',
        f'stereo_vision_{timestamp}')

    # All seven sensor topics, listed explicitly so tf/rviz traffic stays
    # out of the bag. Messages keep their header.stamp (CARLA sim time),
    # which is what the timing analysis in bag_analysis.py relies on.
    bag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '/carla/camera/rgb/left/image',
            '/carla/camera/rgb/right/image',
            '/carla/lidar/points',
            '/carla/stereo/points',
            '/tf',
            '/tf_static',
            '-o', bag_path,
        ],
        output='screen')

    return LaunchDescription([sensors_launch, stereo_camera, bag_record])
