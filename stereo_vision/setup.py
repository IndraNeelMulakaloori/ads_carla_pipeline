from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'stereo_vision'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zeid Kootbally',
    maintainer_email='zeidk@umd.edu',
    description='Publish a CARLA sensor suite onto ROS 2 topics (ENPM818Z L2).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_manager = stereo_vision.sensor_manager:main',
            'stereo_camera =  stereo_vision.stereo_camera:main'
        ],
    },
)
