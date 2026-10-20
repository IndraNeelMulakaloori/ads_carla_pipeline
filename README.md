# ADS Development Pipeline for CARLA

This workspace contains a ROS 2 CARLA integration and stereo-vision pipeline for generating a colored 3D point cloud from synchronized stereo RGB images. The repository currently includes two ROS packages:

- `l2_carla_demo`: CARLA-to-ROS message conversion utilities and the lecture demo bridge
- `stereo_vision`: CARLA sensor spawning, TF setup, and stereo processing

## Repository layout

```text
ads_development/
├── README.md
├── LICENSE
├── l2_carla_demo/
│   ├── launch/
│   ├── config/
│   ├── rviz/
│   ├── l2_carla_demo/
│   ├── package.xml
│   └── setup.py
├── stereo_vision/
│   ├── launch/
│   ├── config/
│   ├── rviz/
│   ├── urdf/
│   ├── stereo_vision/
│   ├── package.xml
│   └── setup.py
├── results/
└── ...
```

## What the project does

The system connects to CARLA, spawns an ego vehicle with a sensor suite, and republishes sensor data on ROS topics. The `stereo_vision` package then synchronizes left and right RGB camera images, computes stereo disparity, and publishes a colored point cloud on `/carla/stereo/points`.

The depth estimate follows the standard stereo pinhole relation:

$Z = \frac{fB}{d}$

where:
- $f$ is focal length in pixels
- $B$ is the stereo baseline in meters
- $d$ is the disparity in pixels

The current configuration uses a 0.5 m baseline between the left and right RGB cameras and derives the focal length from the configured camera width and FOV.

## ROS topics

| Data | Topic | Message type |
| --- | --- | --- |
| Left RGB image | `/carla/camera/rgb/left/image` | `sensor_msgs/Image` |
| Right RGB image | `/carla/camera/rgb/right/image` | `sensor_msgs/Image` |
| Stereo point cloud | `/carla/stereo/points` | `sensor_msgs/PointCloud2` |
| LiDAR point cloud | `/carla/lidar/points` | `sensor_msgs/PointCloud2` |
| GNSS fix | `/carla/gnss/fix` | `sensor_msgs/NavSatFix` |
| IMU data | `/carla/imu/data` | `sensor_msgs/Imu` |

## Requirements

- ROS 2 with `colcon`, `rosdep`, and the ROS package dependencies installed
- CARLA Simulator with the Python API available inside the same Python environment used for ROS 2
- `l2_carla_demo` available in the workspace for CARLA-to-ROS conversion helpers
- Python packages such as OpenCV, NumPy, and `message_filters`
- Compatible CARLA and ROS 2 versions. The CARLA server should be started before launching the ROS nodes.

## Build

From the ROS 2 workspace root:

```bash
cd ~/carla_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to l2_carla_demo stereo_vision
source install/setup.bash
```

Source `~/carla_ws/install/setup.bash` in each new terminal before running the package.

## Run

### 1. Start CARLA

From the CARLA installation directory:

```bash
./CarlaUE4.sh -quality-level=Low -nosound -vulkan -RenderOffScreen
```

Wait until the server is ready before continuing.

### 2. Start the sensor manager

In a new terminal:

```bash
cd ~/carla_ws
source install/setup.bash
ros2 launch stereo_vision sensors_launch.py
```
By default, it runs on `autopilot:=true` mode
This launch file starts the `sensor_manager` node, publishes the configured sensor data, and loads the robot URDF for TF publishing. Optional arguments:

```bash
ros2 launch stereo_vision sensors_launch.py autopilot:=false
ros2 launch stereo_vision sensors_launch.py config:=/absolute/path/to/stereo_vision/config/carla_config.yaml
```

### 3. Start stereo processing

In a separate sourced terminal:

```bash
cd ~/carla_ws
source install/setup.bash
ros2 run stereo_vision stereo_camera
```

The stereo node must be running alongside the CARLA sensor manager to publish `/carla/stereo/points`.

### 4. Open RViz2

```bash
cd ~/carla_ws
source install/setup.bash
rviz2 -d "$(ros2 pkg prefix stereo_vision)/share/stereo_vision/rviz/stereo_vision.rviz"
```

The configured fixed frame is `ego_vehicle`. If the point cloud is not visible, verify that CARLA is running, the `sensor_manager` node is active, and the `stereo_camera` node is publishing `/carla/stereo/points`.

![RViz2 view of the sensor suite and stereo point cloud](results/stereo_vision_rviz.png)

## Important configuration notes

- The default sensor configuration lives in `stereo_vision/config/carla_config.yaml`
- The left and right RGB cameras are mounted with a horizontal baseline in the YAML file
- `sensor_manager` uses the static transforms from the mount definitions to publish a consistent TF tree
- `stereo_vision` imports conversion helpers from the `l2_carla_demo` package rather than reimplementing CARLA message conversions

## Roadmap

- [x] Spawn CARLA ego vehicle and sensor suite
- [x] Publish synchronized RGB, LiDAR, GNSS, and IMU topics
- [x] Generate a colored stereo point cloud
- [ ] Port core logic to C++
- [ ] Benchmark against LiDAR ground truth
- [ ] Add EKF-based estimation
- [ ] Add 3D detection and tracking
- [ ] Extend to planning and motion-control tasks

## Credits and acknowledgments

This repository builds on the CARLA-ROS starter work from **Dr. Zeid Kootbally** and the public `l2_carla_demo` material used to convert CARLA data into ROS topics. The conversion layer and early architecture remain foundational to this project.

## License

This project is licensed under the Apache License. See [LICENSE](LICENSE) for details.
