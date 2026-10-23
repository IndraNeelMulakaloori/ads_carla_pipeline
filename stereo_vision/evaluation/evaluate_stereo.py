#!/usr/bin/env python3
import numpy as np
import cv2
import time
import pandas as pd
import os
import sys
import yaml
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

# --- 1. PATH ROUTING ---
current_dir = os.path.dirname(os.path.abspath(__file__))

# Path to src/: evaluation -> stereo_vision -> ads_development -> src (4 levels up)
src_dir = os.path.abspath(os.path.join(current_dir, "../../../.."))

# Append GP1_Team4 ads_pipeline to system path to reuse projection functions
gp1_pipeline_dir = os.path.join(src_dir, "GP1_Team4", "ads_pipeline")
sys.path.append(gp1_pipeline_dir)

from ads_pipeline.lidar_projection import build_intrinsic_matrix, build_extrinsic

# Path to config: evaluation -> stereo_vision -> config -> carla_config.yaml
config_path = os.path.abspath(os.path.join(current_dir, "../config/carla_config.yaml"))

# Path to existing results dir: evaluation -> stereo_vision -> ads_development -> results
results_dir = os.path.abspath(os.path.join(current_dir, "../../results"))


class StereoEvaluator:
    def __init__(self, K, T_lidar_to_cam, baseline, width, height):
        """
        Initializes the evaluation pipeline.
        :param K: 3x3 Intrinsic Matrix
        :param T_lidar_to_cam: 4x4 Extrinsic Transform (LiDAR to Camera)
        :param baseline: Distance between left and right cameras (meters)
        """
        self.K = K
        self.T = T_lidar_to_cam
        self.focal_length = K[0, 0]
        self.baseline = baseline
        self.width = width
        self.height = height
        
        block_size = 5
        # Initialize OpenCV SGBM
        self.stereo = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=128,      
            blockSize=block_size,
            P1=8 * 1 * block_size**2,
            P2=32 * 1 * block_size**2,
            disp12MaxDiff=1,         
            uniquenessRatio=15,      
            speckleWindowSize=100,   
            speckleRange=32          
        )

    def compute_stereo_depth(self, left_img, right_img):
        """Computes stereo depth and tracks real-time throughput metrics."""
        start_time = time.perf_counter()
        
        # Convert to grayscale if necessary
        if len(left_img.shape) == 3:
            left_img = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
            right_img = cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)
            
        # Compute Disparity
        disparity = self.stereo.compute(left_img, right_img).astype(np.float32) / 16.0
        
        # Avoid division by zero for invalid disparities
        disparity[disparity <= 0.0] = 0.0001
        
        # Triangulation: Z = fB / d
        stereo_depth = (self.focal_length * self.baseline) / disparity
        
        # Filter out invalid and infinity values (e.g. beyond 50 meters)
        stereo_depth[disparity == 0.0001] = 0
        stereo_depth[stereo_depth > 50.0] = 0
        
        latency_ms = (time.perf_counter() - start_time) * 1000
        return stereo_depth, latency_ms

    def generate_lidar_ground_truth(self, lidar_points):
        """Projects 3D LiDAR points to 2D image plane for pixel-wise comparison."""
        xyz = lidar_points[:, :3]
        xyz_h = np.hstack((xyz, np.ones((xyz.shape[0], 1))))
        
        # Transform from LiDAR frame to Camera frame
        cam_points = (self.T @ xyz_h.T).T
        
        # Filter points behind the camera (Z <= 0)
        valid_z_mask = cam_points[:, 2] > 0
        cam_points = cam_points[valid_z_mask]
        
        # Project to 2D using Intrinsic matrix (K)
        uv_h = (self.K @ cam_points[:, :3].T).T
        u = (uv_h[:, 0] / uv_h[:, 2]).astype(int)
        v = (uv_h[:, 1] / uv_h[:, 2]).astype(int)
        z = cam_points[:, 2] # True depth
        
        # Filter points outside image bounds
        bounds_mask = (u >= 0) & (u < self.width) & (v >= 0) & (v < self.height)
        u = u[bounds_mask]
        v = v[bounds_mask]
        z = z[bounds_mask]
        
        # Create sparse ground-truth depth map
        lidar_depth_map = np.zeros((self.height, self.width), dtype=np.float32)
        
        # Handle occlusion by sorting Z descending
        sort_idx = np.argsort(z)[::-1]
        u = u[sort_idx]
        v = v[sort_idx]
        z = z[sort_idx]
        
        lidar_depth_map[v, u] = z
        return lidar_depth_map

    def calculate_metrics(self, stereo_depth, lidar_depth):
        """Calculates Density, MAE, and RMSE only on intersecting valid pixels."""
        valid_mask = (stereo_depth > 0) & (lidar_depth > 0)
        
        if not np.any(valid_mask):
            return None
            
        z_pred = stereo_depth[valid_mask]
        z_gt = lidar_depth[valid_mask]
        
        errors = np.abs(z_pred - z_gt)
        mae = np.mean(errors)
        rmse = np.sqrt(np.mean(errors**2))
        
        fill_rate = (np.count_nonzero(stereo_depth > 0) / (self.width * self.height)) * 100
        
        return {
            'mae_m': mae,
            'rmse_m': rmse,
            'fill_rate_pct': fill_rate,
            'valid_comparisons': np.count_nonzero(valid_mask)
        }


def load_config_parameters(yaml_path):
    """Parses carla_config.yaml to dynamically extract sensor geometry."""
    with open(yaml_path, 'r') as file:
        raw_config = yaml.safe_load(file)
        
    # NOTE: Adjust these dictionary keys if your YAML nesting is different
    config = raw_config['sensor_manager']['ros__parameters']
    try:
        cam_sensor = config['camera']
        lidar_sensor = config['lidar']
    except KeyError:
        print("Warning: YAML structure didn't match default expectations. Please check the dictionary keys in load_config_parameters().")
        raise
        
    image_w = cam_sensor['width']
    image_h = cam_sensor['height']
    fov = cam_sensor['fov']

    ## sensor mounts 
    cam_left = config['mounts']['camera_rgb_left_link']
    cam_right = config['mounts']['camera_rgb_right_link']
    lidar = config['mounts']['lidar_top']
    lidar_loc = ([lidar['x'], lidar['y'], lidar['z']])
    camera_loc = ([cam_left['x'], cam_left['y'], cam_left['z']])
    camera_pitch = cam_left['pitch']  # Assuming both cameras have the same pitch
    
    # Calculate baseline dynamically
    baseline = abs(cam_left['y'] - cam_right['y'])
    if baseline == 0:
        baseline = 0.54 # Fallback
        
    return image_w, image_h, fov, baseline, lidar_loc, camera_loc, camera_pitch


def run_offline_evaluation(bag_data_frames):
    """Main evaluation loop."""
    print("Loading parameters from carla_config.yaml...")
    img_w, img_h, fov, baseline, lidar_loc, camera_loc, camera_pitch = load_config_parameters(config_path)
    
    K = build_intrinsic_matrix(image_w=img_w, image_h=img_h, fov_deg=fov)
    
    # Pass arguments exactly as your GP1 lidar_projection.py expects them
    T = build_extrinsic(
        lidar_loc=lidar_loc, 
        camera_loc=camera_loc, 
        camera_pitch_deg=camera_pitch
    )
    
    evaluator = StereoEvaluator(K, T, baseline=baseline, width=img_w, height=img_h)
    results = []
    
    print("Starting Stereo Pipeline Evaluation...")
    for idx, frame in enumerate(bag_data_frames):
        left_img = frame['left']
        right_img = frame['right']
        lidar_pts = frame['lidar'] 
        
        stereo_depth, latency_ms = evaluator.compute_stereo_depth(left_img, right_img)
        lidar_depth = evaluator.generate_lidar_ground_truth(lidar_pts)
        metrics = evaluator.calculate_metrics(stereo_depth, lidar_depth)
        
        if metrics:
            metrics['latency_ms'] = latency_ms
            metrics['fps'] = 1000.0 / latency_ms if latency_ms > 0 else 0
            results.append(metrics)
            
            print(f"Frame {idx}: FPS: {metrics['fps']:.1f} | MAE: {metrics['mae_m']:.3f}m | "
                  f"RMSE: {metrics['rmse_m']:.3f}m | Fill Rate: {metrics['fill_rate_pct']:.1f}%")

    if results:
        df = pd.DataFrame(results)
        print("\n--- FINAL PIPELINE BENCHMARKS ---")
        print(f"Average Throughput: {df['fps'].mean():.1f} FPS ({df['latency_ms'].mean():.1f} ms)")
        print(f"Average Z-Error (MAE): {df['mae_m'].mean():.3f} meters")
        print(f"Average RMSE: {df['rmse_m'].mean():.3f} meters")
        print(f"Average Fill Rate: {df['fill_rate_pct'].mean():.1f}%")
        
        # Save to existing ads_development/results directory
        os.makedirs(results_dir, exist_ok=True)
        csv_path = os.path.join(results_dir, 'stereo_evaluation_metrics.csv')
        df.to_csv(csv_path, index=False)
        print(f"Metrics saved successfully to: {csv_path}")


if __name__ == "__main__":
   # Update this to the folder containing your .db3 file
    bag_dir = '/home/neelmvss/ros_bag_data/stereo_vision/stereo_vision_20261023_143619' 
    
    print(f"Opening ROS bag: {bag_dir}")
    
    storage_options = rosbag2_py.StorageOptions(uri=bag_dir, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

    left_imgs = []
    right_imgs = []
    lidar_clouds = []

    def msg_to_gray(msg):
        """Bypasses cv_bridge to avoid NumPy 2.x crashes in ROS 2 Humble."""
        bpp = msg.step // msg.width
        img_array = np.ndarray(shape=(msg.height, msg.width, bpp), dtype=np.uint8, buffer=msg.data)
        if bpp == 4:
            return cv2.cvtColor(img_array, cv2.COLOR_BGRA2GRAY)
        elif bpp == 3:
            return cv2.cvtColor(img_array, cv2.COLOR_BGR2GRAY)
        return img_array.copy()

    print("Extracting and deserializing messages...")
    import sensor_msgs_py.point_cloud2 as pc2
    
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)
        
        if topic == '/carla/camera/rgb/left/image':
            left_imgs.append(msg_to_gray(msg))
        elif topic == '/carla/camera/rgb/right/image':
            right_imgs.append(msg_to_gray(msg))
        elif topic == '/carla/lidar/points':
            gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            pts = np.array([[p[0], p[1], p[2]] for p in gen], dtype=np.float32)
            lidar_clouds.append(pts)

    print(f"Extracted {len(left_imgs)} left, {len(right_imgs)} right, {len(lidar_clouds)} lidar messages.")
    
    min_len = min(len(left_imgs), len(right_imgs), len(lidar_clouds))
    
    if min_len == 0:
        print("Error: Did not find matching sensor data in the bag.")
        sys.exit(1)
        
    bag_data_frames = []
    for i in range(min_len):
        bag_data_frames.append({
            'left': left_imgs[i],
            'right': right_imgs[i],
            'lidar': lidar_clouds[i]
        })

    print(f"Successfully synchronized {min_len} frames. Starting evaluation...")
    run_offline_evaluation(bag_data_frames)