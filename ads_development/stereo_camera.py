#!/usr/bin/env python3

import message_filters
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
import math
import numpy as np
from sensor_msgs_py import point_cloud2
from .math_scripts.stereo_math import stereo_math_helper
import cv2 as cv
class StereoCamera(Node):

    def __init__(self, node_name):
        super().__init__(node_name)

        # Parameter declarations (baseline, fov, etc.)
        self.declare_parameter('camera.width', 1280)
        self.declare_parameter('camera.height', 720)
        self.declare_parameter('camera.fov', 90.0)
        self.baseline = 0.5
        self.f, self.cx, self.cy = self.camera_intrinsics()
        # Synchronized subscribers
        self.sub_left = message_filters.Subscriber(
            self, Image, "/carla/camera/rgb/left/image"
        )
        self.sub_right = message_filters.Subscriber(
            self, Image, "/carla/camera/rgb/right/image"
        )

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.sub_left, self.sub_right],
            queue_size=10,
            slop=0.02,
        )
        self.sync.registerCallback(self.stereo_callback)

        # PointCloud publisher
        self.pub_cloud = self.create_publisher(
            PointCloud2, "/carla/stereo/points", 10
        )
        self.get_logger().info(f"Stereo camera processing node initialized.\nfocal length : {self.f:.2f}\nwidth : {self.cx}\nheight : {self.cy}")

    def stereo_callback(self, left_msg: Image, right_msg: Image):
        # Convert ROS Image messages to OpenCV / NumPy arrays
        # Unpack raw byte buffer directly into (H, W, Channels)
        left_raw = np.frombuffer(left_msg.data, dtype=np.uint8).reshape((left_msg.height, left_msg.width, -1))
        right_raw = np.frombuffer(right_msg.data, dtype=np.uint8).reshape((right_msg.height, right_msg.width, -1))

        # Match encoding: if publishers send BGRA/BGR, convert to RGB (or slice channels)
        if left_msg.encoding == 'bgra8':
            left_img_array = cv.cvtColor(left_raw, cv.COLOR_BGRA2RGB)
            right_img_array = cv.cvtColor(right_raw, cv.COLOR_BGRA2RGB)
        elif left_msg.encoding == 'bgr8':
            left_img_array = cv.cvtColor(left_raw, cv.COLOR_BGR2RGB)
            right_img_array = cv.cvtColor(right_raw, cv.COLOR_BGR2RGB)
        else:
            left_img_array = left_raw
            right_img_array = right_raw
        # Compute disparity map using your math helper
        points_3d, colors_3d = stereo_math_helper(
            left_img_array, right_img_array, 
            self.baseline, self.f, self.cx, self.cy
        )
        # Back-project disparity into 3D points (X, Y, Z, RGB)
        # ROS expects RGB colors packed into a single 32-bit memory block
        r = colors_3d[:, 0].astype(np.uint32)
        g = colors_3d[:, 1].astype(np.uint32)
        b = colors_3d[:, 2].astype(np.uint32)
        rgb_packed = (r << 16) | (g << 8) | b
        # Pack into a sensor_msgs/PointCloud2 message
       
        # Stack X, Y, Z, and the packed RGB into an (N, 4) array
        cloud_data = np.column_stack((points_3d, rgb_packed.view(np.float32)))
        
        # Define the binary layout of the message
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        
        # Inherit the exact simulation timestamp from the camera
        header = left_msg.header
        header.frame_id = 'camera_rgb_left_optical'  #         (Ensure header.frame_id = 'camera_rgb_left_optical')
        
        # Build the message using the standard ROS 2 Python library
        cloud_msg = point_cloud2.create_cloud(header, fields, cloud_data)

        # ublish point cloud
        self.pub_cloud.publish(cloud_msg)

        
    def camera_intrinsics(self) -> tuple[float, float, float]:
        """(f, cx, cy) in pixels for a CARLA RGB camera blueprint.

        CARLA cameras are an ideal pinhole: square pixels, principal point exactly
        at the image centre, no distortion.  Real cameras are none of those things,
        which is why the calibration section exists.
        """

        camera_attributes = self.get_parameters_by_prefix('camera')
        w = camera_attributes['width'].value
        h = camera_attributes['height'].value
        fov = camera_attributes['fov'].value
        f = w / (2.0 * math.tan(math.radians(fov) / 2.0))
        return f, w / 2.0, h / 2.0

    
def main(args=None):
    rclpy.init(args=args)
    node_name = 'stereo_camera'
    node = StereoCamera(node_name)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()