"""Turn CARLA sensor data into ROS 2 messages.

There is one idea running through this whole file, and it is the same one as
the axis-permutation slide: two systems name the same three physical
directions differently, and nothing warns you when you get it wrong.

    CARLA      x forward, y RIGHT, z up   (left-handed)
    ROS REP-103  x forward, y LEFT,  z up   (right-handed)

So every y flips sign, and so does every rotation about x and z.  Skip it and
RViz shows a scene that looks fine until you notice the traffic is driving on
the wrong side of the road.
"""

import math

import numpy as np
from geometry_msgs.msg import Quaternion, TransformStamped
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus, PointCloud2, PointField
from std_msgs.msg import Header


## AI Generated code : Semantic Segmentation Color Palette
# Define the CityScapes Color Map  (IDs 0 to 22)
SEMANTIC_COLOR_PALETTE = np.array([
    [  0,   0,   0, 255],  # 0: Unlabeled (Black)
    [ 70,  70,  70, 255],  # 1: Building,  # 2: Fence
    [ 55,  90,  80, 255],  # 3: Other,  # 4: Pedestrian,  # 5: Pole,  # 6: Road line,  # 7: Road,  # 8: Sidewalk,  # 9: Vegetation
    [  0,   0, 142, 255],  # 10: Vehicle,  # 11: Wall,  # 12: Traffic sign
    [ 70, 130, 180, 255],  # 13: Sky
    [ 81,   0,  81, 255],  # 14: Ground,  # 15: Bridge,  # 16: Rail track,  # 17: Guard rail,  # 18: Traffic light,  # 19: Static,  # 20: Dynamic
    [ 45,  60, 150, 255],  # 21: Water
    [145, 170, 100, 255]   # 22: Terrain
], dtype=np.uint8)


def carla_to_ros_point(x: float, y: float, z: float) -> tuple[float, float, float]:
    """One point, from CARLA's left-handed frame into ROS's right-handed one."""
    return x, -y, z


def stamp_from(sim_time: float, clock):
    """A ROS time built from CARLA's simulation clock, not the wall clock.

    In synchronous mode the simulator owns time.  Stamping with the wall clock
    makes every message look late by however long the last tick took, and the
    tf lookups that follow start failing for reasons that have nothing to do
    with the transforms.
    """
    del clock  # kept in the signature so the intent is visible at call sites
    seconds = int(sim_time)
    return seconds, int((sim_time - seconds) * 1e9)


def header(frame_id: str, sim_time: float) -> Header:
    h = Header()
    h.frame_id = frame_id
    sec, nsec = stamp_from(sim_time, None)
    h.stamp.sec = sec
    h.stamp.nanosec = nsec
    return h


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Radians, ROS convention, ZYX order."""
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


def transform_from_carla(carla_transform, parent: str, child: str,
                         sim_time: float) -> TransformStamped:
    """A carla.Transform published as a tf2 transform.

    CARLA reports rotations in degrees as (pitch, yaw, roll) and about a
    left-handed frame, so pitch and yaw both change sign on the way to ROS.
    """
    t = TransformStamped()
    t.header = header(parent, sim_time)
    t.child_frame_id = child

    loc = carla_transform.location
    t.transform.translation.x = float(loc.x)
    t.transform.translation.y = float(-loc.y)
    t.transform.translation.z = float(loc.z)

    rot = carla_transform.rotation
    t.transform.rotation = quaternion_from_euler(
        math.radians(rot.roll),
        math.radians(-rot.pitch),
        math.radians(-rot.yaw),
    )
    return t


# --------------------------------------------------------------------------
# camera
# --------------------------------------------------------------------------
def image_to_msg(image, frame_id: str, msg_class):
    """carla.Image to sensor_msgs/Image.

    CARLA delivers BGRA. Publishing as 'bgra8' rather than converting is both
    cheaper and honest: the encoding field says what the bytes are, and
    cv_bridge and RViz both handle it.
    """
    msg = msg_class()
    msg.header = header(frame_id, image.timestamp)
    msg.height = image.height
    msg.width = image.width
    msg.encoding = "bgra8"
    msg.is_bigendian = 0
    msg.step = 4 * image.width
    msg.data = bytes(image.raw_data)
    return msg


## AI Generated Code : CARLA delivers semantic segmentation images in BGRA format, 
# where the Red channel contains the semantic IDs. 
# We will map these IDs to colors using the SEMANTIC_COLOR_PALETTE defined above.
def semantic_image_to_msg(image, frame_id: str, msg_class):
    """Converts a CARLA Semantic Segmentation image into a visible BGRA8 ROS message."""
    msg = msg_class()
    msg.header = header(frame_id, image.timestamp)
    msg.height = image.height
    msg.width = image.width
    msg.encoding = "bgra8"
    msg.is_bigendian = 0
    msg.step = 4 * image.width


    # CARLA returns data in flat format(image_height * image_width * 4)
    #  we need to reshape it to (height, width, 4)
    raw_array = np.frombuffer(image.raw_data, dtype=np.uint8)
    bgra_img = raw_array.reshape((image.height, image.width, 4))

    # Extract the semantic tags from the Red channel (Index 2 in BGRA)
    semantic_tags = bgra_img[:, :, 2]

    # Prevent crashes by clipping out-of-bounds tags
    semantic_tags = np.clip(semantic_tags, 0, len(SEMANTIC_COLOR_PALETTE) - 1)

    # Map the IDs to human-readable colors
    colored_img = SEMANTIC_COLOR_PALETTE[semantic_tags].astype(np.uint8)

    # Convert back to raw bytes for the ROS 2 message
    msg.data = colored_img.tobytes()
    return msg


def camera_info_msg(image, frame_id: str, fov_deg: float, msg_class):
    """The intrinsics, published alongside the image.

    A CARLA camera is an ideal pinhole: square pixels, principal point exactly
    at the centre, no distortion. So D is zeros, and the calibration section is
    about everything this simulator does not make you do.
    """
    f = image.width / (2.0 * math.tan(math.radians(fov_deg) / 2.0))
    cx, cy = image.width / 2.0, image.height / 2.0

    info = msg_class()
    info.header = header(frame_id, image.timestamp)
    info.height = image.height
    info.width = image.width
    info.distortion_model = "plumb_bob"
    info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    info.k = [f, 0.0, cx,
              0.0, f, cy,
              0.0, 0.0, 1.0]
    info.r = [1.0, 0.0, 0.0,
              0.0, 1.0, 0.0,
              0.0, 0.0, 1.0]
    info.p = [f, 0.0, cx, 0.0,
              0.0, f, cy, 0.0,
              0.0, 0.0, 1.0, 0.0]
    return info


# --------------------------------------------------------------------------
# LiDAR
# --------------------------------------------------------------------------
_LIDAR_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
]


def lidar_to_msg(cloud, frame_id: str) -> PointCloud2:
    """carla.LidarMeasurement to sensor_msgs/PointCloud2."""
    points = np.frombuffer(cloud.raw_data, dtype=np.float32).reshape(-1, 4).copy()
    points[:, 1] *= -1.0          # CARLA y is right, ROS y is left

    msg = PointCloud2()
    msg.header = header(frame_id, cloud.timestamp)
    msg.height = 1                # unordered cloud: one row of N points
    msg.width = points.shape[0]
    msg.fields = _LIDAR_FIELDS
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 16 * msg.width
    msg.is_dense = True
    msg.data = points.astype(np.float32).tobytes()
    return msg


# --------------------------------------------------------------------------
# RADAR
# --------------------------------------------------------------------------
_RADAR_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="velocity", offset=12, datatype=PointField.FLOAT32, count=1),
]


def radar_to_msg(measurement, frame_id: str) -> PointCloud2:
    """carla.RadarMeasurement to a PointCloud2 carrying range rate.

    CARLA reports each detection in spherical coordinates: depth, azimuth,
    altitude, and velocity along the line of sight. The velocity is the
    Doppler measurement from the RADAR slides, and it is the reason a RADAR
    detection is worth more than a range: it is the only sensor here that
    measures speed directly instead of differencing two positions.
    """
    detections = []
    for d in measurement:
        cos_alt = math.cos(d.altitude)
        x = d.depth * cos_alt * math.cos(d.azimuth)
        y = d.depth * cos_alt * math.sin(d.azimuth)
        z = d.depth * math.sin(d.altitude)
        detections.append((x, -y, z, d.velocity))

    points = np.array(detections, dtype=np.float32).reshape(-1, 4)

    msg = PointCloud2()
    msg.header = header(frame_id, measurement.timestamp)
    msg.height = 1
    msg.width = points.shape[0]
    msg.fields = _RADAR_FIELDS
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 16 * msg.width
    msg.is_dense = True
    msg.data = points.tobytes()
    return msg


# --------------------------------------------------------------------------
# IMU and GNSS
# --------------------------------------------------------------------------
def imu_to_msg(measurement, frame_id: str) -> Imu:
    """carla.IMUMeasurement to sensor_msgs/Imu.

    CARLA's compass is a heading in radians measured clockwise from north,
    which is neither the ROS yaw convention nor the same zero, so the
    orientation here is derived from it rather than copied.
    """
    msg = Imu()
    msg.header = header(frame_id, measurement.timestamp)

    a = measurement.accelerometer
    msg.linear_acceleration.x = float(a.x)
    msg.linear_acceleration.y = float(-a.y)
    msg.linear_acceleration.z = float(a.z)

    g = measurement.gyroscope
    msg.angular_velocity.x = float(-g.x)
    msg.angular_velocity.y = float(g.y)
    msg.angular_velocity.z = float(-g.z)

    # compass: 0 at north, increasing clockwise. ROS yaw: 0 at east,
    # increasing counter-clockwise.
    yaw = -measurement.compass + math.pi / 2.0
    msg.orientation = quaternion_from_euler(0.0, 0.0, yaw)

    # A real IMU ships a covariance. CARLA's noise model is whatever you set on
    # the blueprint, so -1 in the first element is the honest answer: unknown.
    msg.orientation_covariance[0] = -1.0
    return msg


def gnss_to_msg(measurement, frame_id: str) -> NavSatFix:
    """carla.GnssMeasurement to sensor_msgs/NavSatFix."""
    msg = NavSatFix()
    msg.header = header(frame_id, measurement.timestamp)
    msg.status.status = NavSatStatus.STATUS_FIX
    msg.status.service = NavSatStatus.SERVICE_GPS
    msg.latitude = float(measurement.latitude)
    msg.longitude = float(measurement.longitude)
    msg.altitude = float(measurement.altitude)
    msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
    return msg
