#!/usr/bin/env python3
"""


Connects to CARLA, spawns the ego vehicle, attaches the sensor suite, and
republishes every stream on a ROS 2 topic.

"""

import math

import carla
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, NavSatFix, Imu
from tf2_ros import StaticTransformBroadcaster

# CARLA-to-ROS conversions (buffer layouts, left- to right-handed axis flip,
# timestamps) are imported from l2_carla_demo, the package behind the L2
# lecture demo, rather than reimplemented here.
from l2_carla_demo.conversions import (
    camera_info_msg,
    transform_from_carla,
)
from .sensor_groups.cameras import (
    _attach_rgb_camera,
    
)

from .sensor_groups.lidar_radar import (
    _attach_lidar,
)

from .sensor_groups.imu_gnss import (
    _attach_imu,
    _attach_gnss

)

EGO_FRAME = 'ego_vehicle'


class SensorManager(Node):

    def __init__(self):
        super().__init__('sensor_manager')

        # ------------------------------------------------------------------
        # Parameters. Declare everything; read nothing from a literal.
        # ------------------------------------------------------------------
        self.declare_parameter('host', 'localhost')
        self.declare_parameter('port', 2000)
        self.declare_parameter('timeout_s', 10.0)
        self.declare_parameter('synchronous_mode', True)
        self.declare_parameter('fixed_delta_seconds', 0.05)
        self.declare_parameter('town', 'Town01')
        self.declare_parameter('vehicle_blueprint', 'vehicle.tesla.model3')
        self.declare_parameter('spawn_index', 0)
        self.declare_parameter('traffic_manager_port', 8000)
        self.declare_parameter('autopilot', False)
        # Mount declarations
        self.mount_sensors = [
            'camera_rgb_front',
            'camera_depth_front',
            'camera_semseg_front',
            'lidar_top',
            'radar_front',
            'gnss_link',
            'imu_link',
        ]
        axes = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']

        for sensor in self.mount_sensors:
            for axis in axes:
                self.declare_parameter(f'mounts.{sensor}.{axis}', 0.0)

        # Caemra attribute declarations
        camera_defaults = {
            'width': 0,
            'height': 0,
            'fov': 0.0,
            'sensor_tick': 0.00,
        }
        camera_prefixes = ['camera', 'depth_camera', 'semseg_camera']

        for prefix in camera_prefixes:
            for attr, default_val in camera_defaults.items():
                self.declare_parameter(f'{prefix}.{attr}', default_val)
        # AI Assisted : Populated the fields
        sensor_attributes = {
            # LiDAR
            'lidar.channels': 0,
            'lidar.range': 0.0,
            'lidar.points_per_second': 0,
            'lidar.rotation_frequency': 0.0,
            'lidar.upper_fov': 0.0,
            'lidar.lower_fov': 0.0,
            'lidar.sensor_tick': 0.0,

            # Radar
            'radar.horizontal_fov': 0.0,
            'radar.vertical_fov': 0.0,
            'radar.range': 0.0,
            'radar.points_per_second': 0,
            'radar.sensor_tick': 0.0,

            # GNSS
            'gnss.noise_lat_stddev': 0.0,
            'gnss.noise_lon_stddev': 0.0,
            'gnss.noise_alt_stddev': 0.0,
            'gnss.sensor_tick': 0.0,

            # IMU
            'imu.noise_accel_stddev_x': 0.0,
            'imu.noise_accel_stddev_y': 0.0,
            'imu.noise_accel_stddev_z': 0.0,
            'imu.noise_gyro_stddev_x': 0.0,
            'imu.noise_gyro_stddev_y': 0.0,
            'imu.noise_gyro_stddev_z': 0.0,
            'imu.sensor_tick': 0.0,
        }

        for param, value in sensor_attributes.items():
            self.declare_parameter(param, value)

        self.actors = []
        self.world = None
        self.traffic_manager = None
        # The server settings as we found them. destroy() restores these, so
        # keep the original around from the moment you change anything.
        self._original_settings = None

        # ------------------------------------------------------------------
        # Connect
        # ------------------------------------------------------------------
        try:
            host = self.get_parameter('host').value
            port = self.get_parameter('port').value
            self.client = carla.Client(
                host,
                port)
            self.client.set_timeout(self.get_parameter('timeout_s').value)
            # Load the map explicitly. spawn_index indexes into this map's
            # spawn point list, so inheriting whatever the server had loaded
            # would silently move your vehicle somewhere else.
            self.world = self.client.load_world(
                self.get_parameter('town').value)

        except RuntimeError as exc:
            self.get_logger().error(
                f'No CARLA server at {host}:{port} ({exc}). '
                'Start CarlaUE4 and check host/port in the YAML.')
            raise

        self.blueprints = self.world.get_blueprint_library()

        # Everything from here on changes the server. If any of it fails,
        # hand the server back before the exception propagates; main() never
        # gets a node object to clean up when __init__ raises.
        try:
            self._setup()
        except Exception:
            self.destroy()
            raise
        self.get_logger().info('sensor_manager ready')
        # for attr in self.actors:
        #                 self.get_logger().info(f"{attr}")

    def _setup(self):
        # ------------------------------------------------------------------
        # put the server into synchronous mode BEFORE anything else.
        #
        self._original_settings = self.world.get_settings()
        settings = self.world.get_settings()
        # Apply synchronous mode and the fixed time step from the YAML.
        settings.synchronous_mode = self.get_parameter(
            'synchronous_mode').value
        settings.fixed_delta_seconds = self.get_parameter(
            'fixed_delta_seconds').value
        self.world.apply_settings(settings)
        #
        # Retrofitting this later means re-checking every callback you have
        # written by then. destroy() restores _original_settings, so the
        # next person to connect does not inherit a server that will not
        # advance.
        # ------------------------------------------------------------------

        # ------------------------------------------------------------------
        # Publishers. One per stream, on the topics the launch file expects.
        # ------------------------------------------------------------------
        self.pub_rgb = self.create_publisher(
            msg_type=Image, topic='/carla/camera/rgb/image', qos_profile=0)
        self.pub_lidar = self.create_publisher(
            msg_type=PointCloud2, topic='/carla/lidar/points', qos_profile=10)
        self.pub_gnss = self.create_publisher(
            msg_type=NavSatFix, topic='/carla/gnss/fix', qos_profile=10)
        self.pub_imu = self.create_publisher(
            msg_type=Imu, topic='/carla/imu/data', qos_profile=10)

        self.tf_static = StaticTransformBroadcaster(self)

        self._spawn_vehicle()
        self._publish_static_tf()
        self._attach_sensors()
        # autopilot is off by default; record_launch.py turns it on
        # through the launch argument autopilot:=true.
        if self.get_parameter('autopilot').value:
            self._enable_autopilot()
        self._verify_carla_sensors()
        if self.get_parameter('synchronous_mode').value:
            dt = self.get_parameter('fixed_delta_seconds').value
            self.timer = self.create_timer(dt, self._tick)

    # ----------------------------------------------------------------------
    def _spawn_vehicle(self):
        """Spawn the ego vehicle at the configured spawn point."""
        # Retrieved the spawing point of the vehicle
        spawn_point = self.world.get_map().get_spawn_points()[
            self.get_parameter('spawn_index').value]
        # Retrive car
        vehicle_bp = self.blueprints.find(
            self.get_parameter('vehicle_blueprint').value)
        # Tag it so external viewers (spectator_view) can find it.
        vehicle_bp.set_attribute('role_name', 'ego')
        self.vehicle = self.world.try_spawn_actor(vehicle_bp, spawn_point)
        if self.vehicle is None:
            raise SystemExit("Every spawn point is occupied")

        # Append car to actors list
        self.actors.append(self.vehicle)
        self.get_logger().info(
            f"Launched {self.get_parameter('vehicle_blueprint').value} at {self.get_parameter('spawn_index').value}")

    # ----------------------------------------------------------------------
    def _enable_autopilot(self):
        """Hand the ego vehicle to the Traffic Manager ."""
        # The Traffic Manager drives autopilot vehicles. With the server in
        # synchronous mode it must be synchronous too, or the vehicle stalls.
        tm_port = self.get_parameter('traffic_manager_port').value
        self.traffic_manager = self.client.get_trafficmanager(tm_port)
        self.traffic_manager.set_synchronous_mode(True)
        self.vehicle.set_autopilot(True, tm_port)
        # self.traffic_manager.ignore_lights_percentage(self.vehicle, 100) # debugging param
        self.get_logger().info(
            f'Autopilot enabled (Traffic Manager port {tm_port}).')

    # mount helper from https://enpm818z-fall-2026-docs.readthedocs.io/en/latest/assignments/gp1.html#:~:text=def%20_mount(,%27yaw%27%5D))
    def _mount(self, sensor: str) -> carla.Transform:
        """The carla.Transform for mounts.<name> in the YAML."""
        pose = {key.split('.')[-1]: val.value for key, val in
                self.get_parameters_by_prefix(f'mounts.{sensor}').items()}

        return carla.Transform(
            carla.Location(x=pose['x'], y=pose['y'], z=pose['z']),
            carla.Rotation(roll=pose['roll'], pitch=pose['pitch'],
                           yaw=pose['yaw']))

    # ----------------------------------------------------------------------
    def _publish_static_tf(self):
        """Publish the sensor mounts as a static TF tree.

        RViz2 cannot draw a point cloud and a camera image in the same scene
        until it knows where those sensors sit relative to each other. Skip
        this and shows "Fixed Frame does not exist" and nothing else.
        """
        #
        #         transform_from_carla(carla_transform, parent, child,
        #                              sim_time) does the conversion for you;
        #         sim_time is world.get_snapshot().timestamp.elapsed_seconds.
        #
        #         Check the result with: ros2 run tf2_tools view_frames
        now = self.world.get_snapshot().timestamp.elapsed_seconds

        # For every sensor in the yaml file,
        # it converts the CARLA convetion (left handed) to ros2 convention
        # and published as a tf2 transform
        self.tf_static.sendTransform(
            [
                transform_from_carla(self._mount(
                    sensor), EGO_FRAME, sensor, now)
                for sensor in self.mount_sensors
            ])
        self.get_logger().info(f"Brodcasting Static TF Frames...")

    # ----------------------------------------------------------------------

    def _attach_sensors(self):
        """Attach every sensor in the suite to the ego vehicle."""
        _attach_rgb_camera(
            node=self,
            world=self.world,
            blueprints=self.blueprints,
            vehicle=self.vehicle,
            actors_list=self.actors,
            msg_type=Image,
            publisher=self.pub_rgb
        )
        
        _attach_lidar(
            node=self,
            world=self.world,
            blueprints=self.blueprints,
            vehicle=self.vehicle,
            actors_list=self.actors,
            publisher=self.pub_lidar
        )
       
        _attach_gnss(
            node=self,
            world=self.world,
            blueprints=self.blueprints,
            vehicle=self.vehicle,
            actors_list=self.actors,
            publisher=self.pub_gnss
        )
        _attach_imu(
            node=self,
            world=self.world,
            blueprints=self.blueprints,
            vehicle=self.vehicle,
            actors_list=self.actors,
            publisher=self.pub_imu
        )

        self.get_logger().info(f"All sensors successfully attached..")
        

    def _verify_carla_sensors(self):
        """
        Verification method to check all sensors appear in CARLA
        """
        self._tick()
        sensor_actors = self.world.get_actors().filter('sensor.*')
        for sensor in sensor_actors:
            self.get_logger().info(
                f"{sensor.type_id} : Found & Verified in CARLA world")
    

    def _tick(self):
        """Advance the simulation by one step. Synchronous mode only."""
        self.world.tick()

    
    def destroy(self):
        """Destroy every actor and hand the server back as we found it."""
        self.get_logger().info(f'destroying {len(self.actors)} actors')
        # Stop every sensor first, then destroy actors in reverse spawn order
        # so no sensor is left attached to a destroyed parent. Leaked sensors
        # keep using server resources until the map is reloaded.


        # Check the actor type by it's id and
        # stop if it's a sensor type
        for actor in self.actors:
            if actor.type_id.startswith('sensor.'):
                actor.stop()
        for actor in reversed(self.actors):
            if actor is not None and actor.is_alive:
                actor.destroy()
        self.actors.clear()

        # undo autopilot before restoring world settings, or the
        # Traffic Manager keeps waiting for ticks that never come.
        if self.traffic_manager is not None:
            self.traffic_manager.set_synchronous_mode(False)
            self.traffic_manager = None

        # Hand the clock back. Left in synchronous mode with nothing ticking
        # it, the server looks frozen to the next client.
        if self.world is not None and self._original_settings is not None:
            self.world.apply_settings(self._original_settings)
            self._original_settings = None


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SensorManager()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
