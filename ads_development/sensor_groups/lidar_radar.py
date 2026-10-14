import carla

from l2_carla_demo.conversions import (
    lidar_to_msg,
    radar_to_msg
)

# lidar_radar.py - Lidar and Radar Sensor Group Module
#
# Helper script containing modular setup functions and ROS 2 callbacks for the 
# Lidar and Radar sensors to the ego vehicle, and publishing the point cloud data streams.

def _attach_lidar(node, world, blueprints, vehicle, actors_list, publisher):
    """
    Attach lidar sensor attributes and mount values 
    """
    ## Lidar Sensor attributes
    lidar_bp = blueprints.find('sensor.lidar.ray_cast')
    lidar_attributes = node.get_parameters_by_prefix('lidar')
    for attr, value in lidar_attributes.items():
        if lidar_bp.has_attribute(attr):
            lidar_bp.set_attribute(attr,str(value.value))
        else:
            node.get_logger().warn(f"Attribute {attr} isn't found for {lidar_bp.id}")
    
    
    ## LIDAR Mounts
    lidar_mount = node.get_parameters_by_prefix('mounts.lidar_top')
    mount = carla.Transform(
        carla.Location(x=lidar_mount['x'].value, y=lidar_mount['y'].value, z=lidar_mount['z'].value),     
        carla.Rotation(roll=lidar_mount['roll'].value, pitch=lidar_mount['pitch'].value,yaw=lidar_mount['yaw'].value))
    
    ## Sensor attachment & appendign actor to list
    sensor = world.spawn_actor(
        lidar_bp, mount, attach_to=vehicle,
        attachment_type=carla.AttachmentType.Rigid)

    def _lidar_callback(measurement):
        """
        Republish one lidar point cloud.
        """
        publisher.publish(
            lidar_to_msg(measurement, 'lidar_top'))

    sensor.listen(_lidar_callback)
    actors_list.append(sensor)
    
    ## Logger 
    node.get_logger().info(f"{lidar_bp.id} has been initialised...")


def _attach_radar(node, world, blueprints, vehicle, actors_list, publisher):
    """
    Attach radar sensor attributes and mount values 
    """
    ## Radar Sensor attributes
    radar_bp = blueprints.find('sensor.other.radar')
    radar_attributes = node.get_parameters_by_prefix('radar')
    for attr, value in radar_attributes.items():
        if radar_bp.has_attribute(attr):
            radar_bp.set_attribute(attr,str(value.value))
        else:
            node.get_logger().warn(f"Attribute {attr} isn't found for {radar_bp.id}")    
    
    ## Radar Mounts
    radar_mount = node.get_parameters_by_prefix('mounts.radar_front')
    mount = carla.Transform(
        carla.Location(x=radar_mount['x'].value, y=radar_mount['y'].value, z=radar_mount['z'].value),     
        carla.Rotation(roll=radar_mount['roll'].value, pitch=radar_mount['pitch'].value,yaw=radar_mount['yaw'].value))
    
    ## Sensor attachment & appendign actor to list
    sensor = world.spawn_actor(
        radar_bp, mount, attach_to=vehicle,
        attachment_type=carla.AttachmentType.Rigid)
    
    def _radar_callback(measurement):
        """
        Republish one radar point cloud.
        """
        publisher.publish(
            radar_to_msg(measurement, 'radar_front'))
    
    sensor.listen(_radar_callback)
    actors_list.append(sensor)
    
    ## Logger 
    node.get_logger().info(f"{radar_bp.id} has been initialised...")