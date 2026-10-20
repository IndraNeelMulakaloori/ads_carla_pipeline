import carla

from l2_carla_demo.conversions import (
    gnss_to_msg,
    imu_to_msg,
)

# imu_gnss.py - IMU and GNSS Sensor Group Module
#
# Helper script containing modular setup functions and ROS 2 callbacks  
# for IMU and GNSS sensors.
def _attach_imu(node, world, blueprints, vehicle, actors_list, publisher):
    """
    Attach imu sensor attributes and mount values 
    """
    ## Imu Sensor attributes
    imu_bp = blueprints.find('sensor.other.imu')
    imu_attributes = node.get_parameters_by_prefix('imu')
    for attr, value in imu_attributes.items():
        if imu_bp.has_attribute(attr):
            imu_bp.set_attribute(attr,str(value.value))
        else:
            node.get_logger().warn(f"Attribute {attr} isn't found for {imu_bp.id}")    
    
    ## Imu Mounts
    imu_mount = node.get_parameters_by_prefix('mounts.imu_link')
    mount = carla.Transform(
        carla.Location(x=imu_mount['x'].value, y=imu_mount['y'].value, z=imu_mount['z'].value),     
        carla.Rotation(roll=imu_mount['roll'].value, pitch=imu_mount['pitch'].value,yaw=imu_mount['yaw'].value))
    
    ## Sensor attachment & appendign actor to list
    sensor = world.spawn_actor(
        imu_bp, mount, attach_to=vehicle,
        attachment_type=carla.AttachmentType.Rigid)
    
    def _imu_callback(measurement):
        """
        Republish imu data.
        """
        publisher.publish(
            imu_to_msg(measurement, 'imu_link'))
        
    sensor.listen(_imu_callback)  
    actors_list.append(sensor) 
    
    ## Logger 
    node.get_logger().info(f"{imu_bp.id} has been initialised...")

def _attach_gnss(node, world, blueprints, vehicle, actors_list, publisher):
    """
    Attach gnss sensor attributes and mount values 
    """
    ## GNSS Sensor attributes
    gnss_bp = blueprints.find('sensor.other.gnss')
    gnss_attributes = node.get_parameters_by_prefix('gnss')
    for attr, value in gnss_attributes.items():
        if gnss_bp.has_attribute(attr):
            gnss_bp.set_attribute(attr,str(value.value))
        else:
            node.get_logger().warn(f"Attribute {attr} isn't found for {gnss_bp.id}")    
    
    ## GNSS Mounts
    gnss_mount = node.get_parameters_by_prefix('mounts.gnss_link')
    mount = carla.Transform(
        carla.Location(x=gnss_mount['x'].value, y=gnss_mount['y'].value, z=gnss_mount['z'].value),     
        carla.Rotation(roll=gnss_mount['roll'].value, pitch=gnss_mount['pitch'].value,yaw=gnss_mount['yaw'].value))
    
    ## Sensor attachment & appendign actor to list
    sensor = world.spawn_actor(
        gnss_bp, mount, attach_to=vehicle,
        attachment_type=carla.AttachmentType.Rigid)
    
    def _gnss_callback(measurement):
        """
        Republish gnss data.
        """
        publisher.publish(
            gnss_to_msg(measurement, 'gnss_link'))
        
    sensor.listen(_gnss_callback)  
    actors_list.append(sensor) 
    
    ## Logger 
    node.get_logger().info(f"{gnss_bp.id} has been initialised...")