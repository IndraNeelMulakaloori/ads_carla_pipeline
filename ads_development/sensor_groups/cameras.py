import carla

from l2_carla_demo.conversions import (
    image_to_msg,
)

# cameras.py - Camera Sensor Group Module
#
# Helper script containing modular setup functions and ROS 2 callbacks for the 
# RGB, Depth, and Semantic Segmentation cameras. Handles spawning, attaching 
# sensors to the ego vehicle, and publishing the image data streams.


def _attach_rgb_camera(node, world, blueprints, vehicle, actors_list, msg_type, publisher,mount_name='camera_rgb_front'):
        """
        Attach RGB camera sensor attributes and mount values 
        """
        ## RGB Camera Sensor attributes
        rgb_bp = blueprints.find('sensor.camera.rgb')
        rgb_attributes = node.get_parameters_by_prefix('camera')


        rgb_bp.set_attribute('image_size_x', str(rgb_attributes['width'].value))
        rgb_bp.set_attribute('image_size_y', str(rgb_attributes['height'].value))
        rgb_bp.set_attribute('fov', str(rgb_attributes['fov'].value))
        rgb_bp.set_attribute('sensor_tick', str(rgb_attributes['sensor_tick'].value))

        # ? Can we change carla_config.yaml from width, hiegh to image_size_x,y
        # for attr, value in rgb_attributes.items():
        #     if rgb_bp.has_attribute(attr):
        #         rgb_bp.set_attribute(attr,value)
        #     else :
        #         self.get_logger().warn(f"Attribute {attr} isn't found for {rgb_bp.id}")


        ## RGB Camera Mounts
        rgb_mount = node.get_parameters_by_prefix(f'mounts.{mount_name}')
        mount = carla.Transform(
            carla.Location(x=rgb_mount['x'].value,y= rgb_mount['y'].value, z=rgb_mount['z'].value),
            carla.Rotation(roll=rgb_mount['roll'].value, pitch=rgb_mount['pitch'].value,yaw=rgb_mount['yaw'].value))

        ## Sensor attachment & appendign actor to list
        sensor = world.spawn_actor(
            rgb_bp, mount, attach_to=vehicle,
            attachment_type=carla.AttachmentType.Rigid)
        
        ## Callback
        def _rgb_callback(measurement):
                """Republish one camera frame.
        
                Note what image_to_msg does NOT do: it never touches the wall clock.
                The timestamp comes off the measurement, because that records when
                the data was captured rather than when Python got round to the
                callback. Task 3's timing analysis is meaningless otherwise.
                """
                publisher.publish(
                    image_to_msg(measurement, mount_name, msg_type))
                
        sensor.listen(_rgb_callback)
        actors_list.append(sensor)
        
        ## Logger 
        node.get_logger().info(f"{rgb_bp.id} has been initialised..")

def _attach_depth_camera(node, world, blueprints, vehicle, actors_list,publisher, msg_type):
    """
    Attach depth camera sensor attributes and mount values 
    """
    ## depth Camera Sensor attributes
    depth_bp = blueprints.find('sensor.camera.depth')
    depth_attributes = node.get_parameters_by_prefix('depth_camera')
    depth_bp.set_attribute('image_size_x', str(depth_attributes['width'].value))
    depth_bp.set_attribute('image_size_y', str(depth_attributes['height'].value))
    depth_bp.set_attribute('fov', str(depth_attributes['fov'].value))
    depth_bp.set_attribute('sensor_tick', str(depth_attributes['sensor_tick'].value))
    
    # ? Can we change carla_config.yaml from width, hiegh to image_size_x,y
    # for attr, value in rgb_attributes.items():
    #     if rgb_bp.has_attribute(attr):
    #         rgb_bp.set_attribute(attr,value)
    #     else :
    #         self.get_logger().warn(f"Attribute {attr} isn't found for {rgb_bp.id}")
    
    
    ## Depth Camera Mounts
    depth_mount = node.get_parameters_by_prefix('mounts.camera_depth_front')
    mount = carla.Transform(
        carla.Location(x=depth_mount['x'].value, y= depth_mount['y'].value, z=depth_mount['z'].value),     
        carla.Rotation(roll=depth_mount['roll'].value, pitch=depth_mount['pitch'].value,yaw=depth_mount['yaw'].value))


    
    ## Sensor attachment & appendign actor to list
    sensor = world.spawn_actor(
        depth_bp, mount, attach_to=vehicle,
        attachment_type=carla.AttachmentType.Rigid)
    #Callback 
    def _depth_callback(measurement):
        """
        Republish one depth camera frame.
        """
        publisher.publish(
            image_to_msg(measurement, 'camera_depth_front', msg_type))

    sensor.listen(_depth_callback)
    actors_list.append(sensor)
    
    ## Logger 
    node.get_logger().info(f"{depth_bp.id} has been initialised...")



def _attach_semseg_camera(node, world, blueprints, vehicle, actors_list,publisher, msg_type):
    """
    Attach depth camera sensor attributes and mount values 
    """
    ## depth Camera Sensor attributes
    semseg_bp = blueprints.find('sensor.camera.semantic_segmentation')
    semseg_attributes = node.get_parameters_by_prefix('semseg_camera')
    semseg_bp.set_attribute('image_size_x', str(semseg_attributes['width'].value))
    semseg_bp.set_attribute('image_size_y', str(semseg_attributes['height'].value))
    semseg_bp.set_attribute('fov', str(semseg_attributes['fov'].value))
    semseg_bp.set_attribute('sensor_tick', str(semseg_attributes['sensor_tick'].value))
    
    # ? Can we change carla_config.yaml from width, hiegh to image_size_x,y
    # for attr, value in rgb_attributes.items():
    #     if rgb_bp.has_attribute(attr):
    #         rgb_bp.set_attribute(attr,value)
    #     else :
    #         self.get_logger().warn(f"Attribute {attr} isn't found for {rgb_bp.id}")
    
    
    ## Depth Camera Mounts
    semseg_mount = node.get_parameters_by_prefix('mounts.camera_semseg_front')
    mount = carla.Transform(
        carla.Location(x=semseg_mount['x'].value, y= semseg_mount['y'].value, z=semseg_mount['z'].value),     
        carla.Rotation(roll=semseg_mount['roll'].value, pitch=semseg_mount['pitch'].value,yaw=semseg_mount['yaw'].value))
    
    ## Sensor attachment & appendign actor to list
    sensor = world.spawn_actor(
        semseg_bp, mount, attach_to=vehicle,
        attachment_type=carla.AttachmentType.Rigid)
    
    ## Callback
    def _semseg_callback(measurement):
        """
        Republish one semantic segmenation camera frame.
        """
        #         The semantic camera's raw image holds the class tag in the
        #         red channel and looks black in RViz. Call
        #         measurement.convert(carla.ColorConverter.CityScapesPalette)
        #         before image_to_msg to publish it in colour.
        
        measurement.convert(carla.ColorConverter.CityScapesPalette)
        publisher.publish(
            image_to_msg(measurement, 'camera_semseg_front', msg_type))
        
    sensor.listen(_semseg_callback)
    actors_list.append(sensor)
    
    ## Logger 
    node.get_logger().info(f"{semseg_bp.id} has been initialised...")

        
        