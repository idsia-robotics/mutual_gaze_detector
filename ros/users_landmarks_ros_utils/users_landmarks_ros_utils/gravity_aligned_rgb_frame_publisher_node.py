#!/usr/bin/env python3

import rclpy.node
from rclpy.time import Time
from ament_index_python.packages import get_package_share_directory

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler, quaternion_multiply

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu

import numpy as np
import time

class GravityAlignedRgbFramePublisherNode(rclpy.node.Node):

    def __init__(self):
        super().__init__("gravity_aligned_rgb_frame_publisher_node")
        self.declare_parameters(
            namespace='',
            parameters=[
                ('debug_node', False)
            ])
        
        # Parameters
        self.debug_node = self.get_parameter('debug_node').get_parameter_value().bool_value
        self.get_logger().info("debug_node: {}".format(self.debug_node))
        
        # Subscribers
        self.imu_sub = self.create_subscription(Imu, "/imu", self.imu_callback, 1)

        # Transform listeners
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Transform broadcasters
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Arguments of node
        self.depth_to_rgb_static_tf_obtained = False
                
        self.get_logger().info("Node correctly initialized.")
                
    def imu_callback(self, msg):      
        if not self.depth_to_rgb_static_tf_obtained:
            to_frame_rel = "depth_camera_link"
            from_frame_rel = "rgb_camera_link"
            
            try:
                self.depth_to_rgb_static_tf = self.tf_buffer.lookup_transform(to_frame_rel, from_frame_rel, rclpy.time.Time())
            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
                return
            
            self.depth_to_rgb_static_tf_obtained = True
                 
        linear_acceleration = msg.linear_acceleration
        
        pitch = np.arctan2(linear_acceleration.x, np.sqrt(np.power(linear_acceleration.y,2) + np.power(linear_acceleration.z,2)))
        roll =  np.arctan2(-linear_acceleration.y, np.sqrt(np.power(linear_acceleration.x,2) + np.power(linear_acceleration.z,2)))
               
        # self.get_logger().info(f"Pitch: {pitch*(180/np.pi)}, Roll: {roll*(180/np.pi)}")
        
        t = TransformStamped()

        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'depth_camera_link'
        t.child_frame_id = 'gravity_aligned_rgb_camera_link'

        t.transform.translation.x = self.depth_to_rgb_static_tf.transform.translation.x
        t.transform.translation.y = self.depth_to_rgb_static_tf.transform.translation.y
        t.transform.translation.z = self.depth_to_rgb_static_tf.transform.translation.z
        
        rgb_to_gravity_aligned_quaternion = quaternion_from_euler(pitch, 0, roll)
        depth_to_rgb_quaternion = [self.depth_to_rgb_static_tf.transform.rotation.x,
                                   self.depth_to_rgb_static_tf.transform.rotation.y,
                                   self.depth_to_rgb_static_tf.transform.rotation.z,
                                   self.depth_to_rgb_static_tf.transform.rotation.w]
        depth_to_gravity_aligned_quaternion = quaternion_multiply(depth_to_rgb_quaternion, rgb_to_gravity_aligned_quaternion)
        
        t.transform.rotation.x = depth_to_gravity_aligned_quaternion[0]
        t.transform.rotation.y = depth_to_gravity_aligned_quaternion[1]
        t.transform.rotation.z = depth_to_gravity_aligned_quaternion[2]
        t.transform.rotation.w = depth_to_gravity_aligned_quaternion[3]

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)
                        
def main(args=None):
    rclpy.init(args=args)
    node = GravityAlignedRgbFramePublisherNode()
    
    rclpy.spin(node)
    
    # On shutdown
    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()