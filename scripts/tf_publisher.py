#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

class TFFramePublisher(Node):
    def __init__(self):
        super().__init__('tf_frame_publisher')
        
        # Create static and dynamic transform broadcasters
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribe to odometry to get camera_init -> livox_frame transform
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/Odometry',
            self.odom_callback,
            10
        )
        
        # Publish static transforms
        self.publish_static_transforms()
        
        self.get_logger().info('TF Frame Publisher started')
    
    def odom_callback(self, msg):
        """Callback for odometry to publish camera_init -> livox_frame transform"""
        # Create transform from camera_init to livox_frame based on odometry
        t_camera_init_to_livox = TransformStamped()
        t_camera_init_to_livox.header.stamp = msg.header.stamp
        t_camera_init_to_livox.header.frame_id = 'camera_init'
        t_camera_init_to_livox.child_frame_id = 'livox_frame'
        
        # Use the pose from odometry as the transform
        t_camera_init_to_livox.transform.translation.x = msg.pose.pose.position.x
        t_camera_init_to_livox.transform.translation.y = msg.pose.pose.position.y
        t_camera_init_to_livox.transform.translation.z = msg.pose.pose.position.z
        
        t_camera_init_to_livox.transform.rotation.x = -msg.pose.pose.orientation.x
        t_camera_init_to_livox.transform.rotation.y = -msg.pose.pose.orientation.y
        t_camera_init_to_livox.transform.rotation.z = -msg.pose.pose.orientation.z
        t_camera_init_to_livox.transform.rotation.w = msg.pose.pose.orientation.w
        
        # Publish the dynamic transform
        self.tf_broadcaster.sendTransform(t_camera_init_to_livox)
    
    def publish_static_transforms(self):
        """Publish static transforms between map and camera_init"""
        
        # Transform: map -> camera_init (identity transform since camera_init is LIO's world frame)
        t_map_to_camera_init = TransformStamped()
        t_map_to_camera_init.header.stamp = rclpy.time.Time(seconds=0).to_msg()  # Use Time(0) for static transforms
        t_map_to_camera_init.header.frame_id = 'map'
        t_map_to_camera_init.child_frame_id = 'camera_init'
        
        # Identity transform (no translation, no rotation)
        t_map_to_camera_init.transform.translation.x = 0.0
        t_map_to_camera_init.transform.translation.y = 0.0
        t_map_to_camera_init.transform.translation.z = 0.0
        
        t_map_to_camera_init.transform.rotation.x = 0.0
        t_map_to_camera_init.transform.rotation.y = 0.0
        t_map_to_camera_init.transform.rotation.z = 0.0
        t_map_to_camera_init.transform.rotation.w = 1.0
        
        # Publish static transform
        self.tf_static_broadcaster.sendTransform(t_map_to_camera_init)
        
        self.get_logger().info('Published static transform: map->camera_init')

def main(args=None):
    rclpy.init(args=args)
    
    tf_publisher = TFFramePublisher()
    
    try:
        rclpy.spin(tf_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        tf_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
