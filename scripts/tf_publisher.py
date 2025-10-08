#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Odometry
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster, Buffer, TransformListener
import tf2_geometry_msgs
import numpy as np
from scipy.spatial.transform import Rotation

class TFFramePublisher(Node):
    def __init__(self):
        super().__init__('tf_frame_publisher')
        
        # Create static and dynamic transform broadcasters
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Create TF buffer and listener for transform calculations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create publisher for body odometry
        self.body_odom_publisher = self.create_publisher(Odometry, '/body_odometry', 30)
        self.body_pose_publisher = self.create_publisher(PoseStamped, '/body_pose', 30)
        
        # Create publisher for lidar odometry
        self.lidar_odom_publisher = self.create_publisher(Odometry, '/lidar_odometry', 30)
        self.lidar_pose_publisher = self.create_publisher(PoseStamped, '/lidar_pose', 30)
        
        # Declare parameters for fusion mode
        self.declare_parameter('fusion_mode', 'direct_odometry')  # 'direct_odometry' or 'body_imu_fusion'
        self.declare_parameter('body_odom_topic', '/body_odometry')
        
        # Get fusion mode parameter
        self.fusion_mode = self.get_parameter('fusion_mode').get_parameter_value().string_value
        
        if self.fusion_mode == 'direct_odometry':
            # Subscribe to odometry to get camera_init -> livox_frame transform
            self.odom_subscription = self.create_subscription(
                Odometry,
                '/Odometry',
                self.odom_callback,
                30
            )
            self.get_logger().info('TF Frame Publisher started in direct odometry mode')
            
        elif self.fusion_mode == 'body_imu_fusion':
            # Subscribe to body odometry to get camera_init -> body transform
            body_odom_topic = self.get_parameter('body_odom_topic').get_parameter_value().string_value
            self.body_odom_subscription = self.create_subscription(
                Odometry,
                '/Odometry',
                self.body_odom_callback,
                30
            )
            self.get_logger().info(f'TF Frame Publisher started in body IMU fusion mode, listening to {body_odom_topic}')
        
        # Publish static transforms
        self.publish_static_transforms()
        
        # Create timer to calculate and publish body and lidar odometry
        self.odom_timer = self.create_timer(1.0/30.0, self.calculate_and_publish_odometry)  # 30Hz
    
    def odom_callback(self, msg):
        """Callback for odometry to publish camera_init -> livox_frame transforms"""
        # Create transform from camera_init to livox_frame using LIO odometry directly
        t_camera_init_to_livox = TransformStamped()
        t_camera_init_to_livox.header.stamp = msg.header.stamp
        t_camera_init_to_livox.header.frame_id = 'camera_init'
        t_camera_init_to_livox.child_frame_id = 'livox_frame'
        
        # Use LIO odometry directly (no additional offset)
        t_camera_init_to_livox.transform.translation.x = msg.pose.pose.position.x
        t_camera_init_to_livox.transform.translation.y = msg.pose.pose.position.y
        t_camera_init_to_livox.transform.translation.z = msg.pose.pose.position.z
        
        t_camera_init_to_livox.transform.rotation.x = msg.pose.pose.orientation.x
        t_camera_init_to_livox.transform.rotation.y = msg.pose.pose.orientation.y
        t_camera_init_to_livox.transform.rotation.z = msg.pose.pose.orientation.z
        t_camera_init_to_livox.transform.rotation.w = msg.pose.pose.orientation.w
        
        # Publish transform (map->camera_init is now static)
        self.tf_broadcaster.sendTransform(t_camera_init_to_livox)
    
    def body_odom_callback(self, msg):
        """Callback for body odometry to publish camera_init -> livox_frame transforms"""
        # Create transform from camera_init to livox_frame based on odometry
        t_camera_init_to_livox = TransformStamped()
        t_camera_init_to_livox.header.stamp = msg.header.stamp
        t_camera_init_to_livox.header.frame_id = 'camera_init'
        t_camera_init_to_livox.child_frame_id = 'livox_frame'
        
        # Use the pose from odometry as the transform
        t_camera_init_to_livox.transform.translation.x = msg.pose.pose.position.x
        t_camera_init_to_livox.transform.translation.y = msg.pose.pose.position.y
        t_camera_init_to_livox.transform.translation.z = msg.pose.pose.position.z
        
        t_camera_init_to_livox.transform.rotation.x = msg.pose.pose.orientation.x
        t_camera_init_to_livox.transform.rotation.y = msg.pose.pose.orientation.y
        t_camera_init_to_livox.transform.rotation.z = msg.pose.pose.orientation.z
        t_camera_init_to_livox.transform.rotation.w = msg.pose.pose.orientation.w
        
        # Publish transform (map->camera_init is now static)
        self.tf_broadcaster.sendTransform(t_camera_init_to_livox)
    
    def calculate_and_publish_body_odometry(self):
        """Calculate body odometry by subtracting LiDAR offset from livox_frame position"""
        try:
            # Get transform from map to livox_frame
            transform = self.tf_buffer.lookup_transform(
                'map', 'livox_frame', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            # Create body odometry message
            body_odom = Odometry()
            body_odom.header.stamp = transform.header.stamp
            body_odom.header.frame_id = 'map'
            body_odom.child_frame_id = 'body'
            
            # LiDAR offset: (0.1710, 0, 0.0968) relative to body
            lidar_offset_x = 0.1710
            lidar_offset_y = 0.0
            lidar_offset_z = 0.0968  # 0.0908 for XT-16
            
            # Get rotation matrix from transform quaternion to rotate the offset
            quat = [transform.transform.rotation.x, 
                   transform.transform.rotation.y,
                   transform.transform.rotation.z,
                   transform.transform.rotation.w]
            rotation = Rotation.from_quat(quat)
            rotation_matrix = rotation.as_matrix()
            
            # Transform LiDAR offset from body frame to map frame
            lidar_offset_in_body = np.array([lidar_offset_x, lidar_offset_y, lidar_offset_z])
            lidar_offset_in_map = rotation_matrix @ lidar_offset_in_body
            
            # Subtract rotated LiDAR offset from livox_frame position to get body position
            body_odom.pose.pose.position.x = transform.transform.translation.x - lidar_offset_in_map[0]
            body_odom.pose.pose.position.y = transform.transform.translation.y - lidar_offset_in_map[1]
            body_odom.pose.pose.position.z = transform.transform.translation.z - lidar_offset_in_map[2]
            
            body_odom.pose.pose.orientation.x = transform.transform.rotation.x
            body_odom.pose.pose.orientation.y = transform.transform.rotation.y
            body_odom.pose.pose.orientation.z = transform.transform.rotation.z
            body_odom.pose.pose.orientation.w = transform.transform.rotation.w
            
            # Set covariance (you may want to adjust these values based on your system)
            body_odom.pose.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                                       0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
            
            # Publish body odometry
            self.body_odom_publisher.publish(body_odom)
            
            # Also publish as PoseStamped for convenience
            body_pose = PoseStamped()
            body_pose.header = body_odom.header
            body_pose.pose = body_odom.pose.pose
            self.body_pose_publisher.publish(body_pose)
            
        except Exception as e:
            self.get_logger().debug(f'Could not calculate body odometry: {str(e)}')
    
    def calculate_and_publish_lidar_odometry(self):
        """Calculate lidar (livox_frame) odometry relative to map and publish it"""
        try:
            # Get transform from map to livox_frame
            transform = self.tf_buffer.lookup_transform(
                'map', 'livox_frame', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            # Create lidar odometry message
            lidar_odom = Odometry()
            lidar_odom.header.stamp = transform.header.stamp
            lidar_odom.header.frame_id = 'map'
            lidar_odom.child_frame_id = 'livox_frame'
            
            # Set pose from transform
            lidar_odom.pose.pose.position.x = transform.transform.translation.x
            lidar_odom.pose.pose.position.y = transform.transform.translation.y
            lidar_odom.pose.pose.position.z = transform.transform.translation.z
            
            lidar_odom.pose.pose.orientation.x = transform.transform.rotation.x
            lidar_odom.pose.pose.orientation.y = transform.transform.rotation.y
            lidar_odom.pose.pose.orientation.z = transform.transform.rotation.z
            lidar_odom.pose.pose.orientation.w = transform.transform.rotation.w
            
            # Set covariance (you may want to adjust these values based on your system)
            lidar_odom.pose.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
            
            # Publish lidar odometry
            self.lidar_odom_publisher.publish(lidar_odom)
            
            # Also publish as PoseStamped for convenience
            lidar_pose = PoseStamped()
            lidar_pose.header = lidar_odom.header
            lidar_pose.pose = lidar_odom.pose.pose
            self.lidar_pose_publisher.publish(lidar_pose)
            
        except Exception as e:
            self.get_logger().debug(f'Could not calculate lidar odometry: {str(e)}')
    
    def calculate_and_publish_odometry(self):
        """Calculate and publish both body and lidar odometry"""
        self.calculate_and_publish_body_odometry()
        self.calculate_and_publish_lidar_odometry()
    
    def publish_static_transforms(self):
        """Publish static transforms based on fusion mode"""
        # Always publish livox_frame -> body transform (LiDAR offset)
        t_livox_to_body = TransformStamped()
        t_livox_to_body.header.stamp = rclpy.time.Time(seconds=0).to_msg()  # Use Time(0) for static transforms
        t_livox_to_body.header.frame_id = 'livox_frame'
        t_livox_to_body.child_frame_id = 'body'
        
        # LiDAR offset: (0.1710, 0, 0.0968) relative to body
        # So livox_frame to body offset is negative
        t_livox_to_body.transform.translation.x = -0.1710
        t_livox_to_body.transform.translation.y = -0.0
        t_livox_to_body.transform.translation.z = -0.0968  # 0.0908 for XT-16
        
        # No rotation (identity quaternion)
        t_livox_to_body.transform.rotation.x = 0.0
        t_livox_to_body.transform.rotation.y = 0.0
        t_livox_to_body.transform.rotation.z = 0.0
        t_livox_to_body.transform.rotation.w = 1.0
        
        # Always publish map -> camera_init transform (fixed reference frame)
        t_map_to_camera_init = TransformStamped()
        t_map_to_camera_init.header.stamp = rclpy.time.Time(seconds=0).to_msg()  # Use Time(0) for static transforms
        t_map_to_camera_init.header.frame_id = 'map'
        t_map_to_camera_init.child_frame_id = 'camera_init'
        
        # Camera_init is a fixed reference frame at origin
        t_map_to_camera_init.transform.translation.x = 0.1710
        t_map_to_camera_init.transform.translation.y = 0.0
        t_map_to_camera_init.transform.translation.z = 0.0968
        
        # Camera_init has fixed orientation (identity)
        t_map_to_camera_init.transform.rotation.x = 0.0
        t_map_to_camera_init.transform.rotation.y = 0.0
        t_map_to_camera_init.transform.rotation.z = 0.0
        t_map_to_camera_init.transform.rotation.w = 1.0
        
        # Publish static transforms
        self.tf_static_broadcaster.sendTransform(t_livox_to_body)
        self.tf_static_broadcaster.sendTransform(t_map_to_camera_init)
        self.get_logger().info('Published static transforms: livox_frame->body, map->camera_init')

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
