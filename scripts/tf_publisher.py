#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster, Buffer, TransformListener
import tf2_geometry_msgs
import numpy as np
from scipy.spatial.transform import Rotation
import threading

class TFFramePublisher:
    def __init__(self):
        rospy.init_node('tf_frame_publisher')
        
        # Create static and dynamic transform broadcasters
        self.tf_static_broadcaster = StaticTransformBroadcaster()
        self.tf_broadcaster = TransformBroadcaster()
        
        # Create TF buffer and listener for transform calculations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)
        
        # Create publisher for body odometry
        self.body_odom_publisher = rospy.Publisher('/body_odometry', Odometry, queue_size=30)
        self.body_pose_publisher = rospy.Publisher('/body_pose', PoseStamped, queue_size=30)
        
        # Create publisher for lidar odometry
        self.lidar_odom_publisher = rospy.Publisher('/lidar_odometry', Odometry, queue_size=30)
        self.lidar_pose_publisher = rospy.Publisher('/lidar_pose', PoseStamped, queue_size=30)
        
        # Get parameters for fusion mode
        self.fusion_mode = rospy.get_param('~fusion_mode', 'direct_odometry')  # 'direct_odometry' or 'body_imu_fusion'
        self.body_odom_topic = rospy.get_param('~body_odom_topic', '/body_odometry')
        
        if self.fusion_mode == 'direct_odometry':
            # Subscribe to odometry to get camera_init -> livox_frame transform
            self.odom_subscription = rospy.Subscriber(
                '/Odometry',
                Odometry,
                self.odom_callback,
                queue_size=30
            )
            rospy.loginfo('TF Frame Publisher started in direct odometry mode')
            
        elif self.fusion_mode == 'body_imu_fusion':
            # Subscribe to body odometry to get camera_init -> body transform
            self.body_odom_subscription = rospy.Subscriber(
                '/Odometry',
                Odometry,
                self.body_odom_callback,
                queue_size=30
            )
            rospy.loginfo('TF Frame Publisher started in body IMU fusion mode, listening to %s', self.body_odom_topic)
        
        # Publish static transforms
        self.publish_static_transforms()
        
        # Create timer to calculate and publish body and lidar odometry
        self.odom_timer = rospy.Timer(rospy.Duration(1.0/30.0), self.calculate_and_publish_odometry)  # 30Hz
        
        # Store previous pose and timestamp for velocity calculation
        self.prev_body_pose = None
        self.prev_body_time = None
        self.prev_lidar_pose = None
        self.prev_lidar_time = None
        
        # Store previous velocity for fallback when pose hasn't changed
        self.prev_body_linear_vel = np.array([0.0, 0.0, 0.0])
        self.prev_body_angular_vel = np.array([0.0, 0.0, 0.0])
        self.prev_lidar_linear_vel = np.array([0.0, 0.0, 0.0])
        self.prev_lidar_angular_vel = np.array([0.0, 0.0, 0.0])
        self.prev_publish_time = None
        
        # Store latest IMU data for velocity calculation
        self.latest_imu = None
        self.imu_lock = threading.Lock()
        
        # Store latest input Odometry message (contains twist.linear)
        self.latest_input_odom = None
        self.input_odom_lock = threading.Lock()
        
        # Subscribe to IMU data
        self.imu_subscription = rospy.Subscriber(
            '/livox/imu',
            Imu,
            self.imu_callback,
            queue_size=100
        )
        rospy.loginfo('Subscribed to /livox/imu for velocity information')
    
    def odom_callback(self, msg):
        """Callback for odometry to publish camera_init -> livox_frame transforms"""
        # Store latest input odometry message (contains twist.linear)
        with self.input_odom_lock:
            self.latest_input_odom = msg
        
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
        # Store latest input odometry message (contains twist.linear)
        with self.input_odom_lock:
            self.latest_input_odom = msg
        
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
    
    def imu_callback(self, msg):
        """Callback for IMU data to store latest angular velocity"""
        with self.imu_lock:
            self.latest_imu = msg
    
    def calculate_and_publish_body_odometry(self):
        """Calculate body odometry by subtracting LiDAR offset from livox_frame position"""
        try:
            # Get transform from map to livox_frame
            transform = self.tf_buffer.lookup_transform(
                'map', 'livox_frame', rospy.Time(), timeout=rospy.Duration(0.1)
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
            
            # Calculate velocity (twist) from input Odometry and IMU
            current_time = rospy.Time.now()  # Use current time for consistent 30Hz publishing
            
            # Get linear velocity from input Odometry (10Hz, more accurate than pose difference)
            with self.input_odom_lock:
                if self.latest_input_odom is not None:
                    # Check if input odom timestamp is recent (within 0.5 seconds for 10Hz, more lenient)
                    input_odom_age = (current_time - self.latest_input_odom.header.stamp).to_sec()
                    
                    # Debug: log input odom twist values (only once per second to avoid spam)
                    if not hasattr(self, '_last_debug_time') or (current_time - self._last_debug_time).to_sec() > 1.0:
                        rospy.loginfo('Input odom twist.linear: x=%.3f, y=%.3f, z=%.3f, age=%.3f',
                                     self.latest_input_odom.twist.twist.linear.x,
                                     self.latest_input_odom.twist.twist.linear.y,
                                     self.latest_input_odom.twist.twist.linear.z,
                                     input_odom_age)
                        self._last_debug_time = current_time
                    
                    if abs(input_odom_age) < 0.5:
                        # Input odom twist.linear is in camera_init frame (child_frame_id of input odom)
                        # camera_init and body have same orientation (only translation offset),
                        # so linear velocity is the same in both frames
                        vel_camera_init = np.array([
                            self.latest_input_odom.twist.twist.linear.x,
                            self.latest_input_odom.twist.twist.linear.y,
                            self.latest_input_odom.twist.twist.linear.z
                        ])
                        
                        # Use input odom velocity directly (it's already in the correct frame)
                        vel_body = vel_camera_init.copy()
                        
                        # Optionally fuse with IMU linear acceleration for smoothing
                        with self.imu_lock:
                            if self.latest_imu is not None and self.prev_publish_time is not None:
                                dt = (current_time - self.prev_publish_time).to_sec()
                                if dt > 1e-6 and dt < 1.0:
                                    # Get IMU linear acceleration in livox_frame (same as body frame orientation)
                                    imu_accel = np.array([
                                        self.latest_imu.linear_acceleration.x,
                                        self.latest_imu.linear_acceleration.y,
                                        self.latest_imu.linear_acceleration.z
                                    ])
                                    
                                    # Integrate acceleration to get velocity change
                                    # vel_new = vel_old + accel * dt
                                    vel_from_accel = self.prev_body_linear_vel + imu_accel * dt
                                    
                                    # Simple fusion: weighted average (you can adjust weights)
                                    # Higher weight for input odom (more accurate), lower for IMU integration
                                    weight_odom = 0.8
                                    weight_imu = 0.2
                                    vel_body = weight_odom * vel_body + weight_imu * vel_from_accel
                        
                        body_odom.twist.twist.linear.x = vel_body[0]
                        body_odom.twist.twist.linear.y = vel_body[1]
                        body_odom.twist.twist.linear.z = vel_body[2]
                        self.prev_body_linear_vel = vel_body
                    else:
                        # Input odom too old, use previous velocity
                        body_odom.twist.twist.linear.x = self.prev_body_linear_vel[0]
                        body_odom.twist.twist.linear.y = self.prev_body_linear_vel[1]
                        body_odom.twist.twist.linear.z = self.prev_body_linear_vel[2]
                else:
                    # No input odom available, use previous velocity
                    body_odom.twist.twist.linear.x = self.prev_body_linear_vel[0]
                    body_odom.twist.twist.linear.y = self.prev_body_linear_vel[1]
                    body_odom.twist.twist.linear.z = self.prev_body_linear_vel[2]
            
            # Use IMU angular velocity (body and livox_frame have same orientation, so angular velocity is the same)
            # Always use latest IMU data if available (IMU is high frequency, so it's always recent)
            with self.imu_lock:
                if self.latest_imu is not None:
                    # Use IMU angular velocity directly (already in livox_frame/body frame)
                    # IMU is high frequency (>30Hz), so data is always recent
                    body_odom.twist.twist.angular.x = self.latest_imu.angular_velocity.x
                    body_odom.twist.twist.angular.y = self.latest_imu.angular_velocity.y
                    body_odom.twist.twist.angular.z = self.latest_imu.angular_velocity.z
                    # Update previous angular velocity
                    self.prev_body_angular_vel = np.array([
                        self.latest_imu.angular_velocity.x,
                        self.latest_imu.angular_velocity.y,
                        self.latest_imu.angular_velocity.z
                    ])
                else:
                    # No IMU data available, use previous angular velocity
                    body_odom.twist.twist.angular.x = self.prev_body_angular_vel[0]
                    body_odom.twist.twist.angular.y = self.prev_body_angular_vel[1]
                    body_odom.twist.twist.angular.z = self.prev_body_angular_vel[2]
            
            # Set twist covariance
            body_odom.twist.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
            
            # Update previous pose and time
            self.prev_body_pose = body_odom.pose.pose
            self.prev_body_time = transform.header.stamp  # Keep original transform timestamp for pose tracking
            self.prev_publish_time = current_time  # Use current time for velocity calculation
            
            # Publish body odometry
            self.body_odom_publisher.publish(body_odom)
            
            # Also publish as PoseStamped for convenience
            body_pose = PoseStamped()
            body_pose.header = body_odom.header
            body_pose.pose = body_odom.pose.pose
            self.body_pose_publisher.publish(body_pose)
            
        except Exception as e:
            rospy.logdebug('Could not calculate body odometry: %s', str(e))
    
    def calculate_and_publish_lidar_odometry(self):
        """Calculate lidar (livox_frame) odometry relative to map and publish it"""
        try:
            # Get transform from map to livox_frame
            transform = self.tf_buffer.lookup_transform(
                'map', 'livox_frame', rospy.Time(), timeout=rospy.Duration(0.1)
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
            
            # Calculate velocity (twist) from input Odometry and IMU
            current_time = rospy.Time.now()  # Use current time for consistent 30Hz publishing
            
            # Get linear velocity from input Odometry (10Hz, more accurate than pose difference)
            with self.input_odom_lock:
                if self.latest_input_odom is not None:
                    # Check if input odom timestamp is recent (within 0.2 seconds for 10Hz)
                    input_odom_age = (current_time - self.latest_input_odom.header.stamp).to_sec()
                    if abs(input_odom_age) < 0.2 and hasattr(self.latest_input_odom, 'twist'):
                        # Input odom has twist.linear in camera_init frame
                        # livox_frame and camera_init have same orientation, so velocity is the same
                        vel_lidar = np.array([
                            self.latest_input_odom.twist.twist.linear.x,
                            self.latest_input_odom.twist.twist.linear.y,
                            self.latest_input_odom.twist.twist.linear.z
                        ])
                        
                        # Optionally fuse with IMU linear acceleration for smoothing
                        with self.imu_lock:
                            if self.latest_imu is not None and self.prev_publish_time is not None:
                                dt = (current_time - self.prev_publish_time).to_sec()
                                if dt > 1e-6 and dt < 1.0:
                                    # Get IMU linear acceleration in livox_frame
                                    imu_accel = np.array([
                                        self.latest_imu.linear_acceleration.x,
                                        self.latest_imu.linear_acceleration.y,
                                        self.latest_imu.linear_acceleration.z
                                    ])
                                    
                                    # Integrate acceleration to get velocity change
                                    vel_from_accel = self.prev_lidar_linear_vel + imu_accel * dt
                                    
                                    # Simple fusion: weighted average
                                    weight_odom = 0.8
                                    weight_imu = 0.2
                                    vel_lidar = weight_odom * vel_lidar + weight_imu * vel_from_accel
                        
                        lidar_odom.twist.twist.linear.x = vel_lidar[0]
                        lidar_odom.twist.twist.linear.y = vel_lidar[1]
                        lidar_odom.twist.twist.linear.z = vel_lidar[2]
                        self.prev_lidar_linear_vel = vel_lidar
                    else:
                        # Input odom too old or no twist, use previous velocity
                        lidar_odom.twist.twist.linear.x = self.prev_lidar_linear_vel[0]
                        lidar_odom.twist.twist.linear.y = self.prev_lidar_linear_vel[1]
                        lidar_odom.twist.twist.linear.z = self.prev_lidar_linear_vel[2]
                else:
                    # No input odom available, use previous velocity
                    lidar_odom.twist.twist.linear.x = self.prev_lidar_linear_vel[0]
                    lidar_odom.twist.twist.linear.y = self.prev_lidar_linear_vel[1]
                    lidar_odom.twist.twist.linear.z = self.prev_lidar_linear_vel[2]
            
            # Use IMU angular velocity directly (IMU is in livox_frame)
            # Always use latest IMU data if available (IMU is high frequency, so it's always recent)
            with self.imu_lock:
                if self.latest_imu is not None:
                    # Use IMU angular velocity directly (already in livox_frame)
                    # IMU is high frequency (>30Hz), so data is always recent
                    lidar_odom.twist.twist.angular.x = self.latest_imu.angular_velocity.x
                    lidar_odom.twist.twist.angular.y = self.latest_imu.angular_velocity.y
                    lidar_odom.twist.twist.angular.z = self.latest_imu.angular_velocity.z
                    # Update previous angular velocity
                    self.prev_lidar_angular_vel = np.array([
                        self.latest_imu.angular_velocity.x,
                        self.latest_imu.angular_velocity.y,
                        self.latest_imu.angular_velocity.z
                    ])
                else:
                    # No IMU data available, use previous angular velocity
                    lidar_odom.twist.twist.angular.x = self.prev_lidar_angular_vel[0]
                    lidar_odom.twist.twist.angular.y = self.prev_lidar_angular_vel[1]
                    lidar_odom.twist.twist.angular.z = self.prev_lidar_angular_vel[2]
            
            # Set twist covariance
            lidar_odom.twist.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                                         0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                                         0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                                         0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                                         0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                                         0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
            
            # Update previous pose and time
            self.prev_lidar_pose = lidar_odom.pose.pose
            self.prev_lidar_time = transform.header.stamp  # Keep original transform timestamp for pose tracking
            
            # Publish lidar odometry
            self.lidar_odom_publisher.publish(lidar_odom)
            
            # Also publish as PoseStamped for convenience
            lidar_pose = PoseStamped()
            lidar_pose.header = lidar_odom.header
            lidar_pose.pose = lidar_odom.pose.pose
            self.lidar_pose_publisher.publish(lidar_pose)
            
        except Exception as e:
            rospy.logdebug('Could not calculate lidar odometry: %s', str(e))
    
    def calculate_and_publish_odometry(self, event):
        """Calculate and publish both body and lidar odometry"""
        self.calculate_and_publish_body_odometry()
        self.calculate_and_publish_lidar_odometry()
    
    def publish_static_transforms(self):
        """Publish static transforms based on fusion mode"""
        # Always publish livox_frame -> body transform (LiDAR offset)
        t_livox_to_body = TransformStamped()
        t_livox_to_body.header.stamp = rospy.Time(0)  # Use Time(0) for static transforms
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
        t_map_to_camera_init.header.stamp = rospy.Time(0)  # Use Time(0) for static transforms
        t_map_to_camera_init.header.frame_id = 'map'
        t_map_to_camera_init.child_frame_id = 'camera_init'
        
        # Camera_init is a fixed reference frame at origin
        t_map_to_camera_init.transform.translation.x = 0.1710
        t_map_to_camera_init.transform.translation.y = 0.0
        t_map_to_camera_init.transform.translation.z = 0.43
        
        # Camera_init has fixed orientation (identity)
        t_map_to_camera_init.transform.rotation.x = 0.0
        t_map_to_camera_init.transform.rotation.y = 0.0
        t_map_to_camera_init.transform.rotation.z = 0.0
        t_map_to_camera_init.transform.rotation.w = 1.0
        
        # Publish static transforms
        self.tf_static_broadcaster.sendTransform(t_livox_to_body)
        self.tf_static_broadcaster.sendTransform(t_map_to_camera_init)
        rospy.loginfo('Published static transforms: livox_frame->body, map->camera_init')

def main():
    tf_publisher = TFFramePublisher()
    
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
