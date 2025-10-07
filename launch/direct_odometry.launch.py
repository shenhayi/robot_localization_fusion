#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the launch directory
    pkg_share = get_package_share_directory('robot_localization_fusion')
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_share, 'config', 'ekf_fusion.yaml'),
        description='Full path to the EKF configuration file'
    )
    
    body_odom_topic_arg = DeclareLaunchArgument(
        'body_odom_topic',
        default_value='/body_odometry',
        description='Body odometry topic (for future use)'
    )
    
    # Get launch configurations
    config_file = LaunchConfiguration('config_file')
    body_odom_topic = LaunchConfiguration('body_odom_topic')
    
    # TF frame publisher node (direct odometry mode)
    tf_publisher_node = Node(
        package='robot_localization_fusion',
        executable='tf_publisher.py',
        name='tf_frame_publisher',
        output='screen',
        parameters=[{
            'fusion_mode': 'direct_odometry',
            'body_odom_topic': body_odom_topic
        }]
    )
    
    # EKF filter node for fusing LIO odometry and livox IMU
    ekf_filter_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('/odometry/filtered', '/odom'),
            ('/pose/filtered', '/pose')
        ]
    )
    
    return LaunchDescription([
        config_file_arg,
        body_odom_topic_arg,
        tf_publisher_node,
        ekf_filter_node,
    ])
