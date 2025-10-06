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
    
    # Get launch configurations
    config_file = LaunchConfiguration('config_file')
    
    # EKF filter node for fusing odometry and IMU
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
        ekf_filter_node,
    ])
