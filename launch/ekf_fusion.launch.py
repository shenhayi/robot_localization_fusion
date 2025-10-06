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
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Get launch configurations
    config_file = LaunchConfiguration('config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # EKF filter node
    ekf_filter_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('/odometry/filtered', '/odom'),
            ('/pose/filtered', '/pose')
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    # Optional: Static transform publisher for base_link to imu_link
    # Uncomment if you need a static transform between base_link and IMU
    # static_transform_publisher = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='base_link_to_imu_link',
    #     arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
    # )
    
    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        ekf_filter_node,
        # static_transform_publisher,  # Uncomment if needed
    ])
