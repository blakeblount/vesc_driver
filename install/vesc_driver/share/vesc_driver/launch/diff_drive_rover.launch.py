#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch differential drive using 2 of the 4 rover motors"""
    
    # Package directory
    pkg_share = FindPackageShare('vesc_driver')
    
    # Launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'motor_params.yaml']),
        description='Path to motor configuration file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    left_motor_id_arg = DeclareLaunchArgument(
        'left_motor_id',
        default_value='0',
        description='CAN ID for left motor'
    )
    
    right_motor_id_arg = DeclareLaunchArgument(
        'right_motor_id',
        default_value='1', 
        description='CAN ID for right motor'
    )
    
    # Configuration
    config_file = LaunchConfiguration('config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    left_motor_id = LaunchConfiguration('left_motor_id')
    right_motor_id = LaunchConfiguration('right_motor_id')
    
    # Left motor node
    left_motor_node = Node(
        package='vesc_driver',
        executable='vesc_motor_node',
        name='left_motor',
        namespace='vesc',
        parameters=[
            config_file,
            {
                'motor_id': left_motor_id,
                'use_sim_time': use_sim_time,
            }
        ],
        output='screen',
        emulate_tty=True
    )
    
    # Right motor node
    right_motor_node = Node(
        package='vesc_driver',
        executable='vesc_motor_node',
        name='right_motor',
        namespace='vesc',
        parameters=[
            config_file,
            {
                'motor_id': right_motor_id,
                'use_sim_time': use_sim_time,
            }
        ],
        output='screen',
        emulate_tty=True
    )
    
    # Differential drive controller
    diff_drive_node = Node(
        package='vesc_driver',
        executable='vesc_diff_drive_node',
        name='diff_drive_controller',
        namespace='vesc',
        parameters=[
            config_file,
            {
                'left_motor_id': left_motor_id,
                'right_motor_id': right_motor_id,
                'use_sim_time': use_sim_time,
            }
        ],
        remappings=[
            ('cmd_vel', '/cmd_vel'),
            ('odom', '/odom'),
        ],
        output='screen',
        emulate_tty=True
    )
    
    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        left_motor_id_arg,
        right_motor_id_arg,
        left_motor_node,
        right_motor_node,
        diff_drive_node,
    ])