#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch four-motor rover system with VESC motors"""
    
    # Package directory
    pkg_share = FindPackageShare('vesc_driver')
    
    # Launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'four_motor_config.yaml']),
        description='Path to motor configuration file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Configuration
    config_file = LaunchConfiguration('config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Front Left Motor (ID 0)
    front_left_motor = Node(
        package='vesc_driver',
        executable='vesc_motor_node',
        name='front_left_motor',
        namespace='vesc',
        parameters=[
            config_file,
            {
                'motor_id': 0,
                'use_sim_time': use_sim_time,
            }
        ],
        output='screen',
        emulate_tty=True
    )
    
    # Front Right Motor (ID 1)
    front_right_motor = Node(
        package='vesc_driver',
        executable='vesc_motor_node',
        name='front_right_motor',
        namespace='vesc',
        parameters=[
            config_file,
            {
                'motor_id': 1,
                'use_sim_time': use_sim_time,
            }
        ],
        output='screen',
        emulate_tty=True
    )
    
    # Rear Left Motor (ID 2)
    rear_left_motor = Node(
        package='vesc_driver',
        executable='vesc_motor_node',
        name='rear_left_motor',
        namespace='vesc',
        parameters=[
            config_file,
            {
                'motor_id': 2,
                'use_sim_time': use_sim_time,
            }
        ],
        output='screen',
        emulate_tty=True
    )
    
    # Rear Right Motor (ID 3)
    rear_right_motor = Node(
        package='vesc_driver',
        executable='vesc_motor_node',
        name='rear_right_motor',
        namespace='vesc',
        parameters=[
            config_file,
            {
                'motor_id': 3,
                'use_sim_time': use_sim_time,
            }
        ],
        output='screen',
        emulate_tty=True
    )
    
    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        front_left_motor,
        front_right_motor,
        rear_left_motor,
        rear_right_motor,
    ])