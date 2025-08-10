#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
import numpy as np


class VESCSkidSteerNode(Node):
    """4-wheel skid-steer controller for VESC motors compatible with Nav2"""
    
    def __init__(self):
        super().__init__('vesc_skid_steer_node')
        
        # Declare parameters
        self.declare_parameter('wheel_base', 0.5)  # Distance between front/rear wheels (m)
        self.declare_parameter('wheel_track', 0.4)  # Distance between left/right wheels (m)
        self.declare_parameter('wheel_radius', 0.1)  # Wheel radius (m)
        self.declare_parameter('max_linear_speed', 2.0)  # m/s
        self.declare_parameter('max_angular_speed', 2.0)  # rad/s
        
        # Motor IDs
        self.declare_parameter('front_left_motor_id', 0)
        self.declare_parameter('front_right_motor_id', 1)
        self.declare_parameter('rear_left_motor_id', 2)
        self.declare_parameter('rear_right_motor_id', 3)
        
        # Publishing and frame options
        self.declare_parameter('publish_odom', True)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_rate', 50.0)  # Hz
        
        # Get parameters
        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_track = self.get_parameter('wheel_track').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        
        self.front_left_id = self.get_parameter('front_left_motor_id').value
        self.front_right_id = self.get_parameter('front_right_motor_id').value
        self.rear_left_id = self.get_parameter('rear_left_motor_id').value
        self.rear_right_id = self.get_parameter('rear_right_motor_id').value
        
        self.publish_odom = self.get_parameter('publish_odom').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_rate = self.get_parameter('odom_rate').value
        
        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Wheel positions and velocities
        self.wheel_positions = {
            'front_left': 0.0,
            'front_right': 0.0,
            'rear_left': 0.0,
            'rear_right': 0.0
        }
        
        self.last_wheel_positions = {
            'front_left': 0.0,
            'front_right': 0.0,
            'rear_left': 0.0,
            'rear_right': 0.0
        }
        
        self.wheel_velocities = {
            'front_left': 0.0,
            'front_right': 0.0,
            'rear_left': 0.0,
            'rear_right': 0.0
        }
        
        self.last_time = self.get_clock().now()
        
        # Publishers for motor commands
        self.front_left_pub = self.create_publisher(
            Float64, f'motor_{self.front_left_id}/cmd_vel', 10)
        self.front_right_pub = self.create_publisher(
            Float64, f'motor_{self.front_right_id}/cmd_vel', 10)
        self.rear_left_pub = self.create_publisher(
            Float64, f'motor_{self.rear_left_id}/cmd_vel', 10)
        self.rear_right_pub = self.create_publisher(
            Float64, f'motor_{self.rear_right_id}/cmd_vel', 10)
        
        # Odometry publisher
        if self.publish_odom:
            self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        
        # TF broadcaster
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        # Joint state subscribers for each motor
        self.front_left_sub = self.create_subscription(
            JointState, f'motor_{self.front_left_id}/joint_states',
            lambda msg: self.joint_callback(msg, 'front_left'), 10)
        
        self.front_right_sub = self.create_subscription(
            JointState, f'motor_{self.front_right_id}/joint_states',
            lambda msg: self.joint_callback(msg, 'front_right'), 10)
        
        self.rear_left_sub = self.create_subscription(
            JointState, f'motor_{self.rear_left_id}/joint_states',
            lambda msg: self.joint_callback(msg, 'rear_left'), 10)
        
        self.rear_right_sub = self.create_subscription(
            JointState, f'motor_{self.rear_right_id}/joint_states',
            lambda msg: self.joint_callback(msg, 'rear_right'), 10)
        
        # Timer for odometry publishing
        if self.publish_odom or self.publish_tf:
            self.odom_timer = self.create_timer(
                1.0 / self.odom_rate, self.publish_odometry)
        
        self.get_logger().info(
            f'4-Wheel Skid Steer Node started - Motors: FL={self.front_left_id}, '
            f'FR={self.front_right_id}, RL={self.rear_left_id}, RR={self.rear_right_id}')
    
    def cmd_vel_callback(self, msg: Twist):
        """Convert cmd_vel to individual wheel velocities for skid-steer"""
        # Extract linear and angular velocities
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        
        # Apply speed limits
        linear_vel = max(min(linear_vel, self.max_linear_speed), -self.max_linear_speed)
        angular_vel = max(min(angular_vel, self.max_angular_speed), -self.max_angular_speed)
        
        # Skid-steer kinematics:
        # For skid-steer, we treat it like a differential drive but with 4 wheels
        # Left side wheels move together, right side wheels move together
        # The effective "wheelbase" for turning is the wheel track (left-right distance)
        
        half_track = self.wheel_track / 2.0
        left_vel = linear_vel - angular_vel * half_track
        right_vel = linear_vel + angular_vel * half_track
        
        # Publish commands to all motors
        # Left side motors (front_left and rear_left)
        left_msg = Float64()
        left_msg.data = left_vel
        self.front_left_pub.publish(left_msg)
        self.rear_left_pub.publish(left_msg)
        
        # Right side motors (front_right and rear_right)
        right_msg = Float64()
        right_msg.data = right_vel
        self.front_right_pub.publish(right_msg)
        self.rear_right_pub.publish(right_msg)
    
    def joint_callback(self, msg: JointState, wheel_name: str):
        """Handle joint state updates from individual motors"""
        if len(msg.position) > 0 and len(msg.velocity) > 0:
            # Convert joint position to wheel position (accumulate distance traveled)
            wheel_position = msg.position[0] * self.wheel_radius
            wheel_velocity = msg.velocity[0] * self.wheel_radius
            
            self.wheel_positions[wheel_name] = wheel_position
            self.wheel_velocities[wheel_name] = wheel_velocity
    
    def publish_odometry(self):
        """Compute and publish odometry from 4-wheel encoder data"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt <= 0:
            return
        
        # Calculate wheel displacements
        wheel_deltas = {}
        for wheel in self.wheel_positions:
            wheel_deltas[wheel] = self.wheel_positions[wheel] - self.last_wheel_positions[wheel]
        
        # For skid-steer odometry, average the left and right sides
        # Left side: average of front_left and rear_left
        # Right side: average of front_right and rear_right
        left_delta = (wheel_deltas['front_left'] + wheel_deltas['rear_left']) / 2.0
        right_delta = (wheel_deltas['front_right'] + wheel_deltas['rear_right']) / 2.0
        
        # Calculate robot motion (same as differential drive)
        linear_displacement = (left_delta + right_delta) / 2.0
        angular_displacement = (right_delta - left_delta) / self.wheel_track
        
        # Update robot pose using standard differential drive equations
        delta_theta = angular_displacement
        delta_x = linear_displacement * math.cos(self.theta + delta_theta / 2.0)
        delta_y = linear_displacement * math.sin(self.theta + delta_theta / 2.0)
        
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        
        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Calculate velocities
        linear_vel = linear_displacement / dt if dt > 0 else 0.0
        angular_vel = angular_displacement / dt if dt > 0 else 0.0
        
        # Publish odometry message
        if self.publish_odom:
            odom_msg = Odometry()
            odom_msg.header.stamp = current_time.to_msg()
            odom_msg.header.frame_id = self.odom_frame
            odom_msg.child_frame_id = self.base_frame
            
            # Position
            odom_msg.pose.pose.position.x = self.x
            odom_msg.pose.pose.position.y = self.y
            odom_msg.pose.pose.position.z = 0.0
            
            # Orientation (quaternion from yaw)
            odom_msg.pose.pose.orientation.x = 0.0
            odom_msg.pose.pose.orientation.y = 0.0
            odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
            odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)
            
            # Velocity
            odom_msg.twist.twist.linear.x = linear_vel
            odom_msg.twist.twist.linear.y = 0.0
            odom_msg.twist.twist.linear.z = 0.0
            odom_msg.twist.twist.angular.x = 0.0
            odom_msg.twist.twist.angular.y = 0.0
            odom_msg.twist.twist.angular.z = angular_vel
            
            # Covariance matrices (4-wheel typically more accurate than 2-wheel)
            pose_cov = [0.05, 0.0, 0.0, 0.0, 0.0, 0.0,    # x
                       0.0, 0.05, 0.0, 0.0, 0.0, 0.0,     # y
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,      # z
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,      # roll
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,      # pitch
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.05]     # yaw
            
            twist_cov = [0.02, 0.0, 0.0, 0.0, 0.0, 0.0,   # vx
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,     # vy
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,     # vz
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,     # vroll
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,     # vpitch
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.02]    # vyaw
            
            odom_msg.pose.covariance = pose_cov
            odom_msg.twist.covariance = twist_cov
            
            self.odom_pub.publish(odom_msg)
        
        # Publish TF transform
        if self.publish_tf:
            transform = TransformStamped()
            transform.header.stamp = current_time.to_msg()
            transform.header.frame_id = self.odom_frame
            transform.child_frame_id = self.base_frame
            
            transform.transform.translation.x = self.x
            transform.transform.translation.y = self.y
            transform.transform.translation.z = 0.0
            
            transform.transform.rotation.x = 0.0
            transform.transform.rotation.y = 0.0
            transform.transform.rotation.z = math.sin(self.theta / 2.0)
            transform.transform.rotation.w = math.cos(self.theta / 2.0)
            
            self.tf_broadcaster.sendTransform(transform)
        
        # Update for next iteration
        for wheel in self.wheel_positions:
            self.last_wheel_positions[wheel] = self.wheel_positions[wheel]
        self.last_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = VESCSkidSteerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()