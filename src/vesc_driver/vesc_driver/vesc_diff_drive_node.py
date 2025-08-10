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


class VESCDiffDriveNode(Node):
    """Differential drive controller for VESC motors"""
    
    def __init__(self):
        super().__init__('vesc_diff_drive_node')
        
        # Declare parameters
        self.declare_parameter('wheel_base', 0.5)  # Distance between wheels (m)
        self.declare_parameter('wheel_radius', 0.1)  # Wheel radius (m)
        self.declare_parameter('max_linear_speed', 2.0)  # m/s
        self.declare_parameter('max_angular_speed', 2.0)  # rad/s
        self.declare_parameter('left_motor_id', 0)
        self.declare_parameter('right_motor_id', 1)
        self.declare_parameter('publish_odom', True)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_rate', 50.0)  # Hz
        
        # Get parameters
        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.left_motor_id = self.get_parameter('left_motor_id').value
        self.right_motor_id = self.get_parameter('right_motor_id').value
        self.publish_odom = self.get_parameter('publish_odom').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_rate = self.get_parameter('odom_rate').value
        
        # State variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0
        self.last_left_pos = 0.0
        self.last_right_pos = 0.0
        self.last_time = self.get_clock().now()
        
        # Velocity tracking
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        
        # Publishers
        self.left_cmd_pub = self.create_publisher(
            Float64, f'motor_{self.left_motor_id}/cmd_vel', 10)
        self.right_cmd_pub = self.create_publisher(
            Float64, f'motor_{self.right_motor_id}/cmd_vel', 10)
        
        if self.publish_odom:
            self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        
        # TF broadcaster
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        self.left_joint_sub = self.create_subscription(
            JointState, f'motor_{self.left_motor_id}/joint_states',
            self.left_joint_callback, 10)
        
        self.right_joint_sub = self.create_subscription(
            JointState, f'motor_{self.right_motor_id}/joint_states',
            self.right_joint_callback, 10)
        
        # Timer for odometry publishing
        if self.publish_odom or self.publish_tf:
            self.odom_timer = self.create_timer(
                1.0 / self.odom_rate, self.publish_odometry)
        
        self.get_logger().info(
            f'Differential Drive Node started - '
            f'Left motor: {self.left_motor_id}, Right motor: {self.right_motor_id}, '
            f'Wheelbase: {self.wheel_base}m, Wheel radius: {self.wheel_radius}m')
    
    def cmd_vel_callback(self, msg: Twist):
        """Handle cmd_vel messages and convert to individual wheel velocities"""
        # Extract linear and angular velocities
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        
        # Apply speed limits
        linear_vel = max(min(linear_vel, self.max_linear_speed), -self.max_linear_speed)
        angular_vel = max(min(angular_vel, self.max_angular_speed), -self.max_angular_speed)
        
        # Convert to wheel velocities using differential drive kinematics
        # v_left = linear_vel - (angular_vel * wheelbase) / 2
        # v_right = linear_vel + (angular_vel * wheelbase) / 2
        half_wheelbase = self.wheel_base / 2.0
        left_wheel_vel = linear_vel - angular_vel * half_wheelbase
        right_wheel_vel = linear_vel + angular_vel * half_wheelbase
        
        # Publish wheel velocity commands
        left_msg = Float64()
        left_msg.data = left_wheel_vel
        self.left_cmd_pub.publish(left_msg)
        
        right_msg = Float64()
        right_msg.data = right_wheel_vel
        self.right_cmd_pub.publish(right_msg)
        
        # Store commanded velocities for odometry
        self.linear_velocity = linear_vel
        self.angular_velocity = angular_vel
    
    def left_joint_callback(self, msg: JointState):
        """Handle left motor joint state"""
        if len(msg.position) > 0:
            # Convert joint position to wheel position
            self.left_wheel_pos = msg.position[0] * self.wheel_radius
    
    def right_joint_callback(self, msg: JointState):
        """Handle right motor joint state"""
        if len(msg.position) > 0:
            # Convert joint position to wheel position
            self.right_wheel_pos = msg.position[0] * self.wheel_radius
    
    def publish_odometry(self):
        """Compute and publish odometry"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt <= 0:
            return
        
        # Calculate wheel displacements
        left_delta = self.left_wheel_pos - self.last_left_pos
        right_delta = self.right_wheel_pos - self.last_right_pos
        
        # Calculate robot displacement and rotation
        linear_displacement = (left_delta + right_delta) / 2.0
        angular_displacement = (right_delta - left_delta) / self.wheel_base
        
        # Update robot pose
        self.x += linear_displacement * math.cos(self.theta + angular_displacement / 2.0)
        self.y += linear_displacement * math.sin(self.theta + angular_displacement / 2.0)
        self.theta += angular_displacement
        
        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Calculate velocities
        linear_vel = linear_displacement / dt if dt > 0 else 0.0
        angular_vel = angular_displacement / dt if dt > 0 else 0.0
        
        # Publish odometry
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
            
            # Covariance matrices (simple diagonal)
            pose_cov = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                       0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
            
            twist_cov = [0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.05]
            
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
        self.last_left_pos = self.left_wheel_pos
        self.last_right_pos = self.right_wheel_pos
        self.last_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = VESCDiffDriveNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()