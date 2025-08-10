#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray, KeyValue
import socket
import struct
import threading
import time
from typing import Optional


class VESCCANProtocol:
    """VESC CAN protocol constants and message handling"""
    
    # CAN message IDs (base + motor_id)
    CMD_SET_RPM = 0x300
    CMD_SET_CURRENT = 0x400
    CMD_SET_DUTY_CYCLE = 0x500
    
    STATUS_1 = 0x900  # RPM, Current, Duty Cycle
    STATUS_2 = 0xA00  # Amp Hours, Amp Hours Charged
    STATUS_3 = 0xB00  # Watt Hours, Watt Hours Charged
    STATUS_4 = 0xC00  # Temp FETC, Temp Motor, Current In, PID Pos
    STATUS_5 = 0xD00  # Tacho, Voltage In
    
    @staticmethod
    def pack_rpm_command(rpm: float) -> bytes:
        """Pack RPM command for CAN transmission"""
        rpm_int = int(rpm)
        return struct.pack('>i', rpm_int)[:4]
    
    @staticmethod
    def pack_duty_command(duty: float) -> bytes:
        """Pack duty cycle command for CAN transmission (-1.0 to 1.0)"""
        duty_int = int(duty * 100000)
        return struct.pack('>i', duty_int)[:4]
    
    @staticmethod
    def unpack_status_1(data: bytes) -> dict:
        """Unpack STATUS_1 message: RPM, Current, Duty Cycle"""
        if len(data) >= 8:
            rpm, current, duty = struct.unpack('>ihhxx', data[:8])
            return {
                'rpm': float(rpm),
                'current': float(current) / 10.0,
                'duty_cycle': float(duty) / 1000.0
            }
        return {}
    
    @staticmethod
    def unpack_status_5(data: bytes) -> dict:
        """Unpack STATUS_5 message: Tacho, Voltage"""
        if len(data) >= 8:
            tacho, voltage = struct.unpack('>ih', data[:6])
            return {
                'tacho': int(tacho),
                'voltage': float(voltage) / 10.0
            }
        return {}


class VESCMotorNode(Node):
    """ROS2 node for controlling a single VESC motor via CAN bus"""
    
    def __init__(self):
        super().__init__('vesc_motor_node')
        
        # Declare parameters
        self.declare_parameter('motor_id', 0)
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('control_mode', 'rpm')  # rpm, duty_cycle
        self.declare_parameter('max_rpm', 3000.0)
        self.declare_parameter('min_rpm', -3000.0)
        self.declare_parameter('wheel_radius', 0.1)  # meters
        self.declare_parameter('publish_rate', 20.0)  # Hz
        self.declare_parameter('command_timeout', 1.0)  # seconds
        
        # Get parameters
        self.motor_id = self.get_parameter('motor_id').value
        self.can_interface = self.get_parameter('can_interface').value
        self.control_mode = self.get_parameter('control_mode').value
        self.max_rpm = self.get_parameter('max_rpm').value
        self.min_rpm = self.get_parameter('min_rpm').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.command_timeout = self.get_parameter('command_timeout').value
        
        # Initialize CAN socket
        self.can_socket = None
        self.setup_can_socket()
        
        # State variables
        self.last_command_time = time.time()
        self.current_rpm = 0.0
        self.current_voltage = 0.0
        self.current_current = 0.0
        self.duty_cycle = 0.0
        self.tacho_count = 0
        self.last_tacho = 0
        
        # Create publishers
        self.joint_state_pub = self.create_publisher(
            JointState, f'motor_{self.motor_id}/joint_states', 10)
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10)
        
        # Create subscribers
        if self.control_mode == 'rpm':
            self.cmd_sub = self.create_subscription(
                Float64, f'motor_{self.motor_id}/cmd_rpm', 
                self.rpm_command_callback, 10)
        elif self.control_mode == 'duty_cycle':
            self.cmd_sub = self.create_subscription(
                Float64, f'motor_{self.motor_id}/cmd_duty', 
                self.duty_command_callback, 10)
        
        # Create velocity subscriber for convenience
        self.vel_sub = self.create_subscription(
            Float64, f'motor_{self.motor_id}/cmd_vel',
            self.velocity_command_callback, 10)
        
        # Create timers
        self.status_timer = self.create_timer(
            1.0 / self.publish_rate, self.publish_status)
        self.safety_timer = self.create_timer(0.1, self.safety_check)
        
        # Start CAN receiver thread
        self.can_thread = threading.Thread(target=self.can_receiver_thread, daemon=True)
        self.can_thread.start()
        
        self.get_logger().info(
            f'VESC Motor Node started - ID: {self.motor_id}, '
            f'Interface: {self.can_interface}, Mode: {self.control_mode}')
    
    def setup_can_socket(self):
        """Initialize CAN socket"""
        try:
            self.can_socket = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            self.can_socket.bind((self.can_interface,))
            self.can_socket.settimeout(0.1)
            self.get_logger().info(f'CAN socket connected to {self.can_interface}')
        except Exception as e:
            self.get_logger().error(f'Failed to setup CAN socket: {e}')
            self.can_socket = None
    
    def rpm_command_callback(self, msg: Float64):
        """Handle RPM command"""
        rpm = max(min(msg.data, self.max_rpm), self.min_rpm)
        self.send_rpm_command(rpm)
        self.last_command_time = time.time()
    
    def duty_command_callback(self, msg: Float64):
        """Handle duty cycle command"""
        duty = max(min(msg.data, 1.0), -1.0)
        self.send_duty_command(duty)
        self.last_command_time = time.time()
    
    def velocity_command_callback(self, msg: Float64):
        """Handle velocity command (m/s) and convert to RPM"""
        if self.control_mode == 'rpm':
            # Convert m/s to RPM: v = ω * r, ω = v/r, RPM = ω * 60/(2π)
            angular_velocity = msg.data / self.wheel_radius  # rad/s
            rpm = angular_velocity * 60.0 / (2.0 * 3.14159)
            rpm = max(min(rpm, self.max_rpm), self.min_rpm)
            self.send_rpm_command(rpm)
            self.last_command_time = time.time()
    
    def send_rpm_command(self, rpm: float):
        """Send RPM command via CAN"""
        if not self.can_socket:
            return
        
        try:
            can_id = VESCCANProtocol.CMD_SET_RPM + self.motor_id
            data = VESCCANProtocol.pack_rpm_command(rpm)
            frame = struct.pack('<IB3x', can_id, len(data)) + data.ljust(8, b'\x00')
            self.can_socket.send(frame)
        except Exception as e:
            self.get_logger().warn(f'Failed to send RPM command: {e}')
    
    def send_duty_command(self, duty: float):
        """Send duty cycle command via CAN"""
        if not self.can_socket:
            return
        
        try:
            can_id = VESCCANProtocol.CMD_SET_DUTY_CYCLE + self.motor_id
            data = VESCCANProtocol.pack_duty_command(duty)
            frame = struct.pack('<IB3x', can_id, len(data)) + data.ljust(8, b'\x00')
            self.can_socket.send(frame)
        except Exception as e:
            self.get_logger().warn(f'Failed to send duty command: {e}')
    
    def can_receiver_thread(self):
        """Thread for receiving CAN messages"""
        while rclpy.ok() and self.can_socket:
            try:
                frame = self.can_socket.recv(16)
                if len(frame) >= 8:
                    can_id, length = struct.unpack('<IB3x', frame[:8])
                    data = frame[8:8+length]
                    self.process_can_message(can_id, data)
            except socket.timeout:
                continue
            except Exception as e:
                if rclpy.ok():
                    self.get_logger().warn(f'CAN receive error: {e}')
    
    def process_can_message(self, can_id: int, data: bytes):
        """Process received CAN message"""
        motor_id = can_id & 0xFF
        if motor_id != self.motor_id:
            return
        
        msg_type = can_id & 0xFF00
        
        if msg_type == VESCCANProtocol.STATUS_1:
            status = VESCCANProtocol.unpack_status_1(data)
            if status:
                self.current_rpm = status['rpm']
                self.current_current = status['current']
                self.duty_cycle = status['duty_cycle']
        
        elif msg_type == VESCCANProtocol.STATUS_5:
            status = VESCCANProtocol.unpack_status_5(data)
            if status:
                self.tacho_count = status['tacho']
                self.current_voltage = status['voltage']
    
    def publish_status(self):
        """Publish motor status as JointState"""
        # Calculate position from tacho count
        position = (self.tacho_count - self.last_tacho) * 2.0 * 3.14159 / 6.0  # Assuming 6 pole pairs
        
        # Calculate velocity from RPM
        velocity = self.current_rpm * 2.0 * 3.14159 / 60.0  # rad/s
        
        # Create JointState message
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = f'motor_{self.motor_id}'
        joint_state.name = [f'motor_{self.motor_id}_joint']
        joint_state.position = [position]
        joint_state.velocity = [velocity]
        joint_state.effort = [self.current_current]
        
        self.joint_state_pub.publish(joint_state)
        
        # Publish diagnostics
        self.publish_diagnostics()
    
    def publish_diagnostics(self):
        """Publish diagnostic information"""
        diagnostic_array = DiagnosticArray()
        diagnostic_array.header.stamp = self.get_clock().now().to_msg()
        
        status = DiagnosticStatus()
        status.name = f'VESC Motor {self.motor_id}'
        status.hardware_id = f'motor_{self.motor_id}'
        
        # Determine status level
        if time.time() - self.last_command_time > self.command_timeout:
            status.level = DiagnosticStatus.WARN
            status.message = 'No recent commands'
        elif abs(self.current_voltage) < 10.0:  # Low voltage threshold
            status.level = DiagnosticStatus.WARN
            status.message = 'Low voltage'
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'Operating normally'
        
        # Add key values
        status.values = [
            KeyValue(key='RPM', value=str(self.current_rpm)),
            KeyValue(key='Current (A)', value=str(self.current_current)),
            KeyValue(key='Voltage (V)', value=str(self.current_voltage)),
            KeyValue(key='Duty Cycle', value=str(self.duty_cycle)),
            KeyValue(key='Tacho Count', value=str(self.tacho_count)),
        ]
        
        diagnostic_array.status = [status]
        self.diagnostics_pub.publish(diagnostic_array)
    
    def safety_check(self):
        """Safety check - stop motor if no commands received"""
        if time.time() - self.last_command_time > self.command_timeout:
            if self.control_mode == 'rpm':
                self.send_rpm_command(0.0)
            else:
                self.send_duty_command(0.0)
    
    def destroy_node(self):
        """Cleanup when node is destroyed"""
        if self.can_socket:
            # Send stop command
            if self.control_mode == 'rpm':
                self.send_rpm_command(0.0)
            else:
                self.send_duty_command(0.0)
            self.can_socket.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VESCMotorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()