#!/usr/bin/env python3
"""
OpenDroid Hardware Serial Bridge Node
Subscribes to ROS 2 Twist /cmd_vel and transmits kinematic drive commands to
the low-level microcontroller (ESP32/STM32) driving dual Rhino RMCS-2111 planetary motors.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

class MotorBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_motor_bridge')

        # Declare parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('cmd_vel_topic', 'bcr_bot/cmd_vel')

        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value

        self.get_logger().info(f"Connecting to MCU on port '{port}' @ {baudrate} baud...")
        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.1)
            time.sleep(2.0)  # Allow MCU auto-reset to settle
            self.get_logger().info("Connected to OpenDroid MCU serial interface successfully.")
        except Exception as e:
            self.get_logger().warn(f"Could not open serial port '{port}': {e}. Running in simulation/mock mode.")
            self.ser = None

        self.sub = self.create_subscription(
            Twist,
            topic,
            self.cmd_vel_callback,
            10
        )

    def cmd_vel_callback(self, msg: Twist):
        lin_x = msg.linear.x
        ang_z = msg.angular.z

        command_str = f"V,{lin_x:.3f},{ang_z:.3f}\n"

        if self.ser and self.ser.is_open:
            try:
                self.ser.write(command_str.encode('ascii'))
            except Exception as e:
                self.get_logger().error(f"Serial transmission failed: {e}")
        else:
            self.get_logger().debug(f"[MOCK_SERIAL] Dispatched: {command_str.strip()}")

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            # Send stop command before closing
            try:
                self.ser.write(b"STOP\n")
                self.ser.close()
            except Exception:
                pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
