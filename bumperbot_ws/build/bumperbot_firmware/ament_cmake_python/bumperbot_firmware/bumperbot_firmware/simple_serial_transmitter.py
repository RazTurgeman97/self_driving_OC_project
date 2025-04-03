#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial


class SimpleSerialTransmitter(Node):

    def __init__(self):
        super().__init__("simple_serial_transmitter")
        
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("timeout", 0.1)
        
        self.port_ = self.get_parameter("port", self.port_)
        self.baud_ = self.get_parameter("baud", self.baud_)
        self.timeout_ = self.get_parameter("timeout", self.timeout_)
        
        self.arduino_ = serial.Serial(port=self.port_, baudrate=self.baud_, timeout=self.timeout_)
        
        self.sub_ = self.create_subscription(String, "serial_transmitter", self.msgCallback, 10)

    def msgCallback(self, msg):
        self.arduino_.write(msg.data.encode("utf-8"))
        self.get_logger().info("Sent: " + msg.data)


def main():
    rclpy.init()

    node = SimpleSerialTransmitter()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()