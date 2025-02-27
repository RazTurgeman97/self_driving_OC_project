#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
import numpy as np


class SimpleController(Node):    
    def __init__(self):
        super().__init__('simple_controller')
        
        self.declare_parameter("wheel_raduis", 0.033)
        self.declare_parameter("wheel_separation", 0.17)
        
        self.wheel_raduis_ = self.get_parameter("wheel_raduis").get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter("wheel_separation").get_parameter_value().double_value
        
        self.get_logger().info("Using wheel raduis: %f" % self.wheel_raduis_)
        self.get_logger().info("Using wheel separation: %f" % self.wheel_separation_)
        
        self.wheel_cmd_pub_ = self.create_publisher(Float64MultiArray, "simple_velocity_controller/commands", 10)
        self.vel_sub_ = self.create_subscription(TwistStamped, "bumperbot_controller/cmd_vel", self.valCallback, 10)
        
        self.speed_convertion_ = np.array(
            [self.wheel_raduis_/2, self.wheel_raduis_/2],
            [self.wheel_raduis_/self.wheel_separation_, -self.wheel_raduis_/self.wheel_separation_]
        )
        
        self.get_logger().info("The convertion matrix is %s" % self.speed_convertion_)
        
    def valCallback(self, msg):
        robot_speed = np.array([msg.twist.linear.x],
                               [msg.twist.angular.z])
        
        wheel_speed = np.matmul(np.linalg.inv(self.speed_convertion_), (robot_speed))
        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [wheel_speed[1,0], wheel_speed[0 ,0]]
        self.wheel_cmd_pub_.publish(wheel_speed_msg)
        
        
def main(args=None):
    rclpy.init(args=args)
    node = SimpleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()