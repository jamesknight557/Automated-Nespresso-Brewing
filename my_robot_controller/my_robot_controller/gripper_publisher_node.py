#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64

import math

class GripperPublisher(Node):
    def __init__(self):
        super().__init__('gripper_publisher')
        
        self.mm_publisher_ = self.create_publisher(Float64, 'mm', 10)
        self.mm_timer_ = self.create_timer(1.0, self.publish_mm)

    
    def publish_mm(self):
        print("Please input the distance in millimeters (between 0 and 40):")
        mm_input = input()

        try:
            mm_float = float(mm_input)
            if mm_float < 0 or mm_float > 40:
                raise ValueError
        except ValueError:
            self.get_logger().error("Invalid input. Please provide a numeric value between 0 and 40 for millimeters.")
            return

        mm_msg = Float64()
        mm_msg.data = mm_float

        self.get_logger().info("Publishing millimeters: {}".format(mm_msg.data))
        self.mm_publisher_.publish(mm_msg)

def main(args=None):
    rclpy.init(args=args)
    gripper_publisher = GripperPublisher()
    rclpy.spin(gripper_publisher)
    gripper_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
