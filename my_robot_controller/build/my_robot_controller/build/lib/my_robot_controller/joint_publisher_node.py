#!/usr/bin/env python3
#my_robot_controller/my_robot_controller$ ./joint_publisher_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math

class JointAnglePublisher(Node):
    def __init__(self):
        super().__init__('joint_angle_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, 'joint_angle_publisher', 10)
        self.timer_ = self.create_timer(1.0, self.publish_joint_angles)

    def publish_joint_angles(self):
        joint_angles = Float64MultiArray()
        joint_angles.data = []

        print("Please input 6 joint angles separated by space (in degrees):")
        input_angles = input().split()

        if len(input_angles) != 6:
            self.get_logger().error("Invalid number of joint angles provided. Please provide exactly 6 joint angles.")
            return

        try:
            for angle_str in input_angles:
                angle_float = math.radians(float(angle_str))
                joint_angles.data.append(angle_float)
        except ValueError:
            self.get_logger().error("Invalid input. Please provide numeric values for joint angles.")
            return

        self.get_logger().info("Publishing joint angles: {}".format(joint_angles.data))
        self.publisher_.publish(joint_angles)

def main(args=None):
    rclpy.init(args=args)
    joint_angle_publisher = JointAnglePublisher()
    rclpy.spin(joint_angle_publisher)
    joint_angle_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
