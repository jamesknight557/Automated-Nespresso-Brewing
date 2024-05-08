#!/usr/bin/env python3
#my_robot_controller/my_robot_controller$ ./joint_subscriber_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math

from robolink import *    # RoboDK's API
from robodk import *      # Math toolbox for robots

class JointAngleSubscriber(Node):
    next_position = []
    def __init__(self):

        super().__init__('joint_angle_subscriber')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'joint_angle_publisher',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def get_next_position(self):
        return self.next_position

    def listener_callback(self, msg):
        # Convert received joint angles from radians to degrees
        joint_angles_degrees = [math.degrees(angle) for angle in msg.data]
        self.get_logger().info('Received joint angles: %s' % joint_angles_degrees)
        self.next_position = joint_angles_degrees

def main(args=None):

    # Start the RoboDK API:
    RDK = Robolink()
    current_command = []
     
    #Get the robot (first robot found):
    robot = RDK.Item('', ITEM_TYPE_ROBOT)

    rclpy.init(args=args)
    joint_angle_subscriber = JointAngleSubscriber()


    while(True):
        print("starting")
        rclpy.spin_once(joint_angle_subscriber)
        print("looping")
        new_command = joint_angle_subscriber.get_next_position()
        if new_command != current_command:
            current_command=new_command
            
            print('Moving')

            robot.MoveJ(new_command)



    joint_angle_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
