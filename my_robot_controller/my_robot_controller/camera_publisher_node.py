#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera_feed', 10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)  # Open default camera

    def publish_camera_feed(self):
        ret, frame = self.cap.read()
        if ret:
            image_message = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.publisher_.publish(image_message)

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    
    # Create a timer to publish camera feed periodically
    timer_period = 0.1  # seconds
    camera_publisher.create_timer(timer_period, camera_publisher.publish_camera_feed)
    
    rclpy.spin(camera_publisher)
    
    camera_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
