import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraSubscriber:
    def __init__(self):
        self.bridge = CvBridge()

        # Initialize ROS 2 node
        rclpy.init()
        self.node = rclpy.create_node('camera_subscriber')

        # Subscribe to the camera feed topic
        self.subscription = self.node.create_subscription(Image, 'camera_feed', self.callback, 10)

    def callback(self, msg):
        # Convert ROS image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Display the image
        cv2.imshow('Camera Feed', cv_image)
        cv2.waitKey(1)  # Necessary to update the OpenCV window

def main():
    camera_subscriber = CameraSubscriber()
    rclpy.spin(camera_subscriber.node)

if __name__ == '__main__':
    main()
