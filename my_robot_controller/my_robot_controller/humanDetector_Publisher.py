import cv2
import mediapipe as mp
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class poseDetector():
    def __init__(self, mode=False, upBody=False, modelC=1, smooth=True, detectionCon=0.5, trackCon=0.5):
        self.mode = mode
        self.modelC = modelC
        self.upBody = upBody
        self.smooth = smooth
        self.detectionCon = detectionCon
        self.trackCon = trackCon

        self.mpPose = mp.solutions.pose
        self.pose = self.mpPose.Pose(self.mode, self.upBody, self.modelC, self.smooth, self.detectionCon, self.trackCon)
        self.mpDraw = mp.solutions.drawing_utils

    def findPose(self, img, draw=True):
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        results = self.pose.process(imgRGB)

        if results.pose_landmarks:
            if draw:
                self.mpDraw.draw_landmarks(img, results.pose_landmarks, self.mpPose.POSE_CONNECTIONS)
            return True
        return False

class PoseDetectionNode(Node):
    def __init__(self):
        super().__init__('pose_detection_node')
        self.publisher_ = self.create_publisher(Bool, 'human_detected', 10)
        self.timer = self.create_timer(0.1, self.detect_pose)
        self.detector = poseDetector()
        self.cap = cv2.VideoCapture(0)

    def detect_pose(self):
        success, img = self.cap.read()
        if not success:
            return

        # Detect pose and publish result
        detected = self.detector.findPose(img)
        msg = Bool()
        msg.data = detected
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

        # Display image
        cv2.imshow("Pose Detection", img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    pose_detection_node = PoseDetectionNode()
    try:
        rclpy.spin(pose_detection_node)
    except KeyboardInterrupt:
        pass
    finally:
        pose_detection_node.cap.release()
        cv2.destroyAllWindows()
        pose_detection_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
