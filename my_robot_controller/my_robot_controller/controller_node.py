import tkinter as tk
from PIL import Image, ImageTk
import subprocess
import cv2
from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge
import rclpy

class ROS2NodeController:
    def __init__(self, master):
        self.master = master
        master.title("ROS2 Node Controller")

        # Set window size
        master.geometry("1200x800")

        # Set background color
        master.configure(bg="#ADD8E6")

        # Create a frame for the top right corner
        top_frame = tk.Frame(master, bg="#ADD8E6")
        top_frame.pack(side="top", anchor="ne", padx=10, pady=10)

        # Create input box for robot name
        self.robot_name_label = tk.Label(top_frame, text="Enter Robot Name:")
        self.robot_name_label.pack(side="left", padx=5, pady=5)
        self.robot_name_entry = tk.Entry(top_frame)
        self.robot_name_entry.pack(side="left", padx=5, pady=5)
        
        # Create Enter button
        self.enter_button = tk.Button(top_frame, text="Enter", command=self.display_image)
        self.enter_button.pack(side="left", padx=5, pady=5)

        # Create a label to display the image
        self.image_label = tk.Label(master)
        self.image_label.pack(side="right", padx=10, pady=10, fill=tk.BOTH, expand=True)

        # Create buttons
        self.subscriber_button = tk.Button(master, text="Select Program to Run", command=self.start_publisher, width=30, height=5)
        self.subscriber_button.pack(anchor='w', pady=20)

        self.publisher_button = tk.Button(master, text="Start Goal Subscriber Node", command=self.start_subscriber, width=30, height=5)
        self.publisher_button.pack(anchor='w', pady=20)

        self.gripper_pub_button = tk.Button(master, text="Start Gripper publisher Node", command=self.launch_gripper_pub, width=30, height=5)
        self.gripper_pub_button.pack(anchor='w', pady=20)

        self.gripper_test_button = tk.Button(master, text="Start Gripper Subscriber Node", command=self.launch_gripper_test, width=30, height=5)
        self.gripper_test_button.pack(anchor='w', pady=20)

        self.camera_start_button = tk.Button(master, text="Start Human Tracking and Safety", command=self.start_camera, width=30, height=5)
        self.camera_start_button.pack(anchor='w', pady=20)

    def display_image(self):
        robot_name = self.robot_name_entry.get()
        if robot_name == "ABBIRB120":
            image_path = "ABBIRB120.png"  # Change the path accordingly
        elif robot_name == "UR5":
            image_path = "UR5.png"  # Change the path accordingly
        else:
            # If the entered robot name is not recognized, display a default image or show an error message
            image_path = "default_image.jpg"  # Change the path to a default image or handle the error accordingly
        
        # Load and display the image
        image = Image.open(image_path)
        photo = ImageTk.PhotoImage(image)
        self.image_label.configure(image=photo)
        self.image_label.image = photo

    def launch_gripper_pub(self):
        subprocess.Popen(["gnome-terminal", "--", "bash", "-c", "./gripper_publisher_node.py; exec bash"])

    def start_subscriber(self):
        subprocess.Popen(["gnome-terminal", "--", "bash", "-c", "python3 goal_subscriber_node.py; exec bash"])

    def start_publisher(self):
        subprocess.Popen(["gnome-terminal", "--", "bash", "-c", "python3 programme_publisher_node.py; exec bash"])

    def launch_gripper_test(self):
        subprocess.Popen(["gnome-terminal", "--", "bash", "-c", "python3 gripper_subscribe_node.py; exec bash"])

    def start_camera(self):
        subprocess.Popen(["gnome-terminal", "--", "bash", "-c", "python3 humanDetector_Publisher.py; exec bash"])


def main():
    root = tk.Tk()
    app = ROS2NodeController(root)
    root.mainloop()
    app.camera_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
