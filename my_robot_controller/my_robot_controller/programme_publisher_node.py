import csv
import tkinter as tk
from tkinter import simpledialog,Button
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String, Bool
import time

from robolink import *  # RoboDK's API
from robodk import *  # Math toolbox for robots

class GripperPublisher(Node):
    def __init__(self):
        super().__init__('gripper_publisher')
        self.mm_publisher = self.create_publisher(Float64, 'mm', 10)
        self.goal_publisher = self.create_publisher(String, 'goals', 10)

    def publish_mm(self, mm_value):
        mm_float = float(mm_value)
        mm_msg = Float64()
        mm_msg.data = mm_float
        self.mm_publisher.publish(mm_msg)
        self.get_logger().info(f"Publishing millimeters: {mm_msg.data}")
        
    def publish_goal(self, goal_str):
        goal_msg = String()
        goal_msg.data = goal_str
        self.goal_publisher.publish(goal_msg)
        self.get_logger().info(f"Publishing goal: {goal_msg.data}")

class BusySub(Node):
    def __init__(self):
        super().__init__('busy_subscriber')
        self.busy = False
        self.subscription = self.create_subscription(Bool, 'busy', self.busy_callback, 10)
        self.subscription  # prevent unused variable warning

    def busy_callback(self, msg):
        self.busy = msg.data
        self.get_logger().info(f"Received busy status: {self.busy}")


def read_csv_file(file_path):
    data_array = []
    try:
        with open(file_path, newline='') as csvfile:
            reader = csv.reader(csvfile, delimiter=',')
            for row in reader:
                data_array.append(row)
                for item in row:
                    print(item)
    except Exception as e:
        print(f"Failed to read CSV file: {e}")
    return data_array

class PoseSubscriber(Node):
    def __init__(self):
        super().__init__('pose_subscriber')
        self.subscription = self.create_subscription(
            Bool,
            'human_detected',
            self.human_detected_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.human_detected = False

    def human_detected_callback(self, msg):
        self.human_detected = msg.data
        

def open_file_dialog():
    # Create a root window and hide it immediately
    root = tk.Tk()
    root.withdraw()

    # Create a new top-level window for file selection
    file_select_window = tk.Toplevel(root)
    file_select_window.title("Select a File")
    file_select_window.configure(background='light blue')  # Set background color to light blue

    # Initialize variable to store the selected file path
    selected_file = tk.StringVar(value=None)

    # Function to call when a file is selected
    def on_file_select(file_path):
        selected_file.set(file_path)  # Set the file path
        file_select_window.destroy()  # Destroy the file select window
        root.quit()  # Quit the main loop to allow script continuation

    # List all .txt files in the current directory
    files = [f for f in os.listdir('.') if os.path.isfile(f) and f.endswith('.txt')]

    # Create a button for each file
    for file in files:
        btn = Button(file_select_window, text=file, command=lambda f=file: on_file_select(f))
        btn.pack(pady=10, padx=10, ipadx=10, ipady=10)  # Increase padding and internal padding for larger buttons

    root.mainloop()  # Start the main event loop

    # Check if a file was selected and it exists
    file_path = selected_file.get()
    if file_path and os.path.exists(file_path):
        print(f"Opening file: {file_path}")
        return read_csv_file(file_path)
    else:
        print("No file selected or file does not exist.")
        return []

def main(args=None):
    rclpy.init(args=args)
    gripper_publisher = GripperPublisher()
    busy_subscriber = BusySub()
    pose_subscriber = PoseSubscriber()

    RDK = Robolink()
    robot = RDK.Item('', ITEM_TYPE_ROBOT)
    robot_tool = RDK.Item('RobotiQ 2F-140 Gripper (Closed)')

    while True:
        data = open_file_dialog()
        print("All data from CSV:", data)

        safe = True

        for row in data:
            for value in row:

                
                while safe == True:
                    
                    rclpy.spin_once(pose_subscriber, timeout_sec=0.1)  # timeout can be adjusted

                    safe = pose_subscriber.human_detected

                    if safe == False:
                        time.sleep(5)

                if value.isdigit():  # Handle digit values
                    gripper_publisher.publish_mm(value)
                    time.sleep(1)
        
                if value == 'attach':
                    robot_tool.AttachClosest()
                    print("attached")

                if value == 'detach':
                    robot_tool.DetachAll()
                    print("detached")

                else:  # Handle goals
                    busy = True
                    gripper_publisher.publish_goal(value)

                    while busy == True:
                        
                        rclpy.spin_once(busy_subscriber)
                        busy=busy_subscriber.busy

                    time.sleep(1)

            

    gripper_publisher.destroy_node()
    busy_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
