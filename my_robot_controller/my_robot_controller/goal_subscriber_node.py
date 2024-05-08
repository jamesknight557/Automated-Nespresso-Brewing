import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

from robolink import *  # RoboDK's API
from robodk import *  # Math toolbox for robots

class GoalSubscriber(Node):
    def __init__(self):
        super().__init__('goal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'goals',
            self.goal_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.latest_goal = None  # Store the latest received goal
        
        # Initialize publisher for the busy status
        self.busy_publisher = self.create_publisher(Bool, 'busy', 10)
        
    def goal_callback(self, msg):
        self.latest_goal = msg.data
        self.get_logger().info(f"Received and stored goal: {self.latest_goal}")
        
        # Indicate that the robot is busy
        self.publish_busy_status(True)
        
        self.process_command(msg.data)
        
    def process_command(self, command):
        # Start the RoboDK API:
        RDK = Robolink()

        RDK.setRunMode(RUNMODE_RUN_ROBOT)
        
        # Get the robot (first robot found):
        robot = RDK.Item('', ITEM_TYPE_ROBOT)
        
        print('Processing command:', command)
        # Execute the robot command based on the command type
        if command.startswith('j'):
            robot.MoveJ(RDK.Item(command[2:]))
        elif command.startswith('l'):
            robot.MoveL(RDK.Item(command[2:]))
        
        #robot.WaitMove()  # Wait for the movement to complete
        
        # Indicate that the robot is no longer busy
        self.publish_busy_status(False)
        
    def publish_busy_status(self, is_busy):
        busy_msg = Bool()
        busy_msg.data = is_busy
        self.busy_publisher.publish(busy_msg)
        self.get_logger().info(f"Published busy status: {busy_msg.data}")

def main(args=None):
    rclpy.init(args=args)
    goal_subscriber = GoalSubscriber()

    try:
        # Use spin to keep the script from exiting until the node is explicitly shutdown
        rclpy.spin(goal_subscriber)
    except KeyboardInterrupt:
        # Handle the CTRL+C case to gracefully shutdown the node
        goal_subscriber.get_logger().info('Goal subscriber node stopped cleanly')
    finally:
        # Destroy the node explicitly
        goal_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
