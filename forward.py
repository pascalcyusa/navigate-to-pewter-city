#move in aa straight line

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from irobot_create_msgs.action import DriveDistance

class DriveRobot(Node):
    def __init__(self):
        super().__init__('drive_robot') 
        self.client = ActionClient(self, DriveDistance, '/drive_distance')
        self.send_drive_command()

    def send_drive_command(self):
        self.get_logger().info("Waiting for action server...")
        self.client.wait_for_server()
        goal_msg = DriveDistance.Goal()
        goal_msg.distance = 2.0  # Move 1 meter forward
        goal_msg.max_translation_speed = 0.3
        self.get_logger().info("Sending drive command...")
        self.client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f"Remaining distance: {feedback_msg.feedback.remaining_travel_distance:.2f} m")
def main(args=None):
    rclpy.init(args=args)
    node = DriveRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()