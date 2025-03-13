import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from irobot_create_msgs.action import RotateAngle

class DriveRobot(Node):
    def __init__(self):
        super().__init__('drive_robot')
        self.client_rotate = ActionClient(self, RotateAngle, '/rotate_angle')
        self.turn_90_degrees()

    def turn_90_degrees(self):
        # Rotate 90 degrees (pi/2 radians)
        self.get_logger().info("Rotating 90 degrees...")
        self.client_rotate.wait_for_server()
        rotate_goal = RotateAngle.Goal()
        rotate_goal.angle = 1.57  # 90 degrees = pi/2 radians
        rotate_goal.max_rotation_speed = 0.5  # Radians per second
        self.client_rotate.send_goal_async(rotate_goal, feedback_callback=self.rotation_feedback_callback)

    def rotation_feedback_callback(self, feedback_msg):
        self.get_logger().info(f"Remaining rotation: {feedback_msg.feedback.remaining_angle:.2f} radians")

def main(args=None):
    rclpy.init(args=args)
    node = DriveRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
