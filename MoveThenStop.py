#this code has the robot move straight until it detects something within 6 inches

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from irobot_create_msgs.action import DriveDistance
import RPi.GPIO as GPIO
import time

# GPIO pins for the ultrasonic sensor
GPIO_TRIGGER = 40
GPIO_ECHO = 38
GPIO.setmode(GPIO.BOARD)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

class DriveRobot(Node):
    def __init__(self):
        super().__init__('drive_robot')
        self.client = ActionClient(self, DriveDistance, '/drive_distance')
        self.send_drive_command()

    def send_drive_command(self):
        self.get_logger().info("Waiting for action server...")
        self.client.wait_for_server()

        goal_msg = DriveDistance.Goal()
        goal_msg.distance = 1.0  # Move 1 meter forward
        goal_msg.max_translation_speed = 0.3
        
        self.get_logger().info("Sending drive command...")
        self.client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        # Continuously monitor the distance from the ultrasonic sensor
        while rclpy.ok():
            distance = self.measure_distance()
            self.get_logger().info(f"Distance to object: {distance} cm")
            
            if distance <= 15:  # Stop if an object is detected within 15 cm
                self.get_logger().info("Object detected, stopping robot!")
                self.stop_robot()
                break

            time.sleep(0.1)  # Delay to avoid overwhelming the sensor

    def measure_distance(self):
        # Set Trigger to HIGH
        GPIO.output(GPIO_TRIGGER, True)
        time.sleep(0.00001)
        GPIO.output(GPIO_TRIGGER, False)

        start_time = time.time()
        stop_time = time.time()

        # Wait for the echo signal to be HIGH
        while GPIO.input(GPIO_ECHO) == 0:
            start_time = time.time()

        while GPIO.input(GPIO_ECHO) == 1:
            stop_time = time.time()

        # Calculate the distance
        time_elapsed = stop_time - start_time
        distance_cm = round((time_elapsed * 34300) / 2, 2)  # Convert to cm
        return distance_cm

    def stop_robot(self):
        # Stop the robot by setting distance to 0
        goal_msg = DriveDistance.Goal()
        goal_msg.distance = 0.0  # Stop moving
        goal_msg.max_translation_speed = 0.0
        self.client.send_goal_async(goal_msg)

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
