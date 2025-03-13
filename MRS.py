#this code has the robot go straight until it detects something 6in out and then 
#stops, rotates, and stops again 

import RPi.GPIO as GPIO
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from irobot_create_msgs.action import DriveDistance, RotateAngle

# GPIO pins for the ultrasonic sensor
GPIO_TRIGGER = 40
GPIO_ECHO = 38

GPIO.setmode(GPIO.BOARD)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

# ROS2 setup
rclpy.init()
node = rclpy.create_node('drive_robot')
client_drive = ActionClient(node, DriveDistance, '/drive_distance')
client_rotate = ActionClient(node, RotateAngle, '/rotate_angle')

def measure_distance():
    # Send trigger pulse
    GPIO.output(GPIO_TRIGGER, True)
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)

    # Measure time of flight
    start_time = time.time()
    stop_time = time.time()

    # Save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        start_time = time.time()

    # Save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        stop_time = time.time()

    # Calculate distance in centimeters
    time_elapsed = stop_time - start_time
    distance_cm = round((time_elapsed * 34300) / 2, 2)
    
    return distance_cm

#defining functions: ??

def move_forward():
    print("Moving forward...")
    client_drive.wait_for_server()

    goal_msg = DriveDistance.Goal()
    goal_msg.distance = 1.0  # Move 1 meter forward
    goal_msg.max_translation_speed = 0.3
    
    client_drive.send_goal_async(goal_msg)

def stop_robot():
    print("Stopping the robot...")
    stop_goal = DriveDistance.Goal()
    stop_goal.distance = 0.0
    stop_goal.max_translation_speed = 0.0
    client_drive.send_goal_async(stop_goal)

def turn_90_degrees():
    print("Rotating 90 degrees...")
    client_rotate.wait_for_server()

    rotate_goal = RotateAngle.Goal()
    rotate_goal.angle = 1.57  # 90 degrees = pi/2 radians
    rotate_goal.max_rotation_speed = 0.5
    
    client_rotate.send_goal_async(rotate_goal)

def main():
    try:
        while True:
            distance = measure_distance()
            print(f"Distance: {distance:.2f} cm")

            if distance > 15.24:
                print("Path is clear — moving straight")
                move_forward()
            else:
                print("Obstacle detected — stopping and rotating")
                stop_robot()
                time.sleep(0.5)
                turn_90_degrees()
                time.sleep(1)
                stop_robot()
                break #stop the loop 

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        GPIO.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

print('shutting down')
        rclpy.shutdown()