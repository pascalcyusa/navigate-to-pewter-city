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

def turn_left():
    print("Rotating left 90 degrees...")
    client_rotate.wait_for_server()

    rotate_goal = RotateAngle.Goal()
    rotate_goal.angle = 1.57  # 90 degrees = pi/2 radians
    rotate_goal.max_rotation_speed = 0.5
    
    client_rotate.send_goal_async(rotate_goal)

def main():
    try:
        step = 1  # Initialize step counter

        while True:
            distance = measure_distance()
            print(f"Distance: {distance:.2f} cm")

            if step == 1:
                if distance > 15.24:
                    print("Step 1: Path is clear — moving straight")
                    move_forward()
                else:
                    print("Step 1: Obstacle detected — stopping and rotating")
                    stop_robot()
                    time.sleep(0.5)
                    step = 2  # Move to step 2

            elif step == 2:
                print("Step 2: Turning left 90 degrees")
                stop_robot()  # Make sure robot stops before turning
                time.sleep(0.5)  # Short delay before turning
                turn_left()
                time.sleep(1)  # Give it time to complete the turn
                step = 3  # Move to step 3

            elif step == 3:
                if distance > 15.24:
                    print("Step 3: Path is clear — moving straight")
                    move_forward()
                else:
                    print("Step 3: Obstacle detected — stopping and completing task")
                    stop_robot()
                    break  # End the program after step 3 is done

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        GPIO.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
