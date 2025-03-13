#FINAL CODE THAT HAS FULL PATH 

import RPi.GPIO as GPIO
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from irobot_create_msgs.action import DriveDistance, RotateAngle
from keras.models import load_model
from PIL import Image, ImageOps
import cv2
import numpy as np
from picamera2 import Picamera2
from libcamera import controls

# GPIO setup for ultrasonic sensor
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

# Load object recognition model
model = load_model("all_keras_model.h5", compile=False)
class_names = open("all_labels.txt", "r").readlines()

# Initialize Pi Camera
picam2 = Picamera2()
picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous})
picam2.start()
time.sleep(1)

# Movement direction variables (update these before testing)
first_object = 4  # Example: 0 for PokeBall
first_object_movement = 'right'  # 'left' or 'right'
second_object = 5  # Example: 1 for Squirtle
second_object_movement = 'left'  # 'left' or 'right'
third_object = 0
third_object_movement = 'left'  # 'left' or 'right'
fourth_object = 1
fourth_object_movement = 'right'  # 'left' or 'right'
fifth_object = 2
fifth_object_movement = 'left'  # 'left' or 'right'
sixth_object = 6
sixth_object_movement = 'left'  # 'left' or 'right'
seventh_object = 3
seventh_object_movement = 'right'  # 'left' or 'right'



# Step Tracking
step = 1

# Ultrasonic sensor distance measurement
def measure_distance():
    GPIO.output(GPIO_TRIGGER, True)
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
    
    start_time = time.time()
    stop_time = time.time()

    while GPIO.input(GPIO_ECHO) == 0:
        start_time = time.time()

    while GPIO.input(GPIO_ECHO) == 1:
        stop_time = time.time()

    time_elapsed = stop_time - start_time
    distance_cm = round((time_elapsed * 34300) / 2, 2)
    return distance_cm

# Functions for robot movement
def move_forward():
    print("Moving forward...")
    client_drive.wait_for_server()

    goal_msg = DriveDistance.Goal()
    goal_msg.distance = 1.0
    goal_msg.max_translation_speed = 0.2
    client_drive.send_goal_async(goal_msg)

def stop_robot():
    print("Stopping the robot...")
    stop_goal = DriveDistance.Goal()
    stop_goal.distance = 0.0
    stop_goal.max_translation_speed = 0.0
    client_drive.send_goal_async(stop_goal)

def turn_90_degrees(direction='left'):
    print(f"Turning 90 degrees {direction}...")
    client_rotate.wait_for_server()

    rotate_goal = RotateAngle.Goal()
    rotate_goal.angle = -1.57  # 90 degrees in radians
    rotate_goal.max_rotation_speed = 0.5
    if direction == 'left':
        rotate_goal.angle = 1.57  # Turn left (counterclockwise)
    
    client_rotate.send_goal_async(rotate_goal)

# Object Recognition Loop
def object_recognition():
    image = picam2.capture_array()
    image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    image = cv2.resize(image, (224, 224), interpolation=cv2.INTER_AREA)
    image = np.asarray(image, dtype=np.float32).reshape(1, 224, 224, 3)
    image = (image / 127.5) - 1  # Normalize

    prediction = model.predict(image)
    index = np.argmax(prediction)
    class_name = class_names[index]
    confidence_score = prediction[0][index]

    print(f"Class: {class_name[2:]}, Confidence: {np.round(confidence_score * 100)}%")
    return index, confidence_score

# Main program loop
def main():
    global step, first_object, second_object, first_object_movement

    try:
        while True:
            distance = measure_distance()
            print(f"Distance: {distance:.2f} cm")

            # Step 1: Go straight until object is detected
            if step == 1 and distance > 15.24:
                move_forward()
            elif step == 1 and distance <= 15.24:
                stop_robot()
                step = 2  # Move to step 2

            # Step 2: Object detection and turn
            if step == 2:
                detected_object, _ = object_recognition()

                if detected_object == first_object:
                    print(f"First object {class_names[first_object][2:]} detected!")
                    stop_robot()
                    time.sleep(1)
                    turn_90_degrees(first_object_movement)
                    time.sleep(4)
                    step = 3  # Move to step 3

            # Step 3: Go straight until the second object is detected
            if step == 3:
                distance = measure_distance()
                if distance > 15.24:
                    move_forward()
                elif distance <= 15.24:
                    stop_robot()
                    detected_object, _ = object_recognition()
                    if detected_object == second_object:
                        print(f"Second object {class_names[second_object][2:]} detected!")
                        turn_90_degrees(second_object_movement)
                        time.sleep(4)
                        step = 3  # Move to step 3
            if step == 4:
                distance = measure_distance()
                if distance > 15.24:
                    move_forward()
                elif distance <= 15.24:
                    stop_robot() 
                    detected_object, _ = object_recognition()
                    if detected_object == third_object:
                        print(f"Third object {class_names[third_object][2:]} detected!")
                        turn_90_degrees(third_object_movement)
                        time.sleep(4)
                        step = 5  # Move to step 5
            if step == 5:
                distance = measure_distance()
                if distance > 15.24:
                    move_forward()
                elif distance <= 15.24:
                    stop_robot() 
                    detected_object, _ = object_recognition()
                    if detected_object == fourth_object:
                        print(f"Fourth object {class_names[fourth_object][2:]} detected!")
                        turn_90_degrees(fourth_object_movement)
                        time.sleep(4)
                        step = 6  # Move to step 6        
            if step == 6:
                distance = measure_distance()
                if distance > 15.24:
                    move_forward()
                elif distance <= 15.24:
                    stop_robot() 
                    detected_object, _ = object_recognition()
                    if detected_object == Fifth_object:
                        print(f"Fifth object {class_names[fifth_object][2:]} detected!")
                        turn_90_degrees(fifth_object_movement)
                        time.sleep(4)
                        step = 7  # Move to step 5if step == 6:
            if step == 7:    
                distance = measure_distance()
                if distance > 15.24:
                    move_forward()
                elif distance <= 15.24:
                    stop_robot() 
                    detected_object, _ = object_recognition()
                    if detected_object == sixth_object:
                        print(f"Sixth object {class_names[sixth_object][2:]} detected!")
                        turn_90_degrees(sixth_object_movement)
                        time.sleep(4)
                        step = 8  # Move to step 5        
            if step == 8:
                distance = measure_distance()
                if distance > 15.24:
                    move_forward()
                elif distance <= 15.24:
                    stop_robot() 
                    detected_object, _ = object_recognition()
                    if detected_object == seventh_object:
                        print(f"Seventh object {class_names[seventh_object][2:]} detected!")
                        turn_90_degrees(seventh_object_movement)
                        time.sleep(4)
                        step = 9  # Move to step 5
                        break  # End the loop after second object is detected
            #if step == 9:

            time.sleep(0.1)


    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        GPIO.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

