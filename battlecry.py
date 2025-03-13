import rclpy
from rclpy.node import Node
from irobot_create_msgs.msg import AudioNoteVector, AudioNote
from builtin_interfaces.msg import Duration
import time

def main():
    # Initialize ROS2
    rclpy.init()
    
    # Create a node
    node = rclpy.create_node('battle_cry_test')
    
    # Create publisher
    audio_publisher = node.create_publisher(AudioNoteVector, '/cmd_audio', 10)
    
    # Wait for publisher to be ready
    time.sleep(1)
    
    # Create audio message
    audio_msg = AudioNoteVector()
    audio_msg.append = False
    
    # Define the notes sequence
    notes = [
        (392, 177500000),  # G4
        (523, 355000000),  # C5
        (587, 177500000),  # D5
        (784, 533000000)   # G5
    ]
    
    # Add notes to the message
    audio_msg.notes = []
    for frequency, duration_ns in notes:
        note = AudioNote()
        note.frequency = frequency
        note.max_runtime = Duration(sec=0, nanosec=duration_ns)
        audio_msg.notes.append(note)
    
    print("Playing battle cry...")
    audio_publisher.publish(audio_msg)
    
    # Wait for the sound to finish
    time.sleep(2)
    
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()
    print("Test completed")

if __name__ == '__main__':
    main()