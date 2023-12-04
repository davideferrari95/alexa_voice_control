#!/usr/bin/env python3

# Import ROS2 Libraries
import rclpy, subprocess, time
from std_msgs.msg import String

# Initialize ROS Node
rclpy.init()
node = rclpy.create_node('node_red_tts')

# Launch Node-RED in a New Process
NODE_RED = subprocess.Popen("node-red -u ~/.node-red-2", shell=True)
time.sleep(2)
print()
node.get_logger().warn('TTS Initialized\n')

# ROS Publisher
tts_pub = node.create_publisher(String, '/alexa/tts', 1)
time.sleep(1)

if __name__ == '__main__':

    # Wait Until ROS::OK()
    while rclpy.ok(): pass

    # Node-RED Wait
    NODE_RED.wait()
