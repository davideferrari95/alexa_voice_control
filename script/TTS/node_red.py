#!/usr/bin/env python3

import subprocess, time

# Import ROS2 Libraries
import rclpy
from std_msgs.msg import String

if __name__ == '__main__':

    # Initialize ROS Node
    rclpy.init()
    node = rclpy.create_node('node_red_tts')

    # Launch Node-RED in a New Process
    NODE_RED = subprocess.Popen("node-red", shell=True)
    time.sleep(2)
    print()
    node.get_logger().warn('TTS Initialized\n')

    # ROS Publisher
    tts_pub = node.create_publisher(String, '/tts', 1)
    time.sleep(1)

    try:

        # Wait Until ROS::OK()
        while rclpy.ok(): pass

    finally:

        # Node-RED Wait
        NODE_RED.wait()
