#!/usr/bin/env python3

import rospy, subprocess, time
from std_msgs.msg import String

# Open ROS Skill Server in a Separate Thread
rospy.init_node('node_red_tts', disable_signals=True)

# Launch Node-RED in a New Process
NODE_RED = subprocess.Popen("node-red", shell=True)
time.sleep(2)
print()
rospy.logwarn('TTS Initialized\n')

# ROS Publishers
tts_pub = rospy.Publisher('/tts', String, queue_size=1)
time.sleep(1)

if __name__ == '__main__':

    # Wait Until ROS::OK()
    while not rospy.is_shutdown(): pass

    # Node-RED Wait
    NODE_RED.wait()
