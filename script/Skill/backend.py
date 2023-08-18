#!/usr/bin/env python3

import rospy, subprocess
import time, threading
from flask import Flask
from flask_ask import Ask, question, statement, session
from std_msgs.msg import String
from alexa_voice_control.msg import VoiceCommand

# Import Command Macros
from command_list import *

# Create Flask App
app = Flask(__name__)
ask = Ask(app, "/")

# Open ROS Skill Server in a Separate Thread
threading.Thread(target=lambda: rospy.init_node('skill_server', disable_signals=True)).start()

# Launch Node-RED in a New Process
NODE_RED = subprocess.Popen("node-red", shell=True)
time.sleep(2)
print()
rospy.logwarn('TTS Initialized')

# ROS Publishers
command_pub = rospy.Publisher('voice_command', VoiceCommand, queue_size=1)
tts_pub = rospy.Publisher('/tts', String, queue_size=1)
time.sleep(1)

def send_command(command, area=None):

    # Voice Command Message
    msg = VoiceCommand()
    msg.command = command
    msg.area = area if area is not None else ''
    command_pub.publish(msg)

rospy.logwarn('Alexa Skill Initialized')

@app.route("/")
def homepage():
    return "Server Alexa-ROS"

# LaunchIntent: "Open robot skill"
@ask.launch
def launch():

    # Welcome Message and Re-Prompt
    return question("")

@ask.intent('MovedTheObject')
def MovedTheObject():

    """ Alexa, I moved it """

    send_command(MOVED_OBJECT)
    return statement('ok, I resume the trajectory')

@ask.intent('PutObject')
def PutObject(area):

    """ Alexa, put it here """

    # Area Command if Defined
    if area is not None: send_command(PUT_OBJECT_IN_GIVEN_AREA, area)
    else: send_command(PUT_OBJECT_IN_AREA)
    return statement('ok')

@ask.intent('UserMoved')
def UserMoved():

    """ Alexa I moved back """

    send_command(USER_MOVED)
    return statement("ok, I'm approaching the area near to you")

@ask.intent('UserCantMove')
def UserCantMove():

    """ Alexa, I can't move """

    send_command(USER_CANT_MOVE)
    return question('ok, I can wait your command or replan the trajectory')

@ask.intent('ReplanTrajectory')
def ReplanTrajectory():

    """ Alexa, Replan the trajectory """

    send_command(REPLAN_TRAJECTORY)
    return statement('ok, I search for a new trajectory')

@ask.intent('CanGo')
def CanGo():

    """ Alexa, you can go """

    send_command(CAN_GO)
    return statement('ok, I resume the trajectory')

@ask.intent('AMAZON.StopIntent')
def AmazonStop():
    return statement('Arrivederci')

@ask.intent('AMAZON.HelpIntent')
def AmazonHelp():
    return question('')

# StopIntent: "Stop"
@ask.session_ended
def session_ended():
    return "{}", 200

if __name__ == '__main__':

    # Try Flask App Run -> Skill Back-End
    try: app.run()
    except rospy.ROSInterruptException: pass

    # Node-RED Wait
    NODE_RED.wait()
