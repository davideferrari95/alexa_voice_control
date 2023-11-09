#!/usr/bin/env python3

import rospy, rospkg, subprocess, sys
import time, threading
from flask import Flask
from flask_ask import Ask, question, statement, session
from std_msgs.msg import String
from alexa_voice_control.msg import VoiceCommand

# Import Parent Folders
sys.path.append(f'{rospkg.RosPack().get_path("alexa_voice_control")}/script/utils')

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
command_pub = rospy.Publisher('/alexa/voice_command', VoiceCommand, queue_size=1)
tts_pub = rospy.Publisher('/alexa/tts', String, queue_size=1)
time.sleep(1)

def send_command(command, object=None):

    # Voice Command Message
    msg = VoiceCommand()
    msg.command = command
    msg.info = command_info[command]
    msg.object = object if object is not None else ''
    command_pub.publish(msg)

rospy.logwarn('Alexa Skill Initialized')

@app.route("/")
def homepage():
    return "Server Alexa-ROS"

# LaunchIntent: "Open robot skill"
@ask.launch
def launch():

    # Welcome Message
    print('Welcome')
    return statement('welcome')

@ask.intent('BeginExperiment')
def BeginExperiment():

    """ Alexa, Start the experiment """

    print('BeginExperiment')
    send_command(BEGIN_EXPERIMENT)
    return statement('ok, I start the experiment')

@ask.intent('ProvideScrew')
def ProvideScrew():

    """ Alexa, Give me the Screws """

    print('ProvideScrew')
    send_command(PROVIDE_SCREW)
    return statement('ok, I bring you the screws')

@ask.intent('ProvideScrewdriver')
def ProvideScrewdriver():

    """ Alexa, Give me the Screwdriver """

    print('ProvideScrewdriver')
    send_command(PROVIDE_SCREWDRIVER)
    return statement('ok, I bring you the screwdriver')

@ask.intent('HoldObject')
def HoldObject():

    """ Alexa, Help me to hold the object """

    print('HoldObject')
    send_command(HOLD_OBJECT)
    return statement('ok, tell me when I can take it')

@ask.intent('TakeObject')
def TakeObject():

    """ Alexa, you can take the object """

    print('TakeObject')
    send_command(TAKE_OBJECT)
    return statement("ok, I'll take it")

@ask.intent('MoveMounting')
def MoveMounting():

    """ Alexa, move to the mounting position """

    print('MoveMounting')
    send_command(MOVE_MOUNTING)
    return statement("ok, I'll move to the mounting position")

@ask.intent('AMAZON.StopIntent')
def AmazonStop():
    return statement('Bye')

@ask.intent('AMAZON.HelpIntent')
def AmazonHelp():
    return statement('Help')

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
