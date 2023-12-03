#!/usr/bin/env python3

import rclpy, time
import subprocess, sys

from flask import Flask
from flask_ask import Ask, question, statement, session
from std_msgs.msg import String
from alexa_voice_control.msg import VoiceCommand
from pathlib import Path

# Import Parent Folders
sys.path.append(str(Path(__file__).resolve().parents[1] / "utils"))

# Import Command Macros
from command_list import *

# Create Flask App
app = Flask(__name__)
ask = Ask(app, "/")

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


    # Open ROS Skill Server in a Separate Thread
    rclpy.init()
    node = rclpy.create_node('skill_server')

    # Launch Node-RED in a New Process
    NODE_RED = subprocess.Popen("node-red", shell=True)
    time.sleep(2)
    print()
    node.get_logger().warn('TTS Initialized')

    # ROS Publishers
    command_pub = node.create_publisher(VoiceCommand, '/alexa/voice_command', 1)
    tts_pub = node.create_publisher(String, '/alexa/tts', 1)
    time.sleep(1)

    def send_command(command, object=None):

        # Voice Command Message
        msg = VoiceCommand()
        msg.command = command
        msg.info = command_info[command]
        msg.object = object if object is not None else ''
        command_pub.publish(msg)

    node.get_logger().warn('Alexa Skill Initialized')

    # Try Flask App Run -> Skill Back-End
    try: app.run()
    except: pass

    # Node-RED Wait
    NODE_RED.wait()
