#!/usr/bin/env python3
import os, rospy
import time, threading

from flask import Flask
from flask_ngrok import run_with_ngrok
from flask_ask import Ask, question, statement, session
from std_msgs.msg import String

# Cache Destructor
import signal, sys, shutil, rospkg
def signal_handler(sig, frame):
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('alexa_voice_control')
    dirpath = package_path + '/src/Skill_Server/__pycache__'
    shutil.rmtree(dirpath, True)
    print('\nCache Deleted\n')
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


#---------------------------------------------- FLASK & ROS INIZIALIZATION ---------------------------------------------#

app = Flask(__name__)
ask = Ask(app, "/")
NGROK = True

# ROS node, publisher, and parameter. The node is started in a separate thread to avoid conflicts with Flask.
# The parameter *disable_signals* must be set if node is not initialized in the main thread.
threading.Thread(target=lambda: rospy.init_node('alexa_skill_server', disable_signals=True)).start()

intent_publisher = rospy.Publisher('/alexa/intents', String, queue_size=1)


#----------------------------------------------------- MAIN INTENTS ----------------------------------------------------#

@app.route("/")
def homepage():
    return "Server Alexa-ROS"

# LaunchIntent: "Open ros control"
@ask.launch
def launch():
    welcome_message = 'Welcome to the ROS Control node. How can I help you?'
    welcome_reprompt_message = 'How can I help you?'
    return question(welcome_message).reprompt(welcome_reprompt_message)

@ask.intent('AMAZON.StopIntent')
def AmazonStop():
    return statement('Goodbye')

@ask.intent('AMAZON.HelpIntent')
def AmazonHelp():
    return question('')

# StopIntent: "Stop"
@ask.session_ended
def session_ended():
    return "{}", 200


#--------------------------------------------------- CUSTOM INTENTS ----------------------------------------------------#

@ask.intent('StartIntent')
def StartTask():
    intent_name = 'Start'
    intent_publisher.publish(intent_name)
    return statement('Ok, I\'ll Start.')

@ask.intent('CustomIntent', default={'custom_variable':None})
def IniziaFunzione(custom_variable):
    intent_name = 'CustomIntent'
    intent_publisher.publish(intent_name)
    return question('Custom variable value is {0}'.format(custom_variable))


#-------------------------------------------------- NGROK HTTP TUNNEL --------------------------------------------------#

# ngrok app running
if __name__ == '__main__':
    if NGROK:
        # Start ngrok when app is run
        time.sleep(5) #time delay
        print("\nNGROK mode")
        run_with_ngrok(app)  
        app.run()
    else:
        print ('Manual tunneling mode')
        dirpath = os.path.dirname(__file__)
        cert_file = os.path.join(dirpath, '../config/ssl_keys/certificate.pem')
        pkey_file = os.path.join(dirpath, '../config/ssl_keys/private-key.pem')
        app.run(host=os.environ['ROS_IP'], port=5000,
                ssl_context=(cert_file, pkey_file))
