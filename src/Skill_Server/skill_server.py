#!/usr/bin/env python3
import os
import rospy
import threading
import requests
import time
import rospkg
import time

from flask import Flask
from flask_ngrok import run_with_ngrok
from flask_ask import Ask, question, statement, session
from std_msgs.msg import String, Float64, Int32


#---------------------------------------------- FLASK & ROS INIZIALIZATION ---------------------------------------------#

app = Flask(__name__)
ask = Ask(app, "/")
NGROK = True

# ROS node, publisher, and parameter. The node is started in a separate thread to avoid conflicts with Flask.
# The parameter *disable_signals* must be set if node is not initialized in the main thread.
threading.Thread(target=lambda: rospy.init_node('alexa_skill_server', disable_signals=True)).start()
intent_number_publisher = rospy.Publisher('/alexa/intent_number', Int32, queue_size=100)
# direction_publisher = rospy.Publisher('/alexa/direction', String, queue_size=100)
# length_publisher = rospy.Publisher('/alexa/lenght', Float64, queue_size=100)


@app.route("/")
def homepage():
    return "Server Alexa-ROS"


#------------------------------------------------------- INTENTS -------------------------------------------------------#

# LaunchIntent -> Avvio dell'applicazione: "Apri nodo ros"
@ask.launch
def launch():
    welcome_message = 'Benvenuto nel nodo di controllo ROS. Come posso aiutarti?'
    welcome_reprompt_message = 'Come posso aiutarti?'
    return question(welcome_message).reprompt(welcome_reprompt_message)

''' 
# MuoviRobotIntent: "Muovi il robot"
@ask.intent('MuoviRobotIntent', default={'direzione':None, 'lunghezza':None})
def move_robot_function(direzione, lunghezza):
    direction_publisher.publish(direzione)
    length_publisher.publish(float(lunghezza))
    return statement('Ho pubblicato direzione e lunghezza sul topic ROS: {0} di {1} metri.'.format(direzione, lunghezza)) 
'''

# MuoviRobotIntent: "Muovi il robot"
@ask.intent('MuoviRobotIntent')
def move_robot_function():
    intent_number = 1
    intent_number_publisher.publish(int(intent_number))
    return statement('Inizio il movimento') 

# StopIntent -> Uscita dall'applicazione: "Stop / Esci..."
@ask.session_ended
def session_ended():
    return "{}", 200


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
    
