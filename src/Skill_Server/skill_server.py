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
from alexa_voice_control.msg import parameter_msg, movement_msg


#---------------------------------------------- FLASK & ROS INIZIALIZATION ---------------------------------------------#

app = Flask(__name__)
ask = Ask(app, "/")
NGROK = True

# ROS node, publisher, and parameter. The node is started in a separate thread to avoid conflicts with Flask.
# The parameter *disable_signals* must be set if node is not initialized in the main thread.
threading.Thread(target=lambda: rospy.init_node('alexa_skill_server', disable_signals=True)).start()

intent_number_publisher = rospy.Publisher('/alexa/intent_number', Int32, queue_size=100)

function_publisher = rospy.Publisher('/alexa/function_request', Int32, queue_size=100)
# nome_funzione + comando (avvio, stop,pausa...)

direction_publisher = rospy.Publisher('/alexa/move_robot/direction', movement_msg, queue_size=100)
position_publisher = rospy.Publisher('/alexa/move_robot/position', String, queue_size=100)
parameter_publisher = rospy.Publisher('/alexa/set_parameter', parameter_msg, queue_size=100)


@app.route("/")
def homepage():
    return "Server Alexa-ROS"


#----------------------------------------------------- MAIN INTENTS ----------------------------------------------------#

# LaunchIntent -> Avvio dell'applicazione: "Apri nodo ros"
@ask.launch
def launch():
    welcome_message = 'Benvenuto nel nodo di controllo ROS. Come posso aiutarti?'
    welcome_reprompt_message = 'Come posso aiutarti?'
    return question(welcome_message).reprompt(welcome_reprompt_message)

@ask.intent('AMAZON.StopIntent')
def AmazonStop():
    return statement('Arrivederci')

@ask.intent('AMAZON.HelpIntent')
def AmazonHelp():
    return question('Le funzioni disponibili sono pulizia e prelievo')

# StopIntent -> Uscita dall'applicazione: "Stop / Esci..."
@ask.session_ended
def session_ended():
    return "{}", 200

@ask.intent('StandbyIntent')
def Standby():
    return statement('Standby confermato')

''' 
# MuoviRobotIntent: "Muovi il robot"
@ask.intent('MuoviRobotIntent', default={'direzione':None, 'lunghezza':None})
def move_robot_function(direzione, lunghezza):
    direction_publisher.publish(direzione)
    length_publisher.publish(float(lunghezza))
    return statement('Ho pubblicato direzione e lunghezza sul topic ROS: {0} di {1} metri.'.format(direzione, lunghezza)) 
'''

#--------------------------------------------------- FUNCTION INTENTS --------------------------------------------------#

funzione_in_esecuzione = 'nessuna funzione in esecuzione'
funzione_in_pausa = 'nessuna funzione in pausa'
funzione_annullata = 'nessuna funzione annullata'
#callback funzione terminata?
#callback funzione annullata?

# IniziaFunzioneIntent: "Inizia la Funzione"
@ask.intent('IniziaFunzioneIntent', default={'funzione':None})
def IniziaFunzione(funzione):
    global funzione_in_esecuzione
    intent_number = 1
    intent_number_publisher.publish(int(intent_number))

    if funzione == 'prelievo':
        function_publisher.publish(1)
        funzione_in_esecuzione = 'prelievo'
        return question('Inizio il {0}'.format(funzione_in_esecuzione)) 
    elif funzione == 'pulizia':
        function_publisher.publish(2)
        funzione_in_esecuzione = 'pulizia'
        return question('Inizio la funzione di {0}'.format(funzione_in_esecuzione)) 
    else:
        return question('La funzione {0} non è disponibile'.format(funzione))


# StopFunzioneIntent: "Ferma la Funzione"
@ask.intent('StopFunzioneIntent')
def FermaFunzione():
    global funzione_in_esecuzione
    if funzione_in_esecuzione != 'nessuna funzione in esecuzione':
        intent_number = 2
        intent_number_publisher.publish(int(intent_number))
        return question('Ho interrotto la funzione {0}'.format(funzione_in_esecuzione))
    else:
        return question('Nessuna funzione in esecuzione')

# PausaFunzioneIntent: "Pausa la Funzione"
@ask.intent('PausaFunzioneIntent')
def PausaFunzione():
    global funzione_in_esecuzione
    global funzione_in_pausa
    if funzione_in_esecuzione != 'nessuna funzione in esecuzione':
        intent_number = 3
        intent_number_publisher.publish(int(intent_number))
        funzione_in_pausa = funzione_in_esecuzione
        return question('Ho messo in pausa la funzione {0}'.format(funzione_in_pausa))
    else:
        return question('Nessuna funzione in esecuzione')

# RiprendiFunzioneIntent: "Riprendi la Funzione"
@ask.intent('RiprendiFunzioneIntent')
def RiprendiFunzione():
    global funzione_in_pausa
    if funzione_in_pausa != 'nessuna funzione in pausa':
        intent_number = 4
        intent_number_publisher.publish(int(intent_number))
        f = funzione_in_pausa
        funzione_in_pausa = 'nessuna funzione in pausa'
        return question('Ho ripreso la funzione {0}'.format(f))
    else:
        return question('Nessuna funzione in pausa')

# RiprovaFunzioneIntent: "Riprova la Funzione"
@ask.intent('RiprovaFunzioneIntent')
def RiprovaFunzione():
    global funzione_annullata
    if funzione_annullata != 'nessuna funzione annullata':
        intent_number = 5
        intent_number_publisher.publish(int(intent_number))
        f = funzione_annullata
        funzione_annullata = 'nessuna funzione annullata'
        return question('Riprovo la funzione {0}'.format(f))
    else:
        return question('Nessuna funzione annullata')


#------------------------------------------------ ROBOT MOVEMENT INTENTS -----------------------------------------------#


@ask.intent('SpostaRobotIntent', default={'lunghezza':None, 'direzione':None})
def SpostaRobot(lunghezza,direzione):
    intent_number = 10
    intent_number_publisher.publish(int(intent_number))
    move = movement_msg()
    move.direction.data = direzione
    move.lenght = float(lunghezza)
    direction_publisher.publish(move)
    return question('Ho mosso il robot di {0} centimetri verso {1}'.format(lunghezza,direzione))

@ask.intent('MoveToPositionIntent', default={'posizione':None})
def MoveToPosition(posizione):
    intent_number = 11
    intent_number_publisher.publish(int(intent_number))
    position_publisher.publish(posizione)
    return question('Inizio il movimento verso la posizione {0}'.format(posizione))

@ask.intent('ImpostaParametroIntent', default={'parametro':None, 'valore':None})
def ImpostaParametro(parametro, valore):
    intent_number = 12
    intent_number_publisher.publish(int(intent_number))

    par = parameter_msg()
    par.parameter_value = float(valore)
    if parametro == "velocità":
        par.parameter_name.data = "velocity" 
    parameter_publisher.publish(par)
    return statement('Invio la richiesta')

@ask.intent('AskInformationIntent', default={'parametro':None})
def AskInformation(parametro):
    intent_number = 13
    intent_number_publisher.publish(int(intent_number))
    parameter_publisher.publish(parametro)
    valore = 'valore di prova'  # get parameter value (request-response)
    return question('Il valore di {0} è {1}'.format(parametro,valore))

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
