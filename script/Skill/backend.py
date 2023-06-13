#!/usr/bin/env python3
#!/home/davide/miniconda3/envs/test/bin python3

import rospy, subprocess
import time, threading
from flask import Flask
from flask_ask import Ask, question, statement, session
from std_msgs.msg import Int32MultiArray, String

# Create Flask App
app = Flask(__name__)
ask = Ask(app, "/")

# Open ROS Skill Server in a Separate Thread
threading.Thread(target=lambda: rospy.init_node('skill_server', disable_signals=True)).start()

# Launch Node-RED in a New Process
NODE_RED = subprocess.Popen("node-red", shell=True)
time.sleep(2)

# ROS Publishers
pub = rospy.Publisher('voice', Int32MultiArray, queue_size=1)
tts_pub = rospy.Publisher('/tts', String, queue_size=1)

# Custom Variables
msg = Int32MultiArray()
command = [0]

area_destra = {'sale': 'sale grosso', 
               'pasta' : 'pennette', 
               'pomodoro': 'passata' , 
               'coltello': 'coltello', 
               'forchetta': 'forchetta di legno',
               'bicchiere': 'bicchiere bianco'
                }

area_sinistra = {'sale': 'sale iodato', 
                 'pasta': 'spaghetti', 
                 'pomodoro': 'pelati', 
                 'forchetta':'forchettetta di plastica', 
                 'bicchiere': 'bicchiere trasparente'
                    }


@app.route("/")
def homepage():
    return "Server Alexa-ROS"

# LaunchIntent: "Open robot skill"
@ask.launch
def launch():
    welcome_message = 'Ciao! Benvenuto nel tutorial Casa Intelligente, io sono Robot Cop, l’obiettivo di questo esperimento è simulare un episodio di vita quotidiana in casa, dovrai apparecchiare la tavola e raccogliere gli ingredienti per preparare il pranzo insieme a me. Tutti gli oggetti sono disposti in tre aree, l’area di destra e l’area di sinistra saranno accessibili solo a me, l’area dove ti trovi sarà accessibile a entrambi. L’esperimento è diviso in due parti: l’apparecchiatura della tavola e la raccolta degli ingredienti per fare la tua pasta al pomodoro! A tua richiesta, ti passerò gli oggetti presenti nelle aree, ti elencherò gli oggetti presenti in una determinata area e ti dirò se un oggetto è presente o meno. Se hai dubbi su come apparecchiare la tavola o o preparare la ricetta, non esitare a farmi domande, sarò più che felice di aiutarti'

    welcome_reprompt_message = 'Cosa posso fare per te?'
    
    return question(welcome_message).reprompt(welcome_reprompt_message)


@ask.intent('PrendiOggetto')        
def takeitem(oggetto, area):
    

    print("Oggetto da prendere:", oggetto, "E area nominata", area)


    if oggetto == 'passata' or (oggetto == 'pomodoro' and area == 'destra'):
        command = [11]
        msg.data = command
        pub.publish(msg)
        return statement('ok')
    
    elif oggetto == 'pelati' or (oggetto == 'pomodoro' and area == 'sinistra'):
        command = [12]
        msg.data = command
        pub.publish(msg)
        return statement('ok')
    
    elif oggetto == 'pomodoro':
        command = [10]
        msg.data = command
        pub.publish(msg)
        return statement('ok')
         
    elif oggetto == 'sale grosso' or (oggetto == 'sale' and area == 'destra'):
        command = [21]
        msg.data = command
        pub.publish(msg)
        return statement('ok')
    
    elif oggetto == 'sale iodato' or (oggetto == 'sale' and area == 'sinistra'):
        command = [22]
        msg.data = command
        pub.publish(msg)
        return statement('ok')

    elif oggetto == 'sale':
        command = [20]
        msg.data = command
        pub.publish(msg)
        return statement('ok')  
     
    elif oggetto == 'pennette' or (oggetto == 'pasta' and area == 'destra'):
        command = [31]
        msg.data = command
        pub.publish(msg)
        return statement('ok')
    
    elif oggetto == 'spaghetti' or (oggetto == 'pasta' and area == 'sinistra'):
        command = [32]
        msg.data = command
        pub.publish(msg)
        return statement('ok')
    
    elif oggetto == 'pasta':
        command = [30]
        msg.data = command
        pub.publish(msg)
        return statement('ok')
    
    elif oggetto == 'forchetta di legno' or (oggetto == 'forchetta' and area == 'destra'):
        command = [41]
        msg.data = command
        pub.publish(msg)
        return statement('ok')
    
    elif oggetto == 'forchetta di plastica' or (oggetto == 'forchetta' and area == 'sinistra'):
        command = [42]
        msg.data = command
        pub.publish(msg)
        return statement('ok')
    
    elif oggetto == 'forchetta':
        command = [40]
        msg.data = command
        pub.publish(msg)
        return statement('ok')
        
    elif oggetto == 'coltello':
        command = [51]
        msg.data = command
        pub.publish(msg)
        return statement('ok')
    
    elif oggetto == 'bicchiere bianco' or (oggetto == 'bicchiere' and area == 'destra'):
        command = [61]
        msg.data = command
        pub.publish(msg)
        return statement('ok')
        
    elif oggetto == 'bicchiere trasparente' or (oggetto == 'bicchiere' and area == 'sinistra'):
        command = [62]
        msg.data = command
        pub.publish(msg)
        return statement('ok')
    
    elif oggetto == 'bicchiere':
        command = [60]
        msg.data = command
        pub.publish(msg)
        return statement('ok')
    else:
        return question('Non ho capito che oggetto vuoi prendere')
        

@ask.intent('qualiOggetti')
def which_items(oggettoGenerico, area, oggettoSpecifico):


    #correzione per il plurale
    if oggettoGenerico == "coltelli": oggettoGenerico = "coltello"
    if oggettoGenerico == "forchette": oggettoGenerico = "forchetta"
    if oggettoGenerico == "bicchieri":  oggettoGenerico = "bicchiere"

    print(oggettoGenerico,area, oggettoSpecifico)


    #dove posso prendere/dove si trova un determinato oggetto
    if oggettoSpecifico is not None:

        if oggettoSpecifico in area_destra.values():

            response = '{} si trova nell\' area di destra'.format(oggettoSpecifico)
            return statement(response)

        elif oggettoSpecifico in area_sinistra.values():

            response = '{} si trova nell\' area di sinistra'.format(oggettoSpecifico)
            return statement(response)
    
    #che oggetti ci sono in una determinata area
    if oggettoGenerico is None and oggettoSpecifico is None and area is not None:

        if area == 'area destra': area = 'destra'
        if area == 'area sinistra': area = 'sinistra'

        if area == 'destra':

            response = 'Nell\'area di destra ci sono: {}'.format(', '.join(area_destra.values()))
            return statement(response)
        
        elif area == 'sinistra':

            response = 'Nell\'area di sinistra ci sono: {}'.format(', '.join(area_sinistra.values()))
            return statement(response)
        
    #specificazione, che tipo di forchetta/coltello/bicchiere ci sono in una determinata area
    if oggettoGenerico is not None and area is not None and oggettoSpecifico is None:
    
        if oggettoGenerico == "forchetta" and area == 'destra':

            response = 'Nell\' area di destra ci sono: {}'.format(area_destra['forchetta'])
            return statement(response) 
        
        elif oggettoGenerico == "forchetta" and area == 'sinistra':

            response = 'Nell\' area di sinistra ci sono: {}'.format(area_sinistra['forchetta'])
            return statement(response)
        
        elif oggettoGenerico == "coltello" and area == 'area destra':

            response = 'Nell\' area di destra c\'è un coltello'
            return statement(response)
        
        elif oggettoGenerico == "coltello" and area == 'sinistra':

            response = 'Nell\' area di sinistra  non c\'è nessun coltello'
            return statement(response)
        
        elif oggettoGenerico == "bicchiere" and area == 'destra':

            response = 'Nell\' area di destra c\'è il {}'.format(area_destra['bicchiere'])
            return statement(response)
        
        elif oggettoGenerico == "bicchiere" and area == 'sinistra':

            response = 'Nell\' area di sinistra c\'è il {}'.format(area_sinistra['bicchiere'])
            return statement(response)
        
        elif oggettoGenerico == "spaghetti" and area == 'destra':

            response = 'Nell\' area di destra ci sono gli {}'.format(area_destra['pasta'])
            return statement(response)
        
        elif oggettoGenerico == "pasta" and area == 'sinistra':

            response = 'Nell\' area di sinistra ci sono gli {}'.format(area_sinistra['pasta'])
            return statement(response)
        
        elif oggettoGenerico == "pomodoro" and area == 'destra':

            response = 'Nell\' area di destra ci sono i {}'.format(area_destra['pomodoro'])
            return statement(response)
        
        elif oggettoGenerico == "pomodoro" and area == 'sinistra':

            response = 'Nell\' area di sinistra c\'è il {}'.format(area_sinistra['pomodoro'])
            return statement(response)
        
        elif oggettoGenerico == "sale" and area == 'destra':

            response = 'Nell\' area di destra c\'è il sale {}'.format(area_destra['sale'])
            return statement(response)
        
        elif oggettoGenerico == "sale" and area == 'sinistra':

            response = 'Nell\' area di sinistra c\'è il sale {}'.format(area_sinistra['sale'])
            return statement(response)
        
    #Che tipo di oggetto generico ci sono?
    if oggettoGenerico is not None and area is None and oggettoSpecifico is None: 

        if oggettoGenerico == 'pasta':

            response = 'Ci sono gli {} nell\'area di destra e {} nell\'area di sinistra'.format(area_destra['pasta'], area_sinistra['pasta'])
            return statement(response)
        
        elif oggettoGenerico == 'pomodoro':

            response = 'Ci sono i {} nell\'area di destra e {} nell\'area di sinistra'.format(area_destra['pomodoro'], area_sinistra['pomodoro'])
            return statement(response)
        
        elif oggettoGenerico == 'sale':

            response = 'C\'è il {} nell\'area di destra e {} nell\'area di sinistra'.format(area_destra['sale'], area_sinistra['sale'])
            return statement(response)
        
        elif oggettoGenerico == 'forchetta':

            response = 'C\'è la {} nell\'area di destra e  la {} nell\'area di sinistra'.format(area_destra['forchetta'], area_sinistra['forchetta'])
            return statement(response)
        
        elif oggettoGenerico == 'bicchiere':

            response = 'Ci sono i {} nell\'area di destra e {} nell\'area di sinistra'.format(area_destra['bicchiere'], area_sinistra['bicchiere'])
            return statement(response)
        
        elif oggettoGenerico == 'coltello':

            response = 'C\'è un coltello nell\'area di destra e non c\'è nessun coltello nell\'area di sinistra'
            return statement(response)
        
        else:
                
            response = 'Non ho capito, quale oggetto vuoi conoscere?'
            return question(response)
        
    else:
        return question('Non ho capito, puoi ripetere?')
        

@ask.intent("chiediPassi")
def askStep(step):

    if step == None:

        response = 'L\'esperimento consiste nel raccogliere tutti gli oggetti necessari per apparecchiare una tavola e cucinare una pasta al pomodoro, puoi chiedermi cosa devi fare per cucinare o apparecchiare'
        return statement(response)
    
    elif step == "cucinare" or step == "raccogliere ingredienti":

        response = 'Per cucinare una pasta al pomodoro avrai bisogno di pasta, pomdoro e sale'
        return statement(response)
    
    elif step == "apparecchiare":

        response = 'Per apparecchiare è necessario ottenere un coltello, una forchetta e un bicchiere'
        return statement(response)


@ask.intent("fineEsperimento")
def endExperiment():

    response = 'Complimenti! Hai terminato l\'esperimento Casa Intelligente! Spero tu ti sia divertito e ti sia stato utile!'
    return statement(response)


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

if __name__ == '__main__':

    try:
        app.run()
    except rospy.ROSInterruptException:
        pass

NODE_RED.wait()
