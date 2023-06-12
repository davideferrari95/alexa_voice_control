#!/usr/bin/env python3
from operator import truediv
import rospy
from std_msgs.msg import String,Bool, Int32MultiArray
from geometry_msgs.msg import Pose
#from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from ur_rtde_controller.msg import CartesianPoint
from ur_rtde_controller.srv import RobotiQGripperControl, RobotiQGripperControlRequest
from ur_rtde_controller.srv import GetForwardKinematic, GetForwardKinematicRequest
import numpy as np
# import subprocess
import time
# import os


"""

TTS Utils:

IPv4 Unimore: 155.185.125.201

"""

class Robot:

    def __init__(self):

        rospy.init_node('expManager')

        #init variabili
        self.gripper_aperto = 100
        self.gripper_chiuso = 0
        self.gripper_70 = 70
        self.voice_msg = None

        self.defaultPos=[2.531209945678711, -1.8816501102843226, 1.7585914770709437, -1.4168628019145508, 4.700905799865723, 0.7452919483184814]
        self.placePos=[1.4981036186218262, -1.8520351848998011, 2.422215286885397, -2.14617981533193, 4.746311187744141, -0.03622609773744756]
        self.intermediatePos = [2.8941173553466797, -1.249845342045166, 1.0492914358722132, -1.3367853921702881, 4.713375091552734, 1.1106750965118408]


        self.oggetti={
            
            
            "il sale grosso":[
            [2.5016584396362305, -0.8632076543620606, 1.2751339117633265, -1.963386674920553, 4.741418838500977, 0.9712827205657959],
            self.gripper_aperto,
            self.gripper_chiuso,
            ],

            "il sale iodato":[
            [3.387843608856201, -1.0682671827128907, 1.6490314642535608, -2.1364666424193324, 4.772143363952637, 1.8173348903656006],
            self.gripper_aperto,
            self.gripper_chiuso,
            ],

            "il bicchiere bianco":[
            [2.3846733570098877, -1.0724294942668458, 1.6509855429278772, -2.178251882592672, 4.730436325073242, 0.85353684425354],
            self.gripper_chiuso,
            self.gripper_aperto,
            ],

            "la passata":[
            [2.3381829261779785, -0.8568876546672364, 1.3088882605182093, -2.0508209667601527, 4.731287956237793, 0.8080871105194092],
            self.gripper_chiuso,
            self.gripper_aperto,
            ], 

            "il coltello":[
            [2.2267518043518066, -1.1784547132304688, 2.059087578450338, -2.477241178552145, 4.735124588012695, 0.6946213245391846],
            self.gripper_aperto,
            self.gripper_chiuso,
            ], 

            "la forchetta di legno":[
            [2.293628692626953, -1.0718673032573243, 1.85292894044985, -2.3308073482909144, 4.7392730712890625, -0.8873665968524378],
            self.gripper_aperto,
            self.gripper_chiuso,
            ], 

            "le pennette":[
            [2.1489624977111816, -1.074343041782715, 1.6118515173541468, -2.0838972530760707, 4.7358078956604, -1.0315120855914515],
            self.gripper_aperto,
            self.gripper_chiuso,
            ], 

            "la forchetta di plastica":[
            [3.3018717765808105, -1.2413643163493653, 2.0898597876178187, -2.3902908764281214, 4.729753017425537, -1.270754639302389],
            self.gripper_aperto,
            self.gripper_chiuso,
            ], 

            "i pelati":[
            [3.3865585327148438, -1.4877928060344239, 2.2749279181109827, -2.3304363689818324, 4.7321391105651855, -1.1869919935809534],
            self.gripper_chiuso,
            self.gripper_aperto,
            ], 

            "il bicchiere trasparente":[
            [3.4324843883514404, -1.2404654783061524, 1.9565780798541468, -2.2602154217162074, 4.73328971862793, -1.139892880116598],
            self.gripper_chiuso,
            self.gripper_aperto,
            ],

            "gli spaghetti":[
            [3.595914840698242, -1.1588075918010254, 1.9754837195025843, -2.4072934589781703, 4.74080753326416, 0.44310879707336426],
            self.gripper_aperto,
            self.gripper_chiuso,
            ]
                    }

        #init oggetto posizione in giunti(se time_from_start=0 allora si muove in velocità)
        self.destinationPos=JointTrajectoryPoint()
        self.destinationPos.time_from_start = rospy.Duration(0)
        self.destinationPos.velocities = [0.4]

        #Publisher per i movimenti in posa e in cartesiano 
        self.ur10Pub=rospy.Publisher('/ur_rtde/controllers/joint_space_controller/command',JointTrajectoryPoint,queue_size=10)
        self.ur10PubCartesian=rospy.Publisher('/ur_rtde/controllers/cartesian_space_controller/command',CartesianPoint,queue_size=10)

        #Andata e Ritorno di Alexa
        rospy.Subscriber('voice', Int32MultiArray, self.voiceCallback)
        self.ttsPub = rospy.Publisher('/tts',String,queue_size=1)


        #init service per il gripper
        self.gripper_srv = rospy.ServiceProxy('/ur_rtde/robotiq_gripper/command', RobotiQGripperControl)
        self.gripper_req = RobotiQGripperControlRequest()

        #init service per cinematica inversa 
        self.cartesian_srv = rospy.ServiceProxy('ur_rtde/getFK', GetForwardKinematic)


        #In totale ci sono 16 oggetti
        self.oggettiGenerici = {'il pomodoro': 10, 
                                    'il sale': 20, 
                                    'la pasta': 30,
                                    'la forchetta': 40,
                                    'il bicchiere': 60,
                                }
        
        self.oggettiSpecifici = {'la passata': 11,
                                    'i pelati': 12, 
                                    'il sale grosso': 21,
                                    'il sale iodato': 22,
                                    'le pennette': 31,
                                    'gli spaghetti': 32,
                                    'la forchetta di plastica': 42,
                                    'la forchetta di legno': 41,
                                    'il coltello': 51,
                                    'il bicchiere bianco': 61,
                                    'il bicchiere trasparente': 62,
                                }
        
    #callback for alexa
    def voiceCallback(self,data):

       self.voice_msg = data.data 
       print("\nComando vocale {} ricevuto ".format(self.voice_msg,))
       self.voiceCheck = True

    #function for close/open gripper
    def gripping(self,position):

        # self.gripper_req.position, self.gripper_req.speed, self.gripper_req.force = 100, 100, 25
        self.gripper_req.position = position
        rospy.wait_for_service('/ur_rtde/robotiq_gripper/command')
        self.gripper_response = self.gripper_srv(self.gripper_req)
        '''if gripper_response.status != "2":
        raise Exception("Sorry, An exception occurred")'''

    def handover(self,position,start_grip, end_grip, tts):

        #posizione iniziale
        self.gripping(start_grip)

        #vado a mettermi nella home position
        self.intermidatePosition()

        #chiamata servise cinemarica inversa
        cartesian_req= GetForwardKinematicRequest()
        cartesian_req.joint_position=position
        rospy.wait_for_service('ur_rtde/getFK')
        cartesian_response = self.cartesian_srv(cartesian_req)
        print(cartesian_response)

        #init posizione cartesiana
        cartesian_pose = CartesianPoint()

        #alzo la z di 40cm
        cartesian_response.tcp_position.position.z+=0.40
        cartesian_pose.cartesian_pose = cartesian_response.tcp_position

        #velocita di salita e discesa in cartesiano
        cartesian_pose.velocity = 0.2

        #pubblico come prima posizione la posizione in cartesiano alzata di 40 cm 
        self.ur10PubCartesian.publish(cartesian_pose)
        flag = rospy.wait_for_message('/ur_rtde/trajectory_executed', Bool)
        if flag.data is not True:
            raise Exception("Sorry, An exception occurred") 
        #time.sleep(1)


        #destinazione in giunti dell'oggetto
        self.destinationPos.positions=position
        self.ur10Pub.publish(self.destinationPos)
        flag = rospy.wait_for_message('/ur_rtde/trajectory_executed', Bool)
        if flag.data is not True:
            raise Exception("Sorry, An exception occurred") 
        time.sleep(1)

        #chiusura/ apertura del gripper
        self.gripping(end_grip)

        #ripubblico la posizione alzata di 40 cm in cartesiano
        self.ur10PubCartesian.publish(cartesian_pose)
        flag = rospy.wait_for_message('/ur_rtde/trajectory_executed', Bool)
        if flag.data is not True:
            raise Exception("Sorry, An exception occurred") 
        time.sleep(0.5)

        self.intermidatePosition()

        #chiamata servise cinemarica inversa
        cartesian_req= GetForwardKinematicRequest()
        cartesian_req.joint_position= self.placePos
        rospy.wait_for_service('ur_rtde/getFK')
        cartesian_response = self.cartesian_srv(cartesian_req)
        print(cartesian_response)

        #init posizione cartesiana
        cartesian_pose = CartesianPoint()

        #alzo la z di 40cm
        cartesian_response.tcp_position.position.z+=0.40
        cartesian_pose.cartesian_pose = cartesian_response.tcp_position

        #velocita di salita e discesa in cartesiano
        cartesian_pose.velocity = 0.4

        #pubblico come prima posizione la posizione in cartesiano alzata di 40 cm 
        self.ur10PubCartesian.publish(cartesian_pose)
        flag = rospy.wait_for_message('/ur_rtde/trajectory_executed', Bool)
        if flag.data is not True:
            raise Exception("Sorry, An exception occurred") 
        time.sleep(1)


        self.destinationPos.positions=self.placePos
        self.ur10Pub.publish(self.destinationPos)
        flag = rospy.wait_for_message('/ur_rtde/trajectory_executed', Bool)
        if flag.data is not True:
            raise Exception("Sorry, An exception occurred")
        
        self.gripping(start_grip)

        #chiamata servise cinemarica inversa
        cartesian_req= GetForwardKinematicRequest()
        cartesian_req.joint_position= self.placePos
        rospy.wait_for_service('ur_rtde/getFK')
        cartesian_response = self.cartesian_srv(cartesian_req)
        print(cartesian_response)

        #init posizione cartesiana
        cartesian_pose = CartesianPoint()

        #alzo la z di 40cm
        cartesian_response.tcp_position.position.z+=0.20
        cartesian_pose.cartesian_pose = cartesian_response.tcp_position

        #velocita di salita e discesa in cartesiano
        cartesian_pose.velocity = 0.4

        #pubblico come prima posizione la posizione in cartesiano alzata di 40 cm 
        self.ur10PubCartesian.publish(cartesian_pose)
        flag = rospy.wait_for_message('/ur_rtde/trajectory_executed', Bool)
        if flag.data is not True:
            raise Exception("Sorry, An exception occurred")

        #Risposta con Nodered
        self.ttsPub.publish(tts)
        time.sleep(1)

        self.returningPosition()

    def returningPosition(self):

        self.destinationPos.positions=self.defaultPos
        self.destinationPos.velocities = [0.8]
        self.ur10Pub.publish(self.destinationPos)
        flag = rospy.wait_for_message('/ur_rtde/trajectory_executed', Bool)
        if flag.data is not True:
            raise Exception("Sorry, An exception occurred")
        
    def intermidatePosition(self):

        self.destinationPos.positions=self.intermediatePos
        self.destinationPos.velocities = [0.7]
        self.ur10Pub.publish(self.destinationPos)
        flag = rospy.wait_for_message('/ur_rtde/trajectory_executed', Bool)
        if flag.data is not True:
            raise Exception("Sorry, An exception occurred")
        
    def checkExistace(self):

        if self.voice_msg[0] in self.oggettiGenerici.values():

            self.ttsPub.publish("Mi dispiace, c'è più di un oggetto che corrisponde alla tua descrizione, sii più preciso, ti ricordo che puoi sempre chiedermi di dirti quali sono gli oggetti presenti o per tipo o per area")
            self.voice_msg = None
            print("Generico")

        
    def experiment(self):

        if self.voice_msg is not None:

            #vedo se gli oggetti sono dichiarati in maniera precisa
            self.checkExistace()

            for key, value in self.oggettiSpecifici.items():

                if self.voice_msg is not None:

                    if self.voice_msg[0] == value :

                        tts_iniziale = "Ok, vado a prenderti {}".format(key)

                        self.ttsPub.publish(tts_iniziale)

                        tts_finale = "Ecco {}".format(key)

                        self.handover(self.oggetti[key][0],self.oggetti[key][1], self.oggetti[key][2], tts_finale)

                        self.voice_msg = None

if __name__ == '__main__':


    R = Robot()
    try:
            
        time.sleep(0.5)
        print('--Nodo Inizializzato--')
        R.returningPosition() 

        while not rospy.is_shutdown():

            R.experiment()

    except rospy.ROSInterruptException:
        R.returningPosition()
        print("Nodo interrotto manualmente")
        pass
