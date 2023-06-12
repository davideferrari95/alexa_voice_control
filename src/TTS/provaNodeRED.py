#!/usr/bin/env python3
#!/home/alberto/miniconda3/envs/davide_env/bin python3

#IMPORTANTE: installa ROSBRIDGE e prima di avviare quest script lancia il comando: roslaunch rosbridge_server rosbridge_websocket.launch port:=9091  e poi apri node_RED
#comando per pubblicare sul topic : rostopic pub /other std_msgs/String "data: 'prova'"

import rospy
from std_msgs.msg import String
import time

rospy.init_node('red_node')

ttsPub = rospy.Publisher('/tts',String,queue_size=1)
otherPub = rospy.Subscriber('/other',String,queue_size=1)



def main():

    time.sleep(1)
    print('--nodo inizializzato---attesa messaggi--')
    msg= rospy.wait_for_message('/other',String)

    if msg:
        print(msg.data)
        tts="Mannacc a quella grannd cess e mammet"
        ttsPub.publish(tts)
    else :
        raise Exception("Sorry, An exception occurred")      
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass