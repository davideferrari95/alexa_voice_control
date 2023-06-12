import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import subprocess
import time

#{'object':[posizione, gia preso,prendere con la calamita]}
tools={
        'allenkey 8':[[0,0,0,0,0,0],False,True],
        'allenkey 6':[[0,0,0,0,0,0],False,True],
        'bottle':[[],False,False],
        'cross screwdriver':[[],False,True],
        'flat screwdriver':[[],False,True],
        'multifunction screwdriver':[[],False,True],
        'wrench':[[],False,True],
        'screw':[[],False,True]
        }

rospy.init_node('msg')



def main():
   
    print('--nodo inizializzato---asttesa messaggi--')
    rospy.spin()#aspetta all'infinito


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
       