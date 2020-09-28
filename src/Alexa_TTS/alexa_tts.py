#!/usr/bin/env python3
import os
import rospy
import rospkg
import subprocess
import shlex
import time

from std_msgs.msg import String, Float64, Bool

from selenium import webdriver
from selenium.webdriver.firefox.options import Options
import geckodriver_autoinstaller
import fileinput
import sys


#------------------------------------------ INIZIALIZATION -------------------------------------------#

rospy.init_node('alexa_tts_node')
# r = rospy.Rate(10) # 10hz

inizialization_publisher = rospy.Publisher('/alexa/alexa_tts_initialization', Bool, queue_size=100)
# text_publisher = rospy.Publisher('alexa/text_to_speech/received', String, queue_size=100)

# get script location path
rospack = rospkg.RosPack()
package_path = rospack.get_path('alexa_voice_control')
script_path = package_path + "/script/alexa_remote_control.sh"
#script_path = "~/davide_ws/src/Speech_Recogn/Alexa_TTS/script/alexa_remote_control.sh"

# print("\nPackage Path: " + package_path)
# print("Script Path: " + script_path + "\n")


#------------------------------------------ GET USER AGENT -------------------------------------------#

print("\nGetting User Agent\n")

def replaceAll(file,searchExp,replaceExp):
    for line in fileinput.input(file, inplace=1):
        if searchExp in line:
            line = line.replace(searchExp,replaceExp)
        sys.stdout.write(line)


geckodriver_autoinstaller.install()  # Check if the current version of geckodriver exists
                                     # and if it doesn't exist, download it automatically,
                                     # then add geckodriver to path


options = Options()
options.headless = True
options.add_argument("--window-size=1920,1200")

url = "https://www.whatsmyua.info/"

driver = webdriver.Firefox(options=options)
driver.get(url)
page_html = driver.page_source
user = driver.find_element_by_id('rawUa').text
user_agent_rawUa = user[7:]

driver.quit()

# print(user_agent_rawUa)

# copy and rename "alexa_remote_control.sh.template" in alexa_remote_control.sh
os.system("cp " + script_path + ".template " + script_path)

replaceAll(script_path, "user_agent_da_sostituire_con_script", user_agent_rawUa)


#---------------------------------------- DEVICE CONNNECTION -----------------------------------------#

# launch delete_cookies script
print("\nLaunching Delete Cookies Script\n")
os.system('python3 ' + package_path + '/src/Alexa_TTS/delete_cookies.py')

# copy ".alexa.cookie" in /tmp
os.system("cp " + package_path + "/config/.alexa.cookie /tmp/.alexa.cookie")

# get the list of devices in my Amazon account
print("------------------------------------------------------------")
subprocess.call([script_path, '-a'])
time.sleep(1)
# os.system(script_path + " -a")
print("------------------------------------------------------------")

''' 
Devices list in your account:

Alexa - Cucina
Alexa - Camera
Alexa - Taverna
Ovunque
Davide's alexa-client
This Device
Davide's Alexa Apps

'''

# set name of the alexa device I want to use
#device_name = "Alexa - Camera"
device_name = "Echo Dot di ARSCONTROL"


'''
Script Usage:

    -d <device>|ALL] -e <pause|play|next|prev|fwd|rwd|shuffle|repeat|vol:<0-100>> |
	    -b [list|<\"AA:BB:CC:DD:EE:FF\">] | -q | -r <\"station name\"|stationid> |
	    -s <trackID|'Artist' 'Album'> | -t <ASIN> | -u <seedID> | -v <queueID> | -w <playlistId> |
	    -i | -p | -P | -S | -a | -m <multiroom_device> [device_1 .. device_X] | -lastalexa | -l | -h

	-e : run command, additional SEQUENCECMDs: weather,traffic,flashbriefing,goodmorning,singasong,tellstory,
         speak:'<text>',automation:'<routine name>'

	-b : connect/disconnect/list bluetooth device
	-q : query queue
	-r : play tunein radio
	-s : play library track/library album
	-t : play Prime playlist
	-u : play Prime station
	-v : play Prime historical queue
	-w : play library playlist
	-i : list imported library tracks
	-p : list purchased library tracks
	-P : list Prime playlists
	-S : list Prime stations
	-a : list available devices
	-m : delete multiroom and/or create new multiroom containing devices
	-lastalexa : print device that received the last voice command
	-l : logoff
	-h : help

'''


#--------------------------------------------- FUNCTIONS ---------------------------------------------#

def alexa_tts (text = "testo-di-prova", alexa_device = "Alexa - Camera"):

	default_command = " -d \"" + alexa_device + "\" -e speak:"
	subprocess.call(shlex.split(script_path + default_command + "\"" + text + "\"")) 
	# print(script_path + default_command + "\"" + text + "\"")
	# os.system(script_path + default_command + "\"" + text + "\"")


def text_callback(data):
    rospy.loginfo("TTS Message: %s",data.data)
    alexa_tts(data.data, device_name)


#----------------------------------------------- MAIN ------------------------------------------------#

while not rospy.is_shutdown():

	text_subscriber = rospy.Subscriber('/alexa/text_to_speech', String, text_callback)
	
	inizialization_publisher.publish(True)
	
	print("\nAlexa - TextToSpeech\n")
	# text_publisher.publish("Alexa - TextToSpeech Initialized")
    # alexa_tts("Comunicazione-tra-robot-e-utente-inizializzata")
	
	rospy.spin()
