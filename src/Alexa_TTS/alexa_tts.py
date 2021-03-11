#!/usr/bin/env python3
import os, sys, subprocess
import rospy, rospkg
import shlex, time

from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import String as String_msg
from alexa_voice_control.srv import String as String_srv

# Define your Firefor profile name
firefox_profile_name = 'y66rd0ib.default-release'


#------------------------------------------ INIZIALIZATION -------------------------------------------#

rospy.init_node('alexa_tts_node')

# get script location path
rospack = rospkg.RosPack()
package_path = rospack.get_path('alexa_voice_control')
script_path = package_path + "/script/alexa_remote_control.sh"


#------------------------------------------ GET USER AGENT -------------------------------------------#

print("\nGetting User Agent ...\n", end='')

from selenium import webdriver
from selenium.webdriver.firefox.options import Options
import geckodriver_autoinstaller, fileinput

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

driver = webdriver.Firefox(options=options)
driver.get("https://www.whatsmyua.info/")
page_html = driver.page_source
user = driver.find_element_by_id('rawUa').text
user_agent_rawUa = user[7:]

driver.quit()

# copy and rename "alexa_remote_control.sh.template" in alexa_remote_control.sh
os.system("cp " + script_path + ".template " + script_path)

# print(user_agent_rawUa)
replaceAll(script_path, "user_agent_da_sostituire_con_script", user_agent_rawUa)
print("Done\n")


#------------------------------------------ DELETE COOKIES -------------------------------------------#

# print("Deleting Firefox Alexa Cookies ...")

import os
from os import path

'''Delete Amazon Cookies on Firefox'''
if (path.exists("~/.mozilla/firefox/*.default-release/cookies.sqlite")):
    os.system('sqlite3 ~/.mozilla/firefox/*.default-release/cookies.sqlite \\ \'delete FROM moz_cookies WHERE host LIKE "%amazon.%";\'')
    os.system('sqlite3 ~/.mozilla/firefox/*.default-release/cookies.sqlite \\ \'delete FROM moz_cookies WHERE host LIKE "%www.amazon.%";\'')

if (path.exists("~/.mozilla/firefox/*.default/cookies.sqlite")):
    os.system('sqlite3 ~/.mozilla/firefox/*.default/cookies.sqlite \\ \'delete FROM moz_cookies WHERE host LIKE "%amazon.%";\'')
    os.system('sqlite3 ~/.mozilla/firefox/*.default/cookies.sqlite \\ \'delete FROM moz_cookies WHERE host LIKE "%www.amazon.%";\'')

'''Delete Amazon Cookies on Chrome'''
if (path.exists("~/.config/google-chrome/Default/Cookies")):
    os.system('sqlite3 ~/.config/google-chrome/Default/Cookies \\ \'delete FROM cookies WHERE host_key LIKE "%amazon.%";\'')

# print("Cookies Deleted\n")


#----------------------------------------- GET ALEXA COOKIES -----------------------------------------#

print("Getting New Alexa Cookies ...\n", end='')

import http.cookiejar as cookielib

def to_cookielib_cookie(selenium_cookie):
    return cookielib.Cookie(
        version=0,
        name=selenium_cookie['name'],
        value=selenium_cookie['value'],
        port='80',
        port_specified=False,
        domain=selenium_cookie['domain'],
        domain_specified=True,
        domain_initial_dot=False,
        path=selenium_cookie['path'],
        path_specified=True,
        secure=selenium_cookie['secure'],
        expires=selenium_cookie['expiry'],
        discard=False,
        comment=None,
        comment_url=None,
        rest=None,
        rfc2109=False
    )
    
def put_cookies_in_jar(selenium_cookies, cookie_jar):
    for cookie in selenium_cookies:
        cookie_jar.set_cookie(to_cookielib_cookie(cookie))

firefox_profile_path = '~/.mozilla/firefox/' + firefox_profile_name
driver = webdriver.Firefox(firefox_profile=firefox_profile_path, options=options)
driver.get("https://alexa.amazon.it")
cookies = driver.get_cookies()
mcj = cookielib.MozillaCookieJar()

put_cookies_in_jar(cookies, mcj)
mcj.save(package_path + "/config/" + ".alexa.cookie")

print("Done\n\n", end='')


#---------------------------------------- DEVICE CONNNECTION -----------------------------------------#

# copy ".alexa.cookie" in /tmp
os.system("cp " + package_path + "/config/.alexa.cookie /tmp/.alexa.cookie")

# get the list of devices in my Amazon account
print("------------------------------------------------------------")
subprocess.call([script_path, '-a'])
time.sleep(1)
print("------------------------------------------------------------")

''' 
Devices list in your account:

Alexa - Cucina
Alexa - Camera
Alexa - Taverna
Ovunque
This Device
'''

try:
	# get the name of the alexa device I want to use from rosparam
	device_name = rospy.get_param("/alexa_tts_node/alexa_device_name")
except:
	device_name = "Echo Dot di ARSCONTROL"
	#device_name = "Alexa - Camera"

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

#------------------------------------------- TTS FUNCTIONS -------------------------------------------#

def tts_inizialization_callback (request):
    return TriggerResponse(success=True, message="TTS Initialized")

def alexa_tts (text = "testo-di-prova", alexa_device = "Alexa - Camera"):
	default_command = " -d \"" + alexa_device + "\" -e speak:"
	subprocess.call(shlex.split(script_path + default_command + "\"" + text + "\""))

def text_to_speech_callback (req):
	rospy.loginfo("TTS Message: %s", req.message_data)
	alexa_tts(req.message_data, device_name)
	return True

def tts_callback (data):
    rospy.loginfo("TTS Message: %s", data.data)
    alexa_tts(data.data, device_name)

#----------------------------------------------- MAIN ------------------------------------------------#

while not rospy.is_shutdown():
    
    text_to_speech_server = rospy.Service('/alexa/text_to_speech', String_srv, text_to_speech_callback)
    tts_inizialization_server = rospy.Service('/alexa/alexa_tts_initialization', Trigger, tts_inizialization_callback)
    rospy.Subscriber('/alexa/tts', String_msg, tts_callback)

    print("\nAlexa - TextToSpeech\n")
	
    rospy.spin()
