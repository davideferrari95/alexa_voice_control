#!/usr/bin/env python3
import os, subprocess
from subprocess import STDOUT

proc = subprocess.Popen('sudo apt-get install -y python3-pip python3-catkin-pkg-modules python3-rospkg-modules', 
                        shell=True, stdin=None, stdout=open(os.devnull,"wb"), stderr=STDOUT, executable="/bin/bash")
proc.wait()

import rospkg

# Get script location path
rospack = rospkg.RosPack()
package_path = rospack.get_path('alexa_voice_control')

# Set Permission to Executables
os.chmod(package_path + "/ngrok/ngrok", 0o777)

os.chmod(package_path + "/script/Skill/backend.py", 0o777)
os.chmod(package_path + "/script/TTS/node_red.py", 0o777)
os.chmod(package_path + "/script/Exp_Manager.py", 0o777)

# Create ngrok configuration files
os.system('mkdir ~/.ngrok2/')
os.system('cp ' + package_path + '/ngrok/ngrok.yml ~/.ngrok2/ngrok.yml')

# Install pip3 dependencies
os.system('pip3 install -r ' + package_path + '/requirements.txt')
