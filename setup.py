#!/usr/bin/env python3
import os, subprocess
from subprocess import STDOUT

proc = subprocess.Popen('sudo apt-get install -y python3-pip python3-catkin-pkg-modules python3-rospkg-modules sqlite3 jq', 
                        shell=True, stdin=None, stdout=open(os.devnull,"wb"), stderr=STDOUT, executable="/bin/bash")
proc.wait()

import rospkg

# Get script location path
rospack = rospkg.RosPack()
package_path = rospack.get_path('alexa_voice_control')
script_path = package_path + "/script/"
ngrok_path = package_path + "/ngrok/"
src_path = package_path + "/src/"
config_path = package_path + "/config/"

""" 
print("\nPackage Path: " + package_path)
print("Script Path: " + script_path)
print("src Path: " + src_path)
print("config Path: " + config_path)
print("ngrok Path: " + ngrok_path + "\n")
"""

# Set Permission to Executables
os.chmod(ngrok_path + "ngrok", 0o777)

os.chmod(script_path + "alexa_remote_control.sh.template", 0o777)
os.chmod(script_path + "alexa_wrapper.sh", 0o777)
os.chmod(script_path + "generate_ssl_cert.sh", 0o777)

os.chmod(src_path + "Alexa_TTS/alexa_tts.py", 0o777)
os.chmod(src_path + "Skill_Server/flask_ngrok.py", 0o777)
os.chmod(src_path + "Skill_Server/skill_server.py", 0o777)

# Create ngrok configuration files
os.system('mkdir ~/.ngrok2/')
os.system('cp ' + config_path + 'ngrok.yml ~/.ngrok2/ngrok.yml')

# Copy secrets.yaml in ~/.config/
# os.system('cp ' + script_path + 'secrets.yaml ~/.config/secrets.yaml')

# Install pip3 dependencies
os.system('pip3 install -r ' + package_path + '/requirements.txt')
