#!/usr/bin/env python3
import os
import subprocess
import shlex
import rospkg

# get script location path
rospack = rospkg.RosPack()
package_path = rospack.get_path('alexa_voice_control')
skill_server_path = package_path + "/src/Skill_Server/"
executable = skill_server_path + "skill_server.py"
# print(executable)
os.chmod(executable, 0o777)
# print('python3 ' + executable)
os.system('python3 ' + executable)