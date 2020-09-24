#!/usr/bin/env python3
import atexit
import json
import os
import platform
import shutil
import subprocess
import tempfile
import time
import zipfile
from pathlib import Path
from threading import Timer

import requests
import rospkg


def _get_command():
    system = platform.system()
    if system == "Darwin":
        command = "ngrok"
    elif system == "Windows":
        command = "ngrok.exe"
    elif system == "Linux":
        command = "ngrok"
    else:
        raise Exception("{system} is not supported".format(system=system))
    return command


def _run_ngrok(port):
    
    # get IP Address
    ip_address = os.popen("ip a | grep \"scope global\" | grep -Po '(?<=inet )[\\d.]+'").read()
    os.environ["ROS_IP"] = ip_address
    #print("\nIP Address: " + ip_address + "ROS_IP: " + os.environ["ROS_IP"])

    # get script location path
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('alexa_voice_control')
    ngrok_path = package_path + "/ngrok/"
    
    command = _get_command()
    executable = str(Path(ngrok_path, command))
    os.chmod(executable, 0o777)

    ngrok = subprocess.Popen([executable, 'http', str(port)])
    atexit.register(ngrok.terminate)
    localhost_url = "http://localhost:4040/api/tunnels"  # Url with tunnel details
    #ROS_IP_url = "http://" + os.environ["ROS_IP"] + ":5000"
    time.sleep(1)
    tunnel_url = requests.get(localhost_url).text  # Get the tunnel information
    #tunnel_url = requests.get(ROS_IP_url).text  # Get the tunnel information
    j = json.loads(tunnel_url)

    tunnel_url = j['tunnels'][0]['public_url']  # Do the parsing of the get
    # tunnel_url = tunnel_url.replace("https", "http")
    return tunnel_url


def start_ngrok(port):
    ngrok_address = _run_ngrok(port)
    ngrok_address_https = ngrok_address
    ngrok_address_http = ngrok_address.replace("https", "http")
    print("\n * Running on " + ngrok_address_http)
    print(" * Running on " + ngrok_address_https)
    print("\n * Traffic stats available on http://127.0.0.1:4040\n")


def run_with_ngrok(app):
    """
    The provided Flask app will be securely exposed to the public internet via ngrok when run,
    and the its ngrok address will be printed to stdout
    :param app: a Flask application object
    :return: None
    """
    old_run = app.run

    def new_run(*args, **kwargs):
        port = kwargs.get('port', 5000)
        thread = Timer(1, start_ngrok, args=(port,))
        thread.setDaemon(True)
        thread.start()
        old_run(*args, **kwargs)

    app.run = new_run
