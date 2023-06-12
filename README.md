# alexa_voice_control

Package that allows communication between ROS and Alexa, implementing two different communication channels:

- User &rarr; ROS: through a custom Alexa Conversation Skill that communicates with a local server in azure functions.
- ROS &rarr; User: using unsolicited Text-To-Speech of Node-RED block that use Alexa-API

## Requirements

- Ubuntu 20.04+
- Python 3.8+
- ROS Noetic+
- ROS bridge
- Node-RED
- ngrok

## Installation

- Install `miniconda` and create a custom environment:

        conda create -n alexa_env python=3.10
        conda activate alexa_env

- Launch `setup.py` for installing the requirements:

        python setup.py

### Node-RED

- Install `Node.js`:

        curl -sL https://deb.nodesource.com/setup_16.x -o /tmp/nodesource_setup.sh
        sudo bash /tmp/nodesource_setup.sh
        sudo apt install nodejs

- To install Node-RED you can use the `npm` command that comes with `node.js`:

        sudo apt install npm
        sudo npm install -g --unsafe-perm node-red

    In Node-RED &rarr; manage palet &rarr; install:

        node-red-contrib-alexa-remote2-applestrudel
        node-red-contrib-ros

    or in bash:

        npm install node-red-contrib-ros
        npm install node-red-contrib-alexa-remote2-applestrudel 

### ROS bridge

- Install rosbridge suite:

        sudo apt-get install ros-noetic-rosbridge-server

### Ngrok

- Install Ngrok
  
        sudo apt update
        sudo apt install snapd
        sudo snap install ngrok

- Add your Token in the `ngrok.yml` file in the `ngrok` folder

### Configuration Node-RED contrib ROS

- in sub/pub node &rarr; ROS SERVER &rarr; add new ros server:

        url: ws://localhost:9091/
        Topic: name_topic

### Configuration Node-RED contrib Alexa Remote 2

- in Alexa initialize node:

    Account &rarr; Add new Alexa Account:

        Name: Nome
        Auth Method: Proxy
        This IP: IP_ADDRESS
        Port : 3456
        File Path : AuthFile
        
        Service Host: alexa.amazon.it
        Page:amazon.it
        Language: it-IT

    Option &rarr; initialize

## Configure Alexa Skill End-Point

- change skill end-point with the ngrok one:

        https://2c13-151-38-117-194.eu.ngrok.io/api/HttpTriggerAlexaROS
