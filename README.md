# Alexa Voice Control

Package that allows communication between ROS2 and Alexa, implementing two different communication channels:

- User &rarr; ROS2: through a custom Alexa Skill that communicates with a local server.
- ROS2 &rarr; User: using unsolicited Text-To-Speech of Node-RED block that use Alexa-API

## Requirements

- Ubuntu 20.04+
- Python 3.8.10
- ROS2 Foxy
- Anaconda / Miniconda

## Installation

### Prerequisites

- Install ROS2 Foxy: [Ubuntu Guide](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)

        sudo apt install ros-foxy-desktop python3-argcomplete

- Install `miniconda`: [Official Guide](https://docs.conda.io/en/main/miniconda.html)

- Create a `conda` environment with `python=3.8.10`:

        conda create -n alexa_env python=3.8.10
        conda activate alexa_env

- Install Python Requirements:

        pip install -r ../path/to/this/repo/requirements.txt

### Node-RED

- Install `Node.js`:

        curl -sL https://deb.nodesource.com/setup_16.x -o /tmp/nodesource_setup.sh
        sudo bash /tmp/nodesource_setup.sh
        sudo apt install nodejs

- To install Node-RED you can use the `npm` command that comes with `node.js`:

        sudo apt install npm
        sudo npm install -g --unsafe-perm node-red

- Install `node-red-contrib-alexa-remote2-applestrudel` and `node-red-ros2-plugin`:

        node-red -u ~/.node-red-2
        cd ~/.node-red-2
        npm install node-red-contrib-alexa-remote2-applestrudel 
        npm install node-red-ros2-plugin

#### Configuration of `node-red-contrib-alexa-remote2-applestrudel`

- Add the `Alexa Init` Node to the Node-RED flow

- Edit the `Alexa Init` Node properties to add your Alexa Account:

  - Account &rarr; Add new Alexa Account &rarr; Edit (Pencil Icon):

  - Global Config:

        Name: <Custom_Name>
        Auth Method: Proxy
        This IP: localhost
        Port : 3456
        File Path : /home/<user>/AuthFile

  - For Italian language:

        Service Host: alexa.amazon.it
        Page:amazon.it
        Language: it-IT

  - For English language:

        Service Host: alexa.amazon.com
        Page:amazon.com
        Language: en-US

- Click on the `Add` button and follow the instructions to add your account (you will be redirected to the Amazon Alexa login page)

- Click on the `Deploy` button to save the configuration

#### Configuration of `node-red-ros2-plugin`

- Install dependencies:

        apt-get install libyaml-cpp-dev libboost-program-options-dev \
         libwebsocketpp-dev libboost-system-dev libboost-dev libssl-dev \
         libcurlpp-dev libasio-dev libcurl4-openssl-dev git

- Clone `Integration-Service` Packages: [Official Guide](https://github.com/eProsima/node-red-ros2-plugin)

        cd ~/colcon_ws/src
        mkdir Node-RED && cd "$_"
        git clone https://github.com/eProsima/Integration-Service.git
        git clone https://github.com/eProsima/WebSocket-SH.git
        git clone https://github.com/eProsima/ROS2-SH.git
        git clone https://github.com/eProsima/FIWARE-SH.git

- Build `Integration-Service` Packages:

        cd ~/colcon_ws
        source /opt/ros/foxy/setup.bash
        colcon build --cmake-args -DIS_ROS2_SH_MODE=DYNAMIC

- Source `Integration-Service` Packages:

        . ~/colcon_ws/install/setup.bash

- Add `export ROS_DOMAIN_ID=10` to `~/.bashrc` to set the Domain ID to `10`:

        echo "export ROS_DOMAIN_ID=10" >> ~/.bashrc

### Ngrok

- Install Ngrok

        sudo apt update
        sudo apt install snapd
        sudo snap install ngrok

- Add your Token in the `ngrok.yml` file in the `ngrok` folder

## Node-RED Flows Configuration

Example: Import `example_flows.json` files in the `scripts` folder to Node-RED to have a working example of the flows.

- Use the `Alexa Init` Node to add your Alexa Account
- Use the `Alexa Routine` Node to play voice messages

- Use the `ROS2 Type` Node linked to the `ROS2 Subscriber` Node to subscribe to ROS2 topics.
- Use the `ROS2 Inject` Node linked to the `ROS2 Type` and `ROS2 Publisher` Nodes to publish to ROS2 topics.
- Configure the `ROS2 Type` Node to match the message type.
- Configure the `ROS2 Subscriber` or `ROS2 Publisher` Node to match the topic name.
- Set the Domain ID of the `ROS2 Subscriber` or `ROS2 Publisher` Node to the same value of the ROS2 Bridge (Equal to `export ROS_DOMAIN_ID`).

## Launch Instructions

- Activate the `conda` environment:

        conda activate alexa_env

- Remember to source ROS2 and export the Domain ID (if not in `~/.bashrc`):

        source /opt/ros/foxy/setup.bash
        . ~/colcon_ws/install/setup.bash
        export ROS_DOMAIN_ID=10

### Configure Alexa Skill End-Point

- Launch ngrok:

        ngrok http 5000

- Change skill end-point with the ngrok one:

        https://2c92-79-42-218-6.ngrok-free.app

- Change the Endpoint Sub-domain:

        My development endpoint is a sub-domain of a domain that has a wildcard certificate from a certificate authority

### Launch Skill Back-End

- Launch Skill Backend + Node-RED:

        ros2 launch alexa_voice_control skill_backend.launch.py

- Launch only Node-RED:

        ros2 launch alexa_voice_control node_red.launch.py
