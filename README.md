# alexa_voice_control

Package that allows communication between ros and alexa, implementing two different communication channels:

- User  →  ROS:  through a custom Alexa Skill that communicates with a local server.
- ROS   →  User: using unsolicited Text-To-Speech  from package [ha-alexa-tts](https://github.com/walthowd/ha-alexa-tts)
 
## Requirements

* Ubuntu 18.04+
* Python 3.6+
* ROS Melodic+
&nbsp; 
* python3-catkin-pkg-modules, python3-rospkg-modules, python3-pip, sqlite3, jq
&nbsp;
    ``` bash
     sudo apt-get install python3-pip
     sudo apt-get install python3-catkin-pkg-modules
     sudo apt-get install python3-rospkg-modules
     sudo apt-get install sqlite3
     sudo apt-get install jq
    ```
* flask-ask (Python): follow installation in the [flask-ask GitHub page](https://github.com/johnwheeler/flask-ask/blob/master/README.rst#installation).
(the use of pip3 instead of pip is recommended)
&nbsp;
* flask-ngrok (Python): follow installation in the [flask-ngrok GitHub page](https://github.com/gstaff/flask-ngrok).
(the use of pip3 instead of pip is recommended)

## Installation

* Navigate to source directory of your ROS catkin workspace (e.g. `catkin_ws`):

  ``` bash
   cd catkin_ws/src
   copy the package
  ```

* Build catkin workspace:

  ``` bash
   cd catkin_ws
   catkin_make
  ```
  
* Source workspace:

  ``` bash
   source catkin_ws/devel/setup.bash
  ```

### HTTPS-Tunnel Configuration (using ngrok)

In order for the Alexa requests to reach the local skill server, the local network must be configured to tunnel HTTPS traffic to a specific port on the local machine, using [ngrok](https://ngrok.com/).

1. Create a ngrok account and get your personal authtoken in [Getting Started](https://dashboard.ngrok.com/get-started/setup)

2. Edit the ngrok configuration file `.../config/ngrok.yml` by entering your authcode and region, as in the example below:

     ``` bash
      authtoken: daeIDBVp5SC8DE80uwrACm...
      region: eu
     ```

### Text-To-Speech Configuration (using [ha-alexa-tts](https://github.com/walthowd/ha-alexa-tts))

This script does work by using the Alexa web interface, so it may break at anytime. However, the TTS component is POSTing a JSON object direct to the API endpoint, so the odds that TTS portion stays working are high.

1. Add your login credentials to the `.../script/secrets.yaml` file, with
the keys `alexa_email` and `alexa_password`, as in the example below:

     ``` bash
      alexa_email: email@example.it
      alexa_password: password
     ```

2. Disable the **Amazon's Two-Step Verification**

3. Get your browser [User Agent](https://www.whatismybrowser.com/detect/what-is-my-user-agent), you obtain a string similar to the example:

     ``` bash
      Mozilla/5.0 (X11; Ubuntu; Linux x86_64; rv:78.0) Gecko/20100101 Firefox/78.0
     ```

4. Edit `.../script/alexa_remote_control.sh` file:

     ``` bash
      Language Settings (default is Italian):
          LANGUAGE="it-IT" (line 60)
          AMAZON='amazon.it' (line 65)
          ALEXA='alexa.amazon.it' (line 70)
      
      Browser Identity:
          BROWSER='your_user_agent' (line 87)
     ```



### Scripts Execution Configuration 

To end the setup you need to run the `setup.py` script, which will apply the configurations set previously.

* Set script file permissions to executable:

  ``` bash
   chmod +x .../catkin_ws/src/alexa_voice_control/setup.py
  ```
  
* Execute the script:

  ``` bash
   python3 .../catkin_ws/src/alexa_voice_control/setup.py
  ```

## Launch Instruction

* Use `roslaunch` to start `voice_control`:

  ``` bash
   roslaunch roslaunch alexa_voice_control voice_control.launch
  ```

* Or you can use `roslaunch` to launch individually `alexa_tts` and `skill_server`:

  ``` bash
   roslaunch roslaunch alexa_voice_control alexa_tts.launch
   roslaunch roslaunch alexa_voice_control skill_server.launch
  ```
  
If there are no errors, a screen similar to this will be displayed on the terminal:

  ``` bash
    ------------------------------------------------------------
    the following devices exist in your account:
    Alexa - Camera
    Ovunque
    This Device
    ------------------------------------------------------------
    
    Alexa - TextToSpeech

    [INFO] [1595548174.476735]: TTS Message: Comunicazione-tra-robot-e-utente-inizializzata
    sending cmd:speak:Comunicazione-tra-robot-e-utente-inizializzata to dev:Alexa - Camera type:A32*********** serial:G09************* customerid:AQX**********
    Sequence command: Alexa.Speak
    
    NGROK mode
    
     * Running on http://6a6a410be6e8.eu.ngrok.io
     * Running on https://6a6a410be6e8.eu.ngrok.io
    
     * Traffic stats available on http://127.0.0.1:4040
  ```

If login fails look at /tmp/.alexa.login. Search for "password" and see if you are being prompted for the captcha. If so, you can attempt to login to Alexa manually from a browser (from the same IP) and see if that fixes the issue for you. It never did for me, so I logged in to https://alexa.amazon.it with Firefox and used the [cookies.txt extension](https://addons.mozilla.org/it/firefox/addon/cookies-txt/) to export my amazon cookies.

* Copy the cookies just obtained in  `.../config/.alexa.cookie`
* Delete all Amazon cookies from your browser

### Endpoint Configuration (Alexa Skill)

To establish communication between the alexa skill and the local flask server it is necessary to connect the https address generated by ngrok with the endpoint of the skill.

* Open the [Amazon Developer Console](https://developer.amazon.com/alexa/console/ask) and navigate to your custom skill:

* Go in the `Build` tab, select `Endpoint` and check the option `HTTPS`.

* Paste the ngrok URL shown on the terminal (see above) in the `Default Region` box:

  ``` bash
   Default Region   https://6a6a410be6e8.eu.ngrok.io
  ```

* Under `SSL Certificate` select "*My development endpoint is a sub-domain of a domain that has a wildcard certificate from a certificate authority.*"

* Save the Endpoint and Build the skill

If you use the free ngrok subscription the https domain changes with each server launch, therefore you need to update the skill endpoint every time. To avoid this, it is necessary to upgrade the ngrok account to the basic € 5 / month plan, obtaining a fixed domain.

## Authors

* **Davide Ferrari davideferr@unimore.it**
