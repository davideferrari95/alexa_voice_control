# alexa_voice_control

Package that allows communication between ros and alexa, implementing two different communication channels:

- User  →  ROS:  through a custom Alexa Skill that communicates with a local server.
- ROS   →  User: using unsolicited Text-To-Speech  from package [ha-alexa-tts](https://github.com/walthowd/ha-alexa-tts)

## Requirements

* Ubuntu 18.04+
* Python 3.6+
* ROS Melodic+
&nbsp; 
* flask-ask (Python): follow installation in the [flask-ask GitHub page](https://github.com/johnwheeler/flask-ask/blob/master/README.rst#installation).
(the use of pip3 instead of pip is recommended)
&nbsp;
* flask-ngrok (Python): follow installation in the [flask-ngrok GitHub page](https://github.com/gstaff/flask-ngrok).
(the use of pip3 instead of pip is recommended)

## Installation

* Navigate to source directory of your ROS catkin workspace (e.g. `catkin_ws`):

  ```
  cd catkin_ws/src
  copy the package (git clone or others)
  ```

* Build catkin workspace:

  ```
  cd catkin_ws
  catkin_make
  ```
  
* Source workspace:

  ```
  source catkin_ws/devel/setup.bash
  ```

### HTTPS-Tunnel Configuration (using ngrok)

In order for the Alexa requests to reach the local skill server, the local network must be configured to tunnel HTTPS traffic to a specific port on the local machine, using [ngrok](https://ngrok.com/).

1. Create a ngrok account and get your personal authtoken in [Getting Started](https://dashboard.ngrok.com/get-started/setup)

2. Set your authtoken (example daeIDBVp5SC8DE80uwrACm...) to ngrok:
 
     ```
    cd .../alexa_voice_control/ngrok
    ./ngrok authtoken daeIDBVp5SC8DE80uwrACm...
     ```

3. Edit the ngrok configuration file `.../config/ngrok.yml` by entering your authcode and region, as in the example below:

     ```
    authtoken: daeIDBVp5SC8DE80uwrACm...
    region: eu
     ```

### Text-To-Speech Configuration (using [ha-alexa-tts](https://github.com/walthowd/ha-alexa-tts))

This script does work by using the Alexa web interface, so it may break at anytime. However, the TTS component is POSTing a JSON object direct to the API endpoint, so the odds that TTS portion stays working are high.

1. Add your login credentials to the `.../script/secrets.yaml` file, with
the keys `alexa_email` and `alexa_password`, as in the example below:

     ```
    alexa_email: email@example.it
    alexa_password: password
     ```

1. Disable the **Amazon's Two-Step Verification**

2. Edit `.../script/alexa_remote_control.sh.template` file and set the desired language:

     ```
    Language Settings (default is Italian):
        LANGUAGE="it-IT" (line 60)
        AMAZON='amazon.it' (line 65)
        ALEXA='alexa.amazon.it' (line 70)
     ```

3. Edit `.../config/firefox.yaml` file and set your firefox profile directory. You can find the folder inside the path `/home/user/.mozilla/firefox/` (in this example the directory is `y66rd0ib.default-release`):

    ```
    firefox_profile: '/home/user/.mozilla/firefox/y66rd0ib.default-release'
    ```

4. Change the default name of the Echo Device in the `alexa_tts.launch` launchfile:

    ```
    <arg name="alexa_device_name" default="Echo Dot di USERNAME"/>
    ```

### Scripts Execution Configuration 

To end the setup you need to run the `setup.py` script, which will apply the configurations set previously.

* Set script file permissions to executable:

  ```
  chmod +x .../catkin_ws/src/alexa_voice_control/setup.py
  ```
  
* Execute the script:

  ```
  python3 .../catkin_ws/src/alexa_voice_control/setup.py
  ```

## Launch Instruction

* Use `roslaunch` to start `voice_control`:

  ```
  roslaunch alexa_voice_control voice_control.launch
  ```

* Or you can use `roslaunch` to launch individually `alexa_tts` and `skill_server`:

  ```
  roslaunch alexa_voice_control alexa_tts.launch
  roslaunch alexa_voice_control skill_server.launch
  ```
  
If there are no errors, a screen similar to this will be displayed on the terminal:

  ```
  ------------------------------------------------------------
  the following devices exist in your account:
  Echo Dot di Username
  Ovunque
  This Device
  ------------------------------------------------------------
  
  Alexa - TextToSpeech

  [INFO] [1595548174.476735]: TTS Message: Alexa-TTS-Communication-Initialized
  sending cmd:speak:Alexa-TTS-Communication-Initialized to dev:Echo Dot di Username type:A32*********** serial:G09************* customerid:AQX**********
  Sequence command: Alexa.Speak
  
  NGROK mode
  
    * Running on http://6a6a410be6e8.eu.ngrok.io
    * Running on https://6a6a410be6e8.eu.ngrok.io
  
    * Traffic stats available on http://127.0.0.1:4040
  ```

If login fails look at /tmp/.alexa.login. Search for "password" and see if you are being prompted for the captcha. If so, you can attempt to login to Alexa manually from a browser (from the same IP) and see if that fixes the issue for you. It never did for me, so I logged in to [https://alexa.amazon.it](https://alexa.amazon.it) with Firefox and used the [cookies.txt extension](https://addons.mozilla.org/it/firefox/addon/cookies-txt/) to export my amazon cookies. 

* Copy the cookies just obtained in  `.../config/.alexa.cookie`
* Delete all Amazon cookies from your browser

The export cookies extension is a manual mode so you need to disable the automatic cookies function inside the alexa_TTS script. 

* Disable the automatic cookies function in TTS scrips by setting `False` the flag `auto_get_cookies` in `alexa_TTS.launch` launchfile (set back to `True` to use the automatic mode)

### Endpoint Configuration (Alexa Skill)

To establish communication between the alexa skill and the local flask server it is necessary to connect the https address generated by ngrok with the endpoint of the skill.

* Open the [Amazon Developer Console](https://developer.amazon.com/alexa/console/ask) and navigate to your custom skill:

* Go in the `Build` tab, select `Endpoint` and check the option `HTTPS`.

* Paste the ngrok URL shown on the terminal (see above) in the `Default Region` box:

  ```
  Default Region   https://6a6a410be6e8.eu.ngrok.io
  ```

* Under `SSL Certificate` select "*My development endpoint is a sub-domain of a domain that has a wildcard certificate from a certificate authority.*"

* Save the Endpoint and Build the skill

If you use the free ngrok subscription the https domain changes with each server launch, therefore you need to update the skill endpoint every time. To avoid this, it is necessary to upgrade the ngrok account to the basic € 5 / month plan, obtaining a fixed domain.

## Authors

* **Davide Ferrari davideferr@unimore.it**
