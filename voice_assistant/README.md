<!---# 1. How I created the packages **"trailbot_interfaces"** and **"voice_assistant"**

The instructions in this section are only for bookkeeping how I created the packages. Follow section [2](#2-how-to-build-and-run-the-packages) to build and run packages.
## 1.1. Create **"trailbot_interfaces"** package to host RunServo.srv

I followed [ros2 tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html) to make custom RunServo.srv
```bash
source /opt/ros/humble/setup.bash 
cd ~/trail_ws/src
ros2 pkg create --build-type ament_cmake trailbot_interfaces
cd trailbot_interfaces
mkdir srv
# Add RunServo.srv file in srv folder
# Make changes to CMakeLists.txt and package.xml as explained in the ros2 tutorial

# Build trailbot_interfaces pkg 
cd ~/trail_ws
colcon build --packages-select trailbot_interfaces

# In a new terminal, check if ros recognizes the new srv
cd ~/trail_ws
source /opt/ros/humble/setup.bash 
source install/setup.bash
ros2 interface show trailbot_interfaces/srv/RunServo

```


## 1.2. Create voice_assistant package

1. Contents of `voice_assistant.py`: Voice assistant code is borrowed from [here](https://github.com/JarodMica/ChatGPT-and-Whiper-with-TTS/blob/main/voice_assistant.py) and "RunServoClient" class' code is adapted from [ros2 tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html#write-the-client-node)
2. Create ros2 package
```bash
source /opt/ros/humble/setup.bash 
cd ~/trail_ws/src
ros2 pkg create --build-type ament_python voice_assistant --dependencies rclpy trailbot_interfaces
# Add voice_assistant.py in trail_ws/src/voice_assistant/voice_assistant/ folder
# Follow ros2 tutorial to change package.xml and setup.py
```
-->

# 1. How to build and run the packages

## 1.1. [Optional] Set up Virtual Environment to install extra python packages

```bash
# Create python virtual env
cd ~/trail_ws/src
mkdir -p colcon_venv/src
cd colcon_venv
virtualenv -p python3 ./venv --system-site-packages --symlinks

# Activate virtual environment
source ./venv/bin/activate

# Add this so that colcon does not build colcon_venv
touch ./venv/COLCON_IGNORE

```

In `setup.cfg` in `voice_assistant` package, add the following lines if they do not exist:
```
[build_scripts]
executable = /usr/bin/env python3
```

**Note:** I could not get conda env to work with ROS2. There were some python path conflicts between conda and ROS2, as a result I would get ModuleNotFound Errors e.g. catkin_pkg not found. To hide conda paths from ROS2, I had to comment out conda related stuff from bashrc and rename my /home/barza/anaconda3 folder so that ROS2 does not use python from anaconda's lib folder. If I have to use conda for non-ROS applications, I will have to undo these changes, which is messy. If you know of a cleaner/better way of hiding conda from ROS2 or using conda with ROS2 without conflicts/errors, please let me know. Thanks!

**Tip (VSCode):** I installed ROS (by Microsoft) and CMake (by twxs) extensions to help in autocompletion of ROS2 code. Run `code .` in `trail_ws/src` folder to open VSCode and choose the python interpreter from colcon_venv.

## 1.2. Install python packages

```bash
# [Optional] Activate virtual env
source ~/trail_ws/src/colcon_venv/venv/bin/activate

# Install required packages
sudo apt install espeak
sudo apt-get update -y
sudo apt-get install -y portaudio19-dev ffmpeg
pip install -r ~/trail_ws/src/TRAILBot/voice_assistant/requirements.txt
```


## 1.3. Build package
```bash
cd ~/trail_ws

# [Optional] Activate virtual env
source src/colcon_venv/venv/bin/activate

# Source ros2
source /opt/ros/humble/setup.bash 

# Build packages
colcon build --packages-select trailbot_interfaces voice_assistant

```

## 1.4. Export API keys
```bash
echo 'export OPENAI_API_KEY="<secret api key, ask Barza>"' >> ~/.bashrc
echo 'export ELEVENLABS_API_KEY="<secret api key, (free per email account and for one month, just sign up at elevenlabs)>"' >> ~/.bashrc
```

## 1.5. Run packages

1. Open a new terminal and run voice_assistant_node
   ```bash
   cd ~/trail_ws

   # [Optional] Activate virtual env
   source src/colcon_venv/venv/bin/activate

   # Source ros2
   source /opt/ros/humble/setup.bash 

   # Source local packages 
   source install/setup.bash

   # Run voice_assistant
   ros2 launch voice_assistant voice_assistant.launch.py

   ```
2. Chatbot starts interacting with the user if the robot's state changes to 'QueryState'. Without behaviour planner running, you can manually change robot's state to 'QueryState' to test/activate chatbot.
   ```bash
   ros2 topic pub --once /state std_msgs/msg/String "{data: "QueryState"}"
   ```

## 1.6. Update Snack Inventory

In a new terminal you can update snack inventory from commandline while nodes are running

```bash
ros2 param set /voice_arduino_bridge_node snack_options '['chips', 'juice', 'candies', 'chocolate']'

ros2 param set /voice_arduino_bridge_node snack_quantity '[3, 1, 5, 6]'
```

## 1.7. Running with Aliases (shortcuts)

### 1.7.1. Setup Aliases in bashrc

1. gedit ~/.bashrc
2. Add the following lines:
   ```bash
   alias voice="ros2 launch voice_assistant voice_assistant.launch.py"
   alias query="ros2 topic pub --once /state std_msgs/msg/String '{data: "QueryState"}'"
   alias run_agent="ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -v6"
   alias run_s1="ros2 service call /runservo trailbot_interfaces/srv/RunServo '{servo: 1}'"
   alias run_s2="ros2 service call /runservo trailbot_interfaces/srv/RunServo '{servo: 2}'"
   alias run_s3="ros2 service call /runservo trailbot_interfaces/srv/RunServo '{servo: 3}'"
   alias run_s4="ros2 service call /runservo trailbot_interfaces/srv/RunServo '{servo: 4}'"
   alias arduino="~/Downloads/arduino-ide_2.1.0_Linux_64bit/arduino-ide"
   ```

### 1.7.2. Test voice interaction and servos
Steps to test voice interaction and servos:

1. Terminal 1: run_agent
2. Connect arduino’s usb wire to laptop
3. Terminal 2: voice
4. Terminal 3: query
5. Chat with the robot once you see “Listening…” on voice assistant node output

**Note:** If you see Audio errors on voice assistant node, you will have to set the mic index in params.yaml to the active index. 

### 1.7.3. Test servos only
If you want to only test servos:

1. Terminal 1: run_agent
2. Connect arduino’s usb wire to laptop
3. Terminal 2: `run_s<servo number>`

### 1.7.4. Test voice interaction only (without Arduino)

1. Change `test_voice_only` param in `voice_assistant/config/params.yaml` to True
2. Terminal 1: voice

## 1.8. Calibrating motor rotation
While calibrating motors, you will need to reupload the new arduino code in the arduino.

1. Make sure you stop microros agent!

2. Terminal: arduino (alias for starting arduino ide)
3. Tune the ‘delay’ parameter for each servo so that the tip of the coil rotates 360 degrees.
4. Upload code to arduino
5. Terminal: run_agent
6. Plug out and plug in arduino USB
7. Terminal: run_s1 (run servo to inspect if the servo completes 360 degree rotation) 


## 1.9. Troubleshoot

1. Make sure you are in a quiet place.
2. If error message includes the following assertion failed:
   ```
    assert source.stream is not None, "Audio source must be entered before adjusting, see documentation for ``AudioSource``; are you using ``source`` outside of a ``with`` statement?"
   ```
   Before this error message, the node should print a list of all microphones connected to your laptop. In your sound settings you can check which microphone your laptop is using/is active. Set the `mic_device_index` in `config/params.yaml` to the one corresponding to the active microphone or keep changing until the program is able to listen to you. Make sure the volume in the sound settings for your mic is high.
3. Always start speaking after you see `Listening…` in the output. Don’t speak when it says `Ambient noise adjust…`.
4. If ambient noise is too high, it won’t pick up your voice. You can tune the “energy_threshold” in `params.yaml` according to the ambient noise. If it is noisy around you, set it to a high value. If it is quiet, set it to a low value. It’s value ranges from 50-5000. It will only pick up your voice if its signal’s amplitude exceeds this threshold.
5. The OpenAI and ElevenLabs API keys are paid for use by Steve. OpenAI will charge us based on the number of words (also called tokens) present in our input sentence to chatgpt and the number of words it replies with. The pricing is $0.002 / 1K tokens for gpt 3.5 turbo we are using.
6. To end conversation with chatgpt, say the word 'Bye' or any of the words in `exit_cmd_options`.

<!--
[Optional]: To test snack_wanted_service node:
ros2 service type /snack_wanted
ros2 interface show trailbot_interfaces/srv/SnackWanted
ros2 service call /snack_wanted trailbot_interfaces/srv/SnackWanted "{snack: 'chips'}"
-->

