# 1. How I created the packages **"servo_interfaces"** and **"voice_assistant_pkg"**

The instructions in this section are only for bookkeeping how I created the packages. Follow section [2](#2-how-to-build-and-run-the-packages) to build and run packages.
## 1.1. Create **"servo_interfaces"** package to host RunServo.srv

I followed [ros2 tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html) to make custom RunServo.srv
```bash
source /opt/ros/humble/setup.bash 
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake servo_interfaces
cd servo_interfaces
mkdir srv
# Add RunServo.srv file in srv folder
# Make changes to CMakeLists.txt and package.xml as explained in the ros2 tutorial

# Build servo_interfaces pkg 
cd ~/ros2_ws
colcon build --packages-select servo_interfaces

# In a new terminal, check if ros recognizes the new srv
cd ~/ros2_ws
source /opt/ros/humble/setup.bash 
source install/setup.bash
ros2 interface show servo_interfaces/srv/RunServo

```

## 1.2. Create voice_assistant_pkg package

1. Contents of `voice_assistant.py`: Voice assistant code is borrowed from [here](https://github.com/JarodMica/ChatGPT-and-Whiper-with-TTS/blob/main/voice_assistant.py) and "RunServoClient" class' code is adapted from [ros2 tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html#write-the-client-node)
2. Create ros2 package
```bash
source /opt/ros/humble/setup.bash 
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python voice_assistant_pkg --dependencies rclpy servo_interfaces
# Add voice_assistant.py in ros2_ws/src/voice_assistant_pkg/voice_assistant_pkg/ folder
# Follow ros2 tutorial to change package.xml and setup.py
```


# 2. How to build and run the packages

## 2.1. Set up Virtual Environment to install extra python packages

```bash
# Create python virtual env
cd ~/ros2_ws/src
mkdir -p colcon_venv/src
cd colcon_venv
virtualenv -p python3 ./venv --system-site-packages --symlinks
source ./venv/bin/activate

# Add this so that colcon does not build colcon_venv
touch ./venv/COLCON_IGNORE

# Install required packages
pip install openai
pip install SpeechRecognition
pip install pyttsx3
sudo apt install espeak
sudo apt-get update -y
sudo apt-get install -y portaudio19-dev
pip install PyAudio
```

In `setup.cfg` in `voice_assistant_pkg` package, add the following lines if they do not exist:
```
[build_scripts]
executable = /usr/bin/env python3
```

**Note:** I could not get conda env to work with ROS2. There were some python path conflicts between conda and ROS2, as a result I would get ModuleNotFound Errors e.g. catkin_pkg not found. To hide conda paths from ROS2, I had to comment out conda related stuff from bashrc and rename my /home/barza/anaconda3 folder so that ROS2 does not use python from anaconda's lib folder. If I have to use conda for non-ROS applications, I will have to undo these changes, which is messy. If you know of a cleaner/better way of hiding conda from ROS2 or using conda with ROS2 without conflicts/errors, please let me know. Thanks!

**Tip (VSCode):** I installed ROS (by Microsoft) and CMake (by twxs) extensions to help in autocompletion of ROS2 code. Run `code .` in `ros2_ws/src` folder to open VSCode and choose the python interpreter from colcon_venv.

## 2.2. Build package
```bash
cd ~/ros2_ws

# Activate virtual env
source src/colcon_venv/venv/bin/activate

# Source ros2
source /opt/ros/humble/setup.bash 

# Build packages
colcon build --packages-select servo_interfaces voice_assistant_pkg

```

## 2.3. Share **servo_interfaces** pkg with microros_ws and Arduino
1. Follow the instructions in [MicroRos_ArduinoDue_Setup_Guide](.MicroRos_ArduinoDue_Setup_Guide.md) to install and setup microros with Arduino Due.
2. After building `servo_interfaces` package in ros2_ws, copy `servo_interfaces` package folder in `microros_ws/src/` folder. In a new terminal, build servo_interfaces in microros_ws again (not sure if this step is necessary. TODO: Verify and update README): 
   ```bash
   source /opt/ros/humble/setup.bash 
   cd microros_ws/
   colcon build --packages-select servo_interfaces

   # Check if ROS2 recognizes servo_interfaces
   source install/setup.bash
   ros2 interface show servo_interfaces/srv/RunServo 
   ```
3. After building `servo_interfaces` package in ros2_ws, copy `servo_interfaces` package folder in `/home/barza/Arduino/libraries/micro_ros_arduino-2.0.5-humble/extras/library_generation/extra_packages/` folder.
   
4. Build micro_ros_arduino library to include our custom servo_interface package:
   ```bash
   cd ~/Arduino/libraries/micro_ros_arduino-2.0.5-humble/

   sudo docker pull microros/micro_ros_static_library_builder:humble

   sudo docker run -it --rm -v $(pwd):/project --env MICROROS_LIBRARY_FOLDER=extras microros/micro_ros_static_library_builder:humble

   ```
5. If build was successful, you should be able to find `servo_interfaces/RunServo.srv` in ` ~/Arduino/libraries/micro_ros_arduino-2.0.5-humble/available_ros2_types` 
   
6. Copy `micro-ros_runservo_service` folder from `TRAILBot/Arduino/` to ` ~/Arduino/`. Open Arduino ide,  compile and upload `micro-ros_runservo_service.ino` in Arduino Due.


## 2.4. Run packages

1. Power Cycle/Reconnect Arduino Due to laptop via USB.

2. Open a new terminal and run microros agent with --dev (port connected to Arduino):

   ```bash
   cd ~/microros_ws

   # Source ros2
   source /opt/ros/humble/setup.bash 

   # Source local packages 
   source install/setup.bash

   # Run microros agent
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -v6
   ```

3. **Optional:** Open a new terminal and call `/runservo` service to see if you can operate servos:
   ```bash
   ros2 service call /runservo servo_interfaces/srv/RunServo {servo: 1}
   ros2 service call /runservo servo_interfaces/srv/RunServo {servo: 2}
   ros2 service call /runservo servo_interfaces/srv/RunServo {servo: 3}
   ros2 service call /runservo servo_interfaces/srv/RunServo {servo: 4}
   ```

4. Open a new terminal and run chatbot
   ```bash
   cd ~/ros2_ws

   # Activate virtual env
   source src/colcon_venv/venv/bin/activate

   # Source ros2
   source /opt/ros/humble/setup.bash 

   # Source local packages 
   source install/setup.bash

   # Run node
   ros2 run voice_assistant_pkg chatbot
   ```

## 2.5. Troubleshoot

1. Make sure you are in a quiet place.
2. If error message includes the following assertion failed:
   ```
    assert source.stream is not None, "Audio source must be entered before adjusting, see documentation for ``AudioSource``; are you using ``source`` outside of a ``with`` statement?"
   ```
   Uncomment line `# print(sr.Microphone.list_microphone_names())` to list all microphones connected to your laptop. In your sound settings you can check which microphone your laptop is using/is active. Set the device_index to the one corresponding to the active microphone or keep changing until the program is able to listen to you. Make sure the volume in the sound settings for your mic is high.
3. Always start speaking after you see `Listening…` in the output. Don’t speak when it says `Ambient noise adjust…`.
4. If ambient noise is too high, it won’t pick up your voice. You can tune the “r.energy_threshold” according to the ambient noise. If it is noisy around you, set it to a high value. If it is quiet, set it to a low value. It’s value ranges from 50-5000. It will only pick up your voice if its signal’s amplitude exceeds this threshold.
5. Use my openai key provided in the script. It is paid to use by Steve. They will charge us based on the number of words (also called tokens) present in our input sentence to chatgpt and the number of words it replies with. The pricing is $0.002 / 1K tokens for gpt 3.5 turbo we are using.
6. To end conversation with chatgpt, say the word 'Bye'.


# ros2 service type /snack_wanted
# ros2 interface show trailbot_interfaces/srv/SnackWanted
#ros2 service call /snack_wanted trailbot_interfaces/srv/SnackWanted "{snack: 'chips'}"
ros2 topic pub --once /state std_msgs/msg/String "{data: "Query"}"