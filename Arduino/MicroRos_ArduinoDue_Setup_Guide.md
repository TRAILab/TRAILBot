# 1. MicroROS
The following content is borrowed from [here](https://roboticsknowledgebase.com/wiki/interfacing/microros-for-ros2-on-microcontrollers/#installation-overview). For more information on microROS, checkout [micro-ros official website](https://micro.ros.org/docs/tutorials/programming_rcl_rclc/overview/).

## 1.1. Introduction
* Micro-ROS node/application: A ros node that runs in microcontroller (Micro-ROS Client libraries- specific to the microcontroller hardware- used to develop micro-ROS application/node)
* Micro-ROS agent runs on your host laptop - allows communication between micro-ROS node and ROS2 stack (i.e. other ros nodes running in your laptop). It enables micro-ROS node to publish and subscribe like any other ROS node.

![microros_img](.images/../images/micro-ros.png)

## 1.2. Installation:
This document assumes you have ROS2 installed on your host machine/laptop. If ROS2 is not installed, please follow the ROS2 installation guide for official instructions (debian packages). Once you have a ROS 2 installation in the computer, follow these steps to install the micro-ROS build system:

```bash
# In a new terminal, source the ROS 2 installation
source /opt/ros/$ROS_DISTRO/setup.bash

# Create a workspace and download the micro-ROS tools
mkdir microros_ws
cd microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Update dependencies using rosdep
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y

# Install pip
sudo apt-get install python3-pip

# Build micro-ROS tools and source them
colcon build
source install/local_setup.bash

# Create firmware  
ros2 run micro_ros_setup create_firmware_ws.sh host

# Build firmware 
ros2 run micro_ros_setup build_firmware.sh
source install/local_setup.bash

# Download micro-ROS-Agent packages
ros2 run micro_ros_setup create_agent_ws.sh

# Build step
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash

# Run a micro-ROS agent
source /opt/ros/humble/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial –dev /dev/ttyACM0 -v6
```

## 1.3. Setting up MicroROS with Arduino Due

1. First, it is necessary to have the Arduino IDE installed, with the Board Manager configured for use with the Arduino Due board. You can confirm this setup by referencing the [Arduino Due Quickstart Guide](https://docs.arduino.cc/hardware/due).
2. The precompiled micro-ROS library can be found on the [releases page](https://github.com/micro-ROS/micro_ros_arduino/releases) of the micro-ROS GitHub repository. Download the ZIP file for the precompiled library correponding to your version of ROS 2 (e.g. humble). Then from within the Arduino IDE, you can include the library in your application by navigating to Sketch -> Include Library -> Add .ZIP Library.
3. For most officially supported boards for micro-ROS, the precompiled library may be all you need to get started. At this time of writing, however, the Arduino Due is a community-supported board which requires an additional patch. To install the patch, run the following commands:
    ```bash
    export ARDUINO_PATH=~/.arduino15/packages/arduino
    cd $ARDUINO_PATH/hardware/sam/1.6.12/
    curl https://raw.githubusercontent.com/micro-ROS/micro_ros_arduino/humble/extras/patching_boards/platform_arduinocore_sam.txt > platform.txt
    ```

The `ARDUINO_PATH` referenced above is typically located at one of the following paths:
On GNU/Linux: `~/.arduino15/packages/arduino`

4. After applying the patch, open the Arduino IDE (or if it is already open, close and re-open it). From here you should be able to open a micro-ROS example, e.g. File -> Examples -> micro_ros_arduino -> micro-ros_publisher. Verify that it compiles, and this would conclude the micro-ROS precompiled library installation.


## 1.4. Testing the Installation

1. First, we will test the installation of the precompiled micro-ROS libraries for the microcontroller. Open the Arduino IDE and navigate to `File -> Examples -> micro_ros_arduino -> micro-ros_publisher` to open up an example sketch. Connect your microcontroller to your computer and upload the sketch. If the installation was properly completed, this should compile and upload successfully.

2. Next, we can initiate the micro-ROS agent to verify installation of the micro-ROS libraries onto the host computer. First check the device name by running `ls /dev`. It will typically be named something like `/dev/ttyACM0` (though if you are not sure, you can always run `ls /dev` before and after plugging in the Arduino to determine what device has changed). 
   
    Assuming a device name of `/dev/ttyACM0`, the micro-ROS agent can be initiated by running: 
    ```bash
    # In a new terminal, source the ROS 2 installation
    source /opt/ros/$ROS_DISTRO/setup.bash

    # Source microros workspace
    cd microros_ws
    source install/setup.bash

    # Run agent
    ros2 run micro_ros_agent micro_ros_agent serial –dev /dev/ttyACM0 -v6
    ```
    The `-v6` parameter is simply for observing debug output, but can be omitted if desired. 
    
    The micro-ROS agent facilitates the serial communication between the host machine and the microcontroller running micro-ROS; if the agent is successful in communicating with the microcontroller, you should see several `send_message` and `recv_message` debug messages printed to the console. If you don't see this, try hitting the reset button on the Arduino.

    Run `ros2 topic list`. You should be able to see the topic `/micro_ros_arduino_node_publisher`. Echo the topic and verify that data is being received from the microcontroller. If you've gotten this far, that means the installation was successful. Congratulations!

**NOTE:** One caveat is that if the `micro_ros_agent` is killed and restarted, the host machine may stop receiving messages even if the micro-ROS application is still running on the microcontroller. If this occurs, you may need to reset/power cycle the microcontroller for those messages to begin being received again. Work-arounds for this are discussed in [Advanced: Heartbeat for Transient Connectivity](#advanced-heartbeat-for-transient-connectivity).
