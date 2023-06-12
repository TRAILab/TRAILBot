# 1. Micro-ros run_servo_service node

## 1.1. Setup microros agent and micro_ros_arduino
Follow the instructions in [MicroRos_ArduinoDue_Setup_Guide](MicroRos_ArduinoDue_Setup_Guide.md) to install and setup microros with Arduino Due. This setup is needed only once.

## 1.2. Upload to Arduino
Open Arduino ide,  compile and upload `micro-ros_runservo_service.ino` in Arduino Due.

## 1.3. Bind Arduino USB port under static name

Everytime you connect arduino, it usually shows up under /dev/ttyACM0 but it can show up under some other port. We can add rule so that whatever port it shows up under is displayed as /dev/arduino:

1. Connect Arduino to laptop
2. ls /dev/
3. udevadm info --name=/dev/ttyACM0 --attribute-walk
4. In the output for arduino find:
   ```bash
   ATTRS{idProduct}=="zzzz"
   ATTRS{idVendor}=="wwww"
   ```
5. cd /etc/udev/rules.d
6. sudo gedit 99-usb-serial.rules
7. Add the following line: `SUBSYSTEM=="tty", ATTRS{idVendor}=="wwww", ATTRS{idProduct}=="zzzz", SYMLINK+="arduino"`
8. Load the new rule: `sudo udevadm trigger`
9. Check if the static name setting is done properly: `ls -l /dev/arduino`

## 1.3. Run micro-ros agent and test run_servo_service node

1. Power Cycle/Reconnect Arduino Due to laptop via USB.

2. Open a new terminal and run microros agent with --dev (port connected to Arduino):

   ```bash
   # In a new terminal:
   cd ~/trail_ws/src/TRAILBot/Arduino
   ./run_microros_agent.sh
   ```

3. **Test node:** Open a new terminal and call `/runservo` service to see if you can operate servos:
   ```bash
   ros2 service call /runservo trailbot_interfaces/srv/RunServo '{servo: 1}'
   ros2 service call /runservo trailbot_interfaces/srv/RunServo '{servo: 2}'
   ros2 service call /runservo trailbot_interfaces/srv/RunServo '{servo: 3}'
   ros2 service call /runservo trailbot_interfaces/srv/RunServo '{servo: 4}'
   ```