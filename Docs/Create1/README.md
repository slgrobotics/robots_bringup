# Turtlebot Create 1 Notes (Jazzy version)

This is an updated and streamlined version of the prior document ( https://github.com/slgrobotics/turtlebot_create/tree/main/RPi_Setup ) - adjusted for ROS Jazzy.

Familiarity with https://github.com/slgrobotics/turtlebot_create/blob/main/README.md is highly recommended.

My Create 1 has a Raspberry Pi 3B ("turtle"). The RPi runs sensors drivers (XV11 LIDAR and BNO055), and Differential Drive Control, inspired by Articulated Robotics.

## Turtle Raspberry Pi 3B Build and Run Instructions:

Turtle has _Ubuntu 24.04 Server (64 bit)_ and _ROS2 Jazzy Base_ installed - see https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Ubuntu-RPi for instructions.

### "Turtle" XV11 LIDAR setup:

Surreal XV Lidar controller v1.2 (Neato Lidar) - connected via USB

https://github.com/getSurreal/XV_Lidar_Controller  - Teensy software

https://www.getsurreal.com/product/lidar-controller-v2-0/   - hardware (Teensy 2.0)

Connect to the USB port at 115200 baud. (test with ```minicom -D /dev/ttyACM0 -b 115200```)

Original ROS2 driver port (by Mark Johnston): https://github.com/mjstn/xv_11_driver

In my fork I modified one file, ```.../xv_11_driver/src/xv_11_driver.cpp```, as _declare_parameter()_ now requires a default value as a second parameter.
I also redefined ```XV11_PORT_DEFAULT``` as ```/dev/ttyACM0```

Commands to compile and install:
```
sudo apt install libboost-dev

mkdir -p ~/robot_ws/src
cd ~/robot_ws/src
git clone https://github.com/slgrobotics/xv_11_driver.git

  (edit the xv_11_driver/src/xv_11_driver.cpp here - XV11_PORT_DEFAULT if your port is not /dev/ttyACM0)

cd ..
colcon build
```
Try running it on _turtle_, see _/scan_ messages in rqt on the Desktop:
```
source ~/robot_ws/install/setup.bash
ros2 run xv_11_driver xv_11_driver &
```

**Note:** Rviz **on your desktop machine** needs at least a static transform, to relate the grid to the laser frame ("_neato_laser_" in this case).

    rviz2 &
    ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map neato_laser &

For Rviz you need:

    Global Options to have Fixed Frame set to a known TF ("map")
  
    Grid reference frame - set to "map"
  
    Add LaserScan, topic "/scan", Style :Spheres" size 0.02
  
    ros2 run tf2_ros tf2_echo map neato_laser            -- to see published TF

### Compile ROS2 driver for BNO055 IMU

Connect to I2C: SCL - pin 05, SDA - pin 03 of Raspberry Pi

Info and tests:

https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor

https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/webserial-visualizer

https://hackmd.io/@edgesense/ryzAq3IFs

BNO055 IMU (via UART or I2C - Python) - seems well supported, active development, ROS2 node:

https://github.com/flynneva/bno055
```
sudo apt install python3-smbus

# mkdir -p ~/robot_ws/src
cd ~/robot_ws/src/
git clone https://github.com/flynneva/bno055.git
vi ~/robot_ws/src/bno055/bno055/params/bno055_params_i2c.yaml   - change i2c_bus to 1. Use i2cdetect -y 1

cd ~/robot_ws
### Note: See https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html
sudo rosdep init     -- do it once
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build
``` 
Try running it on _turtle_, see IMU messages in rqt on the Desktop:
``` 
source ~/robot_ws/install/setup.bash
ros2 run bno055 bno055  --ros-args --params-file ~/robot_ws/src/bno055/bno055/params/bno055_params_i2c.yaml
```

### Compile ROS2 driver for Create 1 base

For iRobot Create 1 (released in 2004, based on Roomba 500 series) and other roombas of 400, 500 and 600 series (https://en.wikipedia.org/wiki/IRobot_Create).

It can be connected via FTDI USB Serial (_/dev/ttyUSB0_ on turtle) or via TTL Serial (pins 1-RXD and 2-TXD on the DB25 connector). A desktop machine can connect via RS232 serial, using special iRobot Create serial cable (/dev/ttyS0 on desktop).

Generally, we follow this guide:

https://github.com/girvenavery2022/create_robot/tree/galactic
    
Review the following. Create 1 requires analog gyro, connected to pin 4 of its Cargo Bay DB25:

https://github.com/AutonomyLab/create_robot/issues/28

https://github.com/slgrobotics/create_robot/tree/foxy

https://github.com/slgrobotics/libcreate

https://github.com/slgrobotics/Misc/tree/master/Arduino/Sketchbook/MPU9250GyroTurtlebot

here are all commands:
```
# mkdir -p ~/robot_ws/src
cd ~/robot_ws/src
git clone https://github.com/slgrobotics/create_robot.git --branch foxy
git clone https://github.com/slgrobotics/libcreate.git
vi ~/robot_ws/src/create_robot/create_bringup/config/default.yaml    (edit port - /dev/ttyS0 if on desktop, /dev/ttyUSB0 on turtle)

cd ~/robot_ws
### Note: See https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html
# sudo rosdep init     -- do it once
rosdep update
# this will take a while, many packages installed:
rosdep install --from-paths src --ignore-src -r -y

# Build will take a long while (over an hour) and needs at least 2GB swap space on RPi 3B.
# You can try limiting number of parallel threads:
# export MAKEFLAGS="-j 1"
# colcon build --parallel-workers=1 --executor sequential
colcon build

# Test it on _turtle_:
source ~/create_ws/install/setup.bash
ros2 launch create_bringup create_1.launch
    or, for Roomba 500/600 series:
ros2 launch create_bringup create_2.launch
```    

### Driving Create 1 Turtlebot with _teleop_

At this point you should be able to use teleop **from your desktop machine:**

(skip this if you don't have a joystick on the desktop machine, use keyboard or something else)

Joystick teleop friendly blog:

https://articulatedrobotics.xyz/mobile-robot-14a-teleop/

To test your joystick:
```
ros2 run joy joy_enumerate_devices
ros2 run joy joy_node      # <-- Run in first terminal
ros2 topic echo /joy       # <-- Run in second terminal
```
https://index.ros.org/p/teleop_twist_joy/github-ros2-teleop_twist_joy/

**__Note:__** *joy_node sends cmd_vel messages ONLY when enable_button is pressed (Usually btn 1, 0 for ROS)
	you MUST set enable_button to desired value (0 for btn 1, the "front trigger").
	ros2 param get /teleop_twist_joy_node enable_button  - to see current value*

-----  **Tip:**  Create teleop.sh to run/configure joystick driver:  -------- 
```
#!/bin/bash
set +x
ros2 launch teleop_twist_joy teleop-launch.py &
sleep 3
ros2 param set /teleop_twist_joy_node enable_button 0
ros2 param set /teleop_twist_joy_node enable_turbo_button 3
set -x
```

### 6. Create a Linux service for on-boot autostart

With Create base, XV11 Laser Scanner and BNO055 IMU ROS2 nodes tested, it is time to set up autostart on boot for hands-free operation.

We need some files (copy them from this repository, under and around https://github.com/slgrobotics/turtlebot_create/tree/main/RPi_Setup/launch):

1. Create and populate launch folder: /home/ros/launch
```
mkdir ~/launch
 -- place myturtle.py and bootup_launch.sh here --
chmod +x ~/launch/bootup_launch.sh    
```
Try running the _bootup_launch.sh_ from the command line to see if anything fails.

2. Create service description file - /etc/systemd/system/robot.service :
```
# /etc/systemd/system/robot.service
[Unit]
Description=turtle
StartLimitIntervalSec=60
StartLimitBurst=5

[Service]
Type=simple
User=ros
Group=ros
WorkingDirectory=/home/ros/launch
ExecStart=/home/ros/launch/bootup_launch.sh
Restart=always

[Install]
WantedBy=multi-user.target
```

3. Enable service:
```
sudo systemctl daemon-reload
sudo systemctl enable turtle.service
sudo systemctl start turtle.service
```
If all went well, the service will start automatically after you reboot the RPi, and all related nodes will show up on _rpt_ and _rpt_graph_

Here are some useful commands:
```
systemctl status turtle.service
systemctl cat turtle.service
sudo systemctl reload-or-restart turtle.service
sudo journalctl -xeu turtle.service

sudo ls -al /etc/systemd/system/turtle.service
sudo ls -al /etc/systemd/system/turtle.service.d/override.conf
sudo ls -al /etc/systemd/system/multi-user.target.wants/turtle.service
ps -ef | grep driver
```
You can now reboot Raspberry Pi, and the three drivers will start automatically and show up in **rqt** and **rqt_graph** on the Desktop

**Now you can proceed to Desktop (Turtle_Setup) folder:**   https://github.com/slgrobotics/turtlebot_create/tree/main/Turtle_Setup

## Useful Links:

