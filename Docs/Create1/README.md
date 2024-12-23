# Turtlebot Create 1 Notes (Jazzy version)

This is an updated and streamlined version of the prior document ( https://github.com/slgrobotics/turtlebot_create/tree/main/RPi_Setup ) - adjusted for ROS Jazzy.

Familiarity with https://github.com/slgrobotics/turtlebot_create/blob/main/README.md is highly recommended.

My Create 1 has a Raspberry Pi 3B ("turtle"). The RPi runs sensors drivers (XV11 LIDAR and BNO055), and Differential Drive Control, inspired by Articulated Robotics.

## Turtle Raspberry Pi 3B Build and Run Instructions:

Turtle has _Ubuntu 24.04 Server (64 bit)_ and _ROS2 Jazzy Base_ installed - see https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Ubuntu-RPi for instructions.

### XV11 LIDAR setup

Refer to https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/XV11_LIDAR.md

Alternatively, use LD14 LIDAR: https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/LD14.md

### ROS2 driver for BNO055 9DOF IMU

Refer to https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/BNO055%20IMU.md

Alternatively, use MPU9250 sensor: https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/MPU9250.md

### Compile ROS2 driver for Create 1 base

For iRobot Create 1 (released in 2004, based on Roomba 500 series) and other roombas of 400, 500 and 600 series (https://en.wikipedia.org/wiki/IRobot_Create).

It can be connected via FTDI USB Serial (_/dev/ttyUSB0_ on turtle) or via TTL Serial (pins 1-RXD and 2-TXD on the DB25 connector). A desktop machine can connect via RS232 serial, using special iRobot Create serial cable (/dev/ttyS0 on desktop).

Generally, we follow this guide:

https://github.com/girvenavery2022/create_robot/tree/galactic
    
Review the following. Create 1 requires analog gyro, connected to pin 4 of its Cargo Bay DB25:

https://github.com/AutonomyLab/create_robot/issues/28

https://github.com/slgrobotics/create_robot   (branch _"jazzy"_ is default there, forked from upstream _"iron"_)

https://github.com/slgrobotics/libcreate

https://github.com/slgrobotics/Misc/tree/master/Arduino/Sketchbook/MPU9250GyroTurtlebot

Here are all commands:
```
# mkdir -p ~/robot_ws/src
cd ~/robot_ws/src
git clone https://github.com/slgrobotics/create_robot.git
git clone https://github.com/slgrobotics/libcreate.git

# you may edit Create Base port here - /dev/ttyS0 if on desktop, /dev/ttyUSB0 on turtle:
vi ~/robot_ws/src/create_robot/create_bringup/config/default.yaml

cd ~/robot_ws
### Note: See https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html
# sudo rosdep init     -- do it once
rosdep update
# this will take a while, many additional packages installed:
rosdep install --from-paths src --ignore-src -r -y

# On RPi 3B build will take VERY long time (over 14 hours in my case) and needs at least 2GB swap space.
# You can try limiting number of parallel threads:
# export MAKEFLAGS="-j 1"
# colcon build --parallel-workers=1 --executor sequential
colcon build
```
This is how it looked on my Raspberry Pi 3B with 1 GB RAM:
```
ros@turtle:~/robot_ws$ export MAKEFLAGS="-j 1"
ros@turtle:~/robot_ws$ colcon build --parallel-workers=1 --executor sequential
[5.063s] WARNING:colcon.colcon_ros.prefix_path.ament:The path '/home/ros/robot_ws/install/create_robot'
                  in the environment variable AMENT_PREFIX_PATH doesn't exist
...
Starting >>> bno055
Finished <<< bno055 [14.3s]
Starting >>> create_description
Finished <<< create_description [2.29s]
Starting >>> create_msgs
Finished <<< create_msgs [6min 3s]
Starting >>> libcreate
Finished <<< libcreate [7min 48s]
Starting >>> xv_11_driver
Finished <<< xv_11_driver [4min 24s]
Starting >>> create_driver
Finished <<< create_driver [14h 14min 39s]
Starting >>> create_bringup
Finished <<< create_bringup [16.5s]
Starting >>> create_robot
Finished <<< create_robot [10.9s]

Summary: 8 packages finished [14h 33min 43s]
ros@turtle:~/robot_ws$
```
Test it on _turtle_:
```
source ~/robot_ws/install/setup.bash
ros2 launch create_bringup create_1.launch
    or, for Roomba 500/600 series:
ros2 launch create_bringup create_2.launch
```
This is how the robot comes up on my screen:
```
ros@turtle:~/robot_ws$ ros2 launch create_bringup create_1.launch
[INFO] [launch]: All log files can be found below /home/ros/.ros/log/2024-07-23-09-11-31-032285-turtle-26928
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [create_driver-1]: process started with pid [26934]
[INFO] [robot_state_publisher-2]: process started with pid [26935]
[robot_state_publisher-2] [INFO] [1721743897.733288931] [robot_state_publisher]: Robot initialized
[create_driver-1] [INFO] [1721743897.817175928] [create_driver]: [CREATE] gyro_offset: 0    gyro_scale: 1
[create_driver-1] [INFO] [1721743897.818827061] [create_driver]: [CREATE] "CREATE_1" selected
[create_driver-1] [INFO] [1721743898.881177255] [create_driver]: [CREATE] Connection established.
[create_driver-1] [INFO] [1721743898.881650532] [create_driver]: [CREATE] Battery level 100.00 %
[create_driver-1] [INFO] [1721743899.054224697] [create_driver]: [CREATE] Ready.
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
cd ~/launch
# place myturtle.py and bootup_launch.sh here:
cp ~/robot_ws/src/create_robot/create_bringup/launch/bootup_launch.sh .
cp ~/robot_ws/src/create_robot/create_bringup/launch/myturtle.py .
chmod +x ~/launch/bootup_launch.sh    
```
Try running the _bootup_launch.sh_ from the command line to see if anything fails.

2. Create service description file (as _"sudo"_) - _/etc/systemd/system/robot.service_ :
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
sudo systemctl enable robot.service
sudo systemctl start robot.service
```
If all went well, the service will start automatically after you reboot the RPi, and all related nodes will show up on _rpt_ and _rpt_graph_

**Notes:** 

1. Logs are stored in _/home/ros/.ros/log_ folder - these can grow if things go wrong.

2. Raspberry Pi 3B is adequate for the _"headless"_ Turtlebot (except for the 14+ hours compilation) - it runs at <30% CPU load and low memory consumption.

3. Any time you need to produce robot URDF from ```.xacro``` files:
```
source ~/robot_ws/install/setup.bash
xacro ~/robot_ws/install/articubot_one/share/articubot_one/description/robot.urdf.xacro use_ros2_control:=true sim_mode:=true > ~/robot_ws/src/articubot_one/description/robot.urdf
```

Here are some useful commands:
```
systemctl status robot.service
systemctl cat robot.service
sudo systemctl reload-or-restart robot.service
sudo journalctl -xeu robot.service

sudo ls -al /etc/systemd/system/robot.service
sudo ls -al /etc/systemd/system/robot.service.d/override.conf
sudo ls -al /etc/systemd/system/multi-user.target.wants/robot.service
ps -ef | grep driver
```
You can now reboot Raspberry Pi, and the three drivers will start automatically and show up in **rqt** and **rqt_graph** on the Desktop

**Now you can proceed to Desktop (Turtle_Setup) folder:**   https://github.com/slgrobotics/turtlebot_create/tree/main/Turtle_Setup

## Useful Links:

https://roboticsbackend.com/ros2-nav2-tutorial/
