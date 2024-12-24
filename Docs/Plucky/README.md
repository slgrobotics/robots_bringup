# Plucky Notes

```
pluck·y /'pl?ke/ adjective
having or showing determined courage in the face of difficulties.
Synonyms of plucky: SPIRITED, BRAVE
```
Plucky photos are here: https://photos.app.goo.gl/YdYQ8kQrNmLkVXTM7

Plucky is a *"larger Turtlebot"* - running [my *articubot_one* code](https://github.com/slgrobotics/articubot_one). Desktop only needs to run RViz for robot control.

Onboard Plucky has a Raspberry Pi 5 8GB ("plucky"). It runs sensors drivers (LD14 LIDAR, MPU9250 and GPS) and Differential Drive Control (inspired by Articulated Robotics), as well as localization and navigation packages.

Plucky has two Arduino boards, one drives Ultrasonic Parking Sensor, and the other, main Arduino Mega 2560, drives the wheels and combines odometry and health data into a single serial stream for the Raspberry Pi.

There's an option of having an "FPV Drone" TV camera and using _Ultrasonic Parking Sensor_ ("_rangers_" in ROS2 terminology). Parking sensors data will be likely used later, as ROS2 supports rangers for obstacle avoidance and mapping.

The Raspberry Pi 5 on Plucky have Ubuntu 24.04 Server 64 bit and ROS2 Jazzy Base installed - https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html

Arduino Mega 2560 code - wheels/sensors driver: 

https://github.com/slgrobotics/Misc/tree/master/Arduino/Sketchbook/PluckyWheelsROS

**Note:** Plucky Arduino Mega 2560 should be on /dev/ttyACM0

Parking sensors info and driver (to be used later):

https://photos.app.goo.gl/WsqkA4XpYSLrVDX59

https://github.com/slgrobotics/Misc/tree/master/Arduino/Sketchbook/ParkingSensorI2C

## Build and Run Instructions:

Follow "Dragger" robot guide starting with "_Raspberry Pi 5 ("dragger") Build and Run Instructions_":

https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Dragger

## Running the robot

For convenience, create and populate _launch_ directory:
```
mkdir ~/launch
cp ~/robot_ws/src/articubot_one/launch/bootup_launch.sh ~/launch/.
chmod +x ~/launch/bootup_launch.sh    
```
The script looks like this:
```
#!/bin/bash

source /opt/ros/jazzy/setup.bash

cd /home/ros/robot_ws
colcon build
cd /home/ros/launch

source /home/ros/robot_ws/install/setup.bash

ros2 launch /home/ros/robot_ws/src/articubot_one/robots/plucky/launch/plucky.launch.py
```
Robot can be started with:
```
cd ~/launch
./bootup_launch.sh
```
On the Desktop, run the following:
```
cd ~/robot_ws
source ~/robot_ws/install/setup.bash
colcon build; ros2 launch articubot_one launch_rviz.launch.py use_sim_time:=false
```
For diagnostics, run *rqt* and *rqt_graph*


## _Optional:_ Create a Linux service for on-boot autostart

With _Plucky_ ROS2 nodes tested, you can set up autostart on boot for hands-free operation.

Follow this guide: https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Ubuntu-RPi/LinuxService.md

## _Optional_: FPV Camera and receiver Setup

Follow this guide: https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/Camera.md

----------------

Back to https://github.com/slgrobotics/robots_bringup

----------------

## Useful Links:

Articulated Robotics (Josh Newans):

https://articulatedrobotics.xyz/mobile-robot-1-project-overview/

https://articulatedrobotics.xyz/mobile-robot-13-ros2-control-real/

https://articulatedrobotics.xyz/mobile-robot-full-list/

https://www.facebook.com/ArticulatedRobotics/

