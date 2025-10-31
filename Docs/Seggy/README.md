**Seggy robot** is built from a 2016 Segway Ninebot miniPRO - that's why the name. And no, it has nothing to do with "*Crabbed, cantankerous*" or "*exaggerated, stylized movement*" ;-).

**Note:** As of *October 2025* Seggy is in active development. Stable, working code is on the *[main](https://github.com/slgrobotics/articubot_one)* branch.
For the latest code see the *[dev](https://github.com/slgrobotics/articubot_one/tree/dev)* branch.

# Seggy Notes

Seggy is a *"larger Turtlebot"* - running [my *articubot_one* code](https://github.com/slgrobotics/articubot_one). Desktop only needs to run RViz for robot control.

Seggy pretends to be a helpful Tablebot, but in his heart he is a knight in shiny armor.

<img width="512" height="768" alt="seggy_steampunk" src="https://github.com/user-attachments/assets/5aac9913-4dd9-45f3-a8b0-9b140d9093d7" />

Seggy photos are here: https://photos.app.goo.gl/yHXs7fP7u7ae8fa78 - and a story of trying to "robotize" it - [here](https://github.com/slgrobotics/robots_bringup/tree/main/Docs/miniPRO).

**Note:** You can just run Seggy robot in Gazebo sim on a Desktop machine (no robot hardware required) - 
refer to [this section](https://github.com/slgrobotics/robots_bringup/blob/main/Docs/ROS-Jazzy/README.md#build-articubot_one-robot-codebase)

Onboard Seggy has a Raspberry Pi 5 8GB ("seggy"). It runs sensors drivers ([LD14 LIDAR](https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/LD14.md),
[MPU9250](https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/MPU9250.md),
[OAK-D Lite](https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/OAK-D_Lite.md),
[Gesture sensor](https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/FaceGesture.md))
and [Differential Drive Control](https://github.com/slgrobotics/diffdrive_arduino) (inspired by Articulated Robotics), as well as localization and navigation packages.

Seggy has a Teensy 4.0 microcontroller, which drives the BLDC wheel motors (using [simpleFOC library](https://github.com/slgrobotics/Misc/tree/master/Arduino/Sketchbook/WheelsROS_Seggy))
and combines odometry and health data into a single serial stream for the Raspberry Pi.

The *Raspberry Pi 5 8GB* on Seggy has Ubuntu 24.04 Server 64 bit and ROS2 Jazzy Base [installed](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html).

**Note:** Seggy Teensy 4.0 serial should appear on */dev/ttyACM0*

## Build and Run Instructions (on the robot's Raspberry Pi 5):

Follow "Dragger" robot guide starting with "_Raspberry Pi 5 ("dragger") Build and Run Instructions_":

https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Dragger

## Running the robot

After all components are installed in *~/robot_ws* folder:

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

ros2 launch /home/ros/robot_ws/src/articubot_one/robots/seggy/launch/seggy.launch.py
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

With _Seggy_ ROS2 nodes tested, you can set up autostart on boot for hands-free operation.

Follow this guide: https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Ubuntu-RPi/LinuxService.md

----------------

**Back to** [Docs Folder](https://github.com/slgrobotics/robots_bringup/tree/main/Docs)
