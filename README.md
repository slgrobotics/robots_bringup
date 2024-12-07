# Robots Bringup (Desktop side)

This is my *robots_bringup* ROS2 package, derived from Turtlebot3 Navigation and some Articulated Robotics code.

It is intended to run navigation on my Turtlebot-like robots - in Gazebo simulation or with physical robots.

This section deals with Desktop ("Ground Station") setup and operation.

If you just want to run a simulation in Gazebo, or need to set up your Desktop with ROS Jazzy -

see https://github.com/slgrobotics/robots_bringup/tree/main/Docs/ROS-Jazzy

If you want to browse robot's code look here:

https://github.com/slgrobotics/articubot_one

This section equally applies to any robot, just keep an eye on the name in the instructions and replace it accordingly.

You need to setup **TURTLEBOT3_MODEL** variable (normally in _.bashrc_) with the robot name. Robot description file will be picked from _~/bringup_ws/src/robots_bringup/urdf_ folder

## Dragger

The outdoorsy robot Dragger (Raspberry 5, GPS) is described here:

https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Dragger

## Plucky

An indoors mapper robot Plucky is described here (including its two RPi 3B **setup instructions**):

https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Plucky

## Create 1 Turtlebot

Classic Turtlebot, a visitor from the ancient times, now Jazzified:

https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Create1

# Any of the robots above:

## Tests - make sure robot's "base" responds properly

1. Publish _cmd_vel_ and see the wheels rotating

Run this script on any machine (Desktop or plucky):

```
$ cat test_wheels.sh 

#!/bin/bash

ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"

```

2. Use Desktop's joystick to drive the robot

Run the following script on the Desktop:
```
$ cat teleop.sh

#!/bin/bash

ros2 launch teleop_twist_joy teleop-launch.py &
sleep 3
ros2 param set /teleop_twist_joy_node enable_button 0
ros2 param set /teleop_twist_joy_node enable_turbo_button 3
```
In theory the standard teleop will work with any joystick. The trick is to see where your "enable" and "turbo" buttons are, as you have to hold one of these to have _cmd_vel_ published (safety measure).

On "Logitech Cordless RumblePad 2" (a.k.a. Logitech Wireless Gamepad F710) Button 0 is blue "X" and Button 3 is yellow "Y". Make sure the back switch is on "D", the "Mode" light is off - and operate Left stick while holding the blue "X" down. If in doubt, _rqt_ will show "joy" topic _axes_ (including buttons) and _cmd_vel_

## Build and Run Instructions:

On the Desktop machine (Ubuntu 24.04, ROS Jazzy)
```
mkdir -p ~/bringup_ws/src
cd ~/bringup_ws/src/
git clone https://github.com/slgrobotics/robots_bringup.git
cd ~/bringup_ws
colcon build
```
## Build Turtlebot3 from sources (ROS Jazzy)

Follow https://github.com/slgrobotics/turtlebot3

make sure you ```source ~/turtlebot3_ws/install/setup.bash```

## Running Cartographer

You may want to try Cartographer first - to see if odometry works well, and to create a map:
```
ros2 launch turtlebot3_cartographer cartographer.launch.py
```
This should bring up RViz and, with Lidar working, the (incomplete) map with all scan data. While driving the robot around you should see the map filled.

------------------------

Ideally, the map will stay put when the robot moves or turns, and _map_ and _odom_ markers should not deviate far fom each other. If the map gets messed up, your odometry is not working properly.
Check your **TURTLEBOT3_MODEL** variable and modify the following files on robot's Raspberry Pi:
```
~/robot_ws/src/articubot_one/description/ros2_control.xacro

           look for  <param name="enc_counts_per_rev">13730</param>

~/robot_ws/src/articubot_one/description/robot_core.xacro

           look for <xacro:property name="wheel_radius" value="0.192"/>
                    <xacro:property name="wheel_offset_y" value="0.290"/>
```
and rebuild the robot:
```
cd ~/robot_ws/
colcon build
```
The _.xacro_ files should be generating the following statements in https://github.com/slgrobotics/robots_bringup/tree/main/urdf
```
<param name="enc_counts_per_rev">2506</param>
wheel parameters like this, also wheel base:
<cylinder length="0.026" radius="0.033"/>

```
**this is not happenning, URDF files are hand edited:**
  The URDF file should be generated from https://github.com/slgrobotics/articubot_one/blob/main/description/robot.urdf.xacro and somehow are residing in the _~/bringup_ws/src/robots_bringup/urdf_ folder

------------------------

Save map (produces two files - "my_map.pgm", "my_map.yaml"):
```
ros2 run nav2_map_server map_saver_cli -f my_map
```
If in doubt (for example, not seeing "map" in TFs), you can always run static transform:
```
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
```
If your map doesn't come up well, and mapping fails when the robot turns - your odometry needs tuning.
Ideally the "odom" and "map" points in RViz should stay close together at all times.
If they separate, either ticks-per-rotation or wheels base are reported incorrectly.

A good test is to start the robot, make sure odom and map are close, drive the bot forward 2 meters, turn 90 degrees.
Odom should not move much.

## Running Nav2

This will bring up full Nav2 stack and Rviz2 (better specify fill path to *my_map.yaml*):
```
source ~/bringup_ws/install/setup.bash
ros2 launch robots_bringup bringup_launch.py map:=my_map.yaml

source ~/bringup_ws/install/setup.bash
ros2 launch robots_bringup rviz_launch.py
```
Note, that *map->odom* transformation is published by **amcl** which was launched as a part of nav2_bringup. But **amcl** needs an initial pose to start working.

In RViz there's a confusing sequence of clicks when you run Nav2 - to enable AMCL posting "map->odom" transform. 

First click on Startup in Nav2 Panel in Rviz. Wait a minute, map should appear. Click on "2D Pose Estimate", wait till LIDAR readings appear (i.e. map->odom TF starts publishing).

If in doubt (i.e. not seeing "map" in TFs), you can always run static transform:
```
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
```
I wasn't able to make Slam Toolbox work (option slam:=True) with Nav2, but AMCL works fine.
