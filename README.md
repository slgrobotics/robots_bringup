# Robots Bringup - Plucky

This is my *robots_bringup* ROS2 package, derived from Turtlebot3 Navigation and some Articulate Robotics code.

It is intended to run navigation on my Tutrtlebot-like robots, starting with Plucky.

Plucky is described here: https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Plucky

## Build and Run Instructions:

On the Desktop machine (Ubuntu 22.04)
```
mkdir -p ~/bringup_ws/src
cd ~/bringup_ws/src/
git clone https://github.com/slgrobotics/robots_bringup.git
cd ~/bringup_ws
colcon build
```

## Running Cartographer

You may want to try Cartographer first - to see if odometry works well, and to create a map:
```
ros2 launch turtlebot3_cartographer cartographer.launch.py
```
Save map ("my_map.pgm"):
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

This will bring up full Nav2 stack and Rviz2:
```
source ~/bringup_ws/install/setup.bash
ros2 launch robots_bringup bringup_launch.py map:=/home/sergei/my_map_plucky.yaml

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
