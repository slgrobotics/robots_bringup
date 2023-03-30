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

source ~/bringup_ws/install/setup.bash
ros2 launch robots_bringup bringup_launch.py map:=/home/sergei/my_map_plucky.yaml

source ~/bringup_ws/install/setup.bash
ros2 launch robots_bringup rviz_launch.py
```
The above will bring up full Nav2 stack and Rviz2

Note, that *map->odom* transformation is published by **amcl** which was launched as a part of nav2_bringup. But **amcl** needs an initial pose to start working.

So, in RViz:


