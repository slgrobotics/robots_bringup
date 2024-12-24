# Robots Bringup (Documentation)

This repository contains instructions and other documents related to my "*articubot_one*" project, which originated from Articulated Robotics code:

https://github.com/slgrobotics/articubot_one

It is intended to run navigation on my Turtlebot-like robots - in Gazebo simulation or with physical robots, indoors or outdoors.

Some of the docs here deal with Desktop ("*Ground Station*") setup and operation, while others describe setup of on-board Raspberry Pi computers.

We assume:

- On the Desktop machine (Intel, AMD) - Ubuntu 24.04 64-bit *desktop*, ROS Jazzy

- On-board single board computer (SBC) - Raspberry Pi 5 8GB or similar, Ubuntu *server* 24.04 64-bit, ROS Jazzy

If you just want to run a simulation in Gazebo, or need to set up your Desktop with ROS Jazzy -

see https://github.com/slgrobotics/robots_bringup/tree/main/Docs/ROS-Jazzy (no robot hardware required).

## Dragger

The outdoorsy robot Dragger (Raspberry 5, GPS) is described here:

https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Dragger

## Plucky

An indoors mapper robot Plucky is described here (Raspberry 5):

https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Plucky

## Create 1 Turtlebot

Classic Turtlebot, a visitor from the ancient times, running on Raspberry 3B, now Jazzified:

https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Create1

**Note:** you don't need to download or build anything from this repository, just browse the Docs folder.  

## Useful links

**Articulated Robotics (Josh Newans):**

https://articulatedrobotics.xyz/category/build-a-mobile-robot-with-ros

https://www.facebook.com/ArticulatedRobotics/

https://articulatedrobotics.xyz/mobile-robot-1-project-overview/

https://articulatedrobotics.xyz/mobile-robot-13-ros2-control-real/

https://control.ros.org/jazzy/index.html

https://www.youtube.com/@ArticulatedRobotics/videos

**GPS - localization and navigation:**

https://automaticaddison.com/sensor-fusion-using-the-robot-localization-package-ros-2/

https://docs.ros.org/en/melodic/api/robot_localization/html/integrating_gps.html

https://docs.nav2.org/tutorials/docs/navigation2_with_gps.html

https://index.ros.org/p/robot_localization/#jazzy
