# Robots Bringup (Documentation)

This repository contains instructions and other documents related to my "[*articubot_one*](https://github.com/slgrobotics/articubot_one)" project.

It is intended to run navigation on my Turtlebot-like robots - in Gazebo simulation or with physical robots, indoors or outdoors.

Some of the docs here deal with Desktop ("*Ground Station*") setup and operation, while others describe setup of on-board Raspberry Pi computers.

We assume:

- On the Desktop machine (Intel, AMD) - Ubuntu 24.04 64-bit *desktop*, ROS Jazzy _desktop_

- On-board single board computer (SBC) - Raspberry Pi 5 8GB or similar, Ubuntu *server* 24.04 64-bit, ROS Jazzy _base_ (access via _ssh_)

If you just want to run a simulation in Gazebo, or need to set up your Desktop with ROS Jazzy -
see [ROS2 Jazzy Setup](https://github.com/slgrobotics/robots_bringup/tree/main/Docs/ROS-Jazzy) (no robot hardware required).

# Here are the robots:

## Dragger

The outdoorsy robot Dragger (Raspberry 5, GPS) is described here:
[Docs/Dragger](https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Dragger)

## Plucky

An indoors mapper robot Plucky is described here (Raspberry 5):
[Docs/Plucky](https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Plucky)

## The Venerable _Create 1_ Turtlebot, now Jazzified

Classic Turtlebot, a visitor from the ancient times, running on Raspberry 3B:
[Docs/Create1](https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Create1)

# How to use this repository

**Note:** you don't need to download or build anything from this repository, just browse the Docs folder.

Here are the likely scenarios for your work.

### 1. Just set up a "Clean Machine" for ROS2 Jazzy work

- Desktop workstation: see [Docs/ROS-Jazzy](https://github.com/slgrobotics/robots_bringup/tree/main/Docs/ROS-Jazzy) (no robot hardware required).

- Set up Raspberry PI: see [Docs/Ubuntu-RPi](https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Ubuntu-RPi)

### 2. Try my robots in Gazebo simulation, browse the code

- See the #1 above, just follow the build and run instructions there for Dragger, Plucky or Turtle sims.

- Bonus: ROS2 Jazzy comes with _Turtlebot3_ simulation, try that to see Nav2 working.

- To browse robot's code look under *~/robot_ws/src/articubot_one* - use VS Code or your favorite IDE.

### 3. Work on your own codebase

- *articubot_one* codebase offers a working example of indoors and outdoors (GPS) operation, use Copy/Paste

- Fork my *articubot_one* repository and make your own robot in ~/robot_ws/src/articubot_one/robots folder

### Found a bug? Want to contribute a fix?

- Report here: https://github.com/slgrobotics/articubot_one/issues

- Create a _Pull Request_: https://github.com/slgrobotics/articubot_one/pulls

---------------------------------

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

https://roboticsbackend.com/ros2-nav2-tutorial/
