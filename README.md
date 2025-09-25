## Robots Bringup (Documentation)

This repository contains instructions and other [documents](https://github.com/slgrobotics/robots_bringup/tree/main/Docs) related to my "[*articubot_one*](https://github.com/slgrobotics/articubot_one)" project.

It is intended to run navigation on my Turtlebot-like robots - in Gazebo simulation or with physical robots, indoors or outdoors.

Some of the [docs](https://github.com/slgrobotics/robots_bringup/tree/main/Docs) here deal with Desktop ("*Ground Station*") setup and operation, while others describe setup of on-board Raspberry Pi computers.

The [Sensors](https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Sensors) section describes setup of GPS, Cameras, IMU and other devices.

Don't forget to properly setup your [WiFi](https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/WiFi_Logger_Visualizer.md#wifi-testing-and-benchmarking) or [Global VPN](https://github.com/slgrobotics/robots_bringup/blob/main/Docs/ROS-Jazzy/README-Husarnet.md).

We assume:

- On the Desktop machine (Intel, AMD) - Ubuntu 24.04 64-bit *desktop*, ROS Jazzy _desktop_

- On-board single board computer (SBC) - Raspberry Pi 5 8GB or similar, Ubuntu *server* 24.04 64-bit, ROS Jazzy _base_ (access via _ssh_)

If you just want to run a **simulation in Gazebo**, or need to set up your Desktop with ROS Jazzy -
see [ROS2 Jazzy Setup](https://github.com/slgrobotics/robots_bringup/tree/main/Docs/ROS-Jazzy) (no robot hardware required).

## Here are the robots:

### Dragger

The outdoorsy robot Dragger (Raspberry 5, GPS) is described here:
[Docs/Dragger](https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Dragger)

### Plucky

An indoors mapper robot Plucky is described here (Raspberry 5):
[Docs/Plucky](https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Plucky)

### The Venerable _Create 1_ Turtlebot, now Jazzified

Classic Turtlebot, a visitor from the ancient times, running on Raspberry 3B under ROS2 Jazzy:
[Docs/Create1](https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Create1)

### "Make Your Own" robot

This section describes how to make a differential drive [robot](https://github.com/slgrobotics/robots_bringup/tree/main/Docs/MakeYourOwn) "from scratch" - out of widely available parts.

---------

**Note:** Purpose and Limitations of this Project

This software is not ready for production use. It has neither been developed nor tested for a specific use case. However, the license conditions of the applicable Open Source licenses allow you to adapt the software to your needs. Before using it in a safety relevant setting, make sure that the software fulfills your requirements and adjust it according to any applicable safety standards, e.g., ISO 26262.

---------

## How to use this repository

**Note:** you don't need to download or build anything from this repository, just browse the [Docs](https://github.com/slgrobotics/robots_bringup/tree/main/Docs) folder.
Actual robots' code is [here](https://github.com/slgrobotics/articubot_one).

Here are the likely scenarios for your work.

### 1. Just set up a "Clean Machine" to work with ROS2 Jazzy

- Desktop workstation: see [Docs/ROS-Jazzy](https://github.com/slgrobotics/robots_bringup/tree/main/Docs/ROS-Jazzy) (no robot hardware required).

- _Optional:_ Set up Raspberry *on-board* Raspberry Pi: see [Docs/Ubuntu-RPi](https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Ubuntu-RPi)

### 2. Try my robots in _Gazebo simulation_, browse the code (no robot hardware needed)

- See the #1 above, then follow the build and run instructions there for _simulated_ Dragger, Plucky or Turtle.

- _Bonus:_ ROS2 Jazzy comes with _Turtlebot3_ simulation, try that to see _Nav2_ working.

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
