## Make Your Own Robot

This section describes hardware components which can be used to make a robot very similar to my Plucky and Dragger.

Turtle (Create 1 robot) is a bit different, and could be replicated verbatim, if you have an iRobot Create 1 base.

Building a robot "from scratch" (wheels, motors, electronics) might be too much for you - consider a Neato, Create 1/2/3 or any other proven base with available ROS2 driver. 

Once you've put together all recommended hardware, [my software](https://github.com/slgrobotics/articubot_one) should work without much change. 
It should work with very little change on almost any hardware, once you figured out how to move the wheels with *cmd_vel* topic and deliver wheel encoder ticks to be published by joint broadcaster.

## General Considerations

In ROS2 ecosystem the process of building robots comes down to combining available components - both on the software side and in hardware. 
The availability of software components (ROS2 _packages_, _Nodes_, _launch_ and _config_ files) takes priority. 
Having, for example, a great sensor or motor controller which does not have a ROS2 driver makes it no more than an expensive paper weight.

As the ROS2 robots are, in general, quite complex - it is important to start with and example source code, in which all configuration and tuning has been done. 
You can take that code base, expand and modify it, while preserving its integrity.
Starting with a very simple "tutorial quality" code sample is not offering an easy path.
Refer to [this guide](https://github.com/slgrobotics/robots_bringup?tab=readme-ov-file#how-to-use-this-repository) for an example of a good "starting point robot code".

In my experience, it is very important to have a robot work in simulated environment, with simulated hardware.
ROS2 interfaces with Gazebo, and if your robot works in simulation, it will likely behave the same way in reality.
Moreover, you can learn the ropes in sim before ever paying a penny for hardware.
You can first "build" your robot in simulation (providing an URDF/.xacro _robot description_ file) and then implement it in hardware.

When working with robots you need a _Workstation_ - usually a gaming-class desktop PC under Ubuntu 24.04 [Desktop](https://github.com/slgrobotics/robots_bringup/tree/main/Docs/ROS-Jazzy).
You will need a [joystick](https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/Joystick.md) connected to it.
Make sure that you can run Plucky, Dragger or Turtle in Gazebo [simulation](https://github.com/slgrobotics/robots_bringup/tree/main/Docs/ROS-Jazzy#bringing-up-robot-simulation-in-gazebo) to confirm proper installation.

Your physical robot will likely have a Raspberry 5 SBC, preferrably with 8 Gb RAM. Install the OS using [this guide](https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Ubuntu-RPi).

You will need a rock-solid WiFi - use [this guide](https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/WiFi_Logger_Visualizer.md#wifi-testing-and-benchmarking) to test it.

Note that VS Code IDE works very well with ROS2 code, and GitHub is a great tool and resource.

## Building Robot in "Layers"

In ROS2 you can build your physical robot in phases, adding "layers" of hardware and software.

**First**, you need to build a "base" (Chassis) - for example, a wheeled cart with motors and motor controller.  
In ROS2 software it will be represented by motor driver and a *base_link* and other elements in *robot_description* (URDF, xacro) files.
iRobot Create 1 is an example of such a "base".
A _Differential drive_ base is easier to configure and handle in ROS2, in comparison with _Ackermann steering_ types.
- **Goal:** You will need to have your base move when controlled by joystick.

**Second**, you need a collection of [sensors](https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Sensors).
A mere minimum is IMU and LIDAR - and for outdoors you need a GPS receiver. I'd recommend against cameras at this phase.
In ROS2 software each sensor will be represented by a _Node_ and will publish its data for all other nodes to consume (subscribe to).
- **Goal:** look at topics (using _rqt_ and *rqt_graph_) and understand how to launch and configure sensors.

**Third**, you need to configure and run Localization (SLAM Toolbox), Navigation (Nav2) packages which ultimately make your robot "alive".
- **Goal:** learn configuration and launch files, as this is how ROS2 is really programmed. 

**Finally**, you can delve into programming robot behaviors (using Behavior Trees or Python scripts for example) to accomplish non-trivial tasks.
You can play with cameras, sonars, point clouds, AI object detection and other cool technologies.


----------------

**Back to** [Docs Folder](https://github.com/slgrobotics/robots_bringup/tree/main/Docs)
