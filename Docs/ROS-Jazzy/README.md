_"It takes all the running you can do, to keep in the same place. If you want to get somewhere else, you must run at least twice as fast as that!"_ - Alice in Wonderland

# Set up Desktop ROS Jazzy "Clean Machine" from scratch

Here we set up a new Desktop PC to become a ROS playground machine, and a Ground Station for our robots. 

## Installing Ubuntu 24.04 LTS ("Noble")

If your machine can't boot from USB Flash Drive:

Install "server" image from DVD (Desktop image doesn't fit on DVD):

https://ubuntu.com/download/server

Or, if your machine can boot up from USB media, go for the Desktop image and skip the next section

https://ubuntu.com/download/desktop

### Expand "server" edition to "desktop" (just add GUI):
```
sudo apt update
sudo apt upgrade
sudo apt install ubuntu-desktop gdm3
sudo reboot now
locale
sudo apt install software-properties-common
```

### (...set up Samba, Chrony and other OS things here...).

Take a look at Raspberry Pi setup and choose what is relevant to your Desktop installation:

https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Ubuntu-RPi/README.md

## Install ROS Jazzy Jalisco Desktop LTS:

Follow  https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html

### Install rqt and rqt_graph with all plugins:
```
sudo apt install ros-${ROS_DISTRO}-rqt*
```

### for joystick operation - make sure your "ros" account has access to ports:
```
sudo adduser <your account> dialout
sudo apt install ros-${ROS_DISTRO}-joy*
```

## Installing Gasebo Harmonic (using "Default Gazebo/ROS Pairing"):

https://gazebosim.org/docs/harmonic/ros_installation#installing-the-default-gazebo-ros-pairing
```
sudo apt install ros-${ROS_DISTRO}-ros-gz
# see if Gazebo UI comes up without crashing:
gz sim
```

### Solving Gazebo crash on startup:
```
export QT_QPA_PLATFORM=xcb
gz sim
gz sim -v 4 shapes.sdf
```
My Windows machine has a good video card, and can act as X Window Server, relieving the Linux box.

Running Gnome Desktop on a different machine (Windows 10 with VcXsrv in my case):
```
export DISPLAY=<machine>.local:0.0
gnome-shell --x11 --replace
```
now Gazebo UI can show up on VcXsrv:
```
export DISPLAY=<machine>.local:0.0
gz sim -v 4 shapes.sdf
```

## Installing Controller Manager and its infrastructure:
```
sudo apt install ros-${ROS_DISTRO}-ros2-control ros-${ROS_DISTRO}-ros2-controllers

# this puts in place /opt/ros/jazzy/lib/libgz_ros2_control-system.so - a plugin for Gazebo:
sudo apt install ros-${ROS_DISTRO}-gz-ros2-control

# some extras just in case:
sudo apt install ros-${ROS_DISTRO}-joint-state-publisher ros-${ROS_DISTRO}-joint-state-publisher-gui
```
Install _"topic relay"_ tool for remapping - in case you ever need it.

See https://github.com/ros-tooling/topic_tools/blob/jazzy/README.md for details.
```
sudo apt install ros-${ROS_DISTRO}-topic-tools
```

### Install xacro processor and Joystick mixer:
```
sudo apt install ros-${ROS_DISTRO}-xacro ros-${ROS_DISTRO}-twist-mux
```
### Joystick setup

Follow this guide: https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/Joystick.md

## Installing additional navigation and visualization components

To allow GPS operation in sim install localization package, SLAM Toolbox and Nav2:
```
sudo apt install ros-${ROS_DISTRO}-robot-localization ros-${ROS_DISTRO}-imu-tools ros-${ROS_DISTRO}-slam-toolbox
sudo apt install ros-${ROS_DISTRO}-navigation2 ros-${ROS_DISTRO}-nav2-bringup ros-${ROS_DISTRO}-rmw-cyclonedds-cpp
```
You need to configure ROS to use Cyclone DDS. Make sure the tail of your _.bashrc_ looks like this:
```
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
# gz sim crashes without the following:
#export QT_QPA_PLATFORM=xcb
```

More info - see "Useful Links" below.

## Note: Visualizing terrain maps

Some robots could be designed for outdoors operation and will use GPS for localization - for example, Dragger. So, we want to see where it is on the aerial map.

There are two ways of visualizing robot position on a map - *mapviz* and *rviz-satellite*

Here are the links:

https://swri-robotics.github.io/mapviz/

https://github.com/nobleo/rviz_satellite/tree/main/

There is no binary distribution of *mapviz* for Jazzy, you have to compile it from sources.
Microsoft deprecated Bing Maps keys, and Azure keys don't seem to work with mapviz.
I wasn't able to make the sim work with mapviz anyway.

For *rviz-satellite* things are much easier. It works as an Rviz2 "AerialMap" plugin. To install it:
```
sudo apt install ros-${ROS_DISTRO}-rviz-satellite
```
Object URI ```https://tile.openstreetmap.org/{z}/{x}/{y}.png```  works just fine and doesn't require API keys.

The plugin is sensitive to time stamps, so all components must use the same *use_sim_time* settings.

Running the sim as described below brings Rviz2 with a map view.

## Build "articubot_one" robot codebase.

**Note:** If you want to browse robot's code look here: https://github.com/slgrobotics/articubot_one

Here we build a ROS2 package we need to run *Dragger*, *Plucky* or *Turtle* in Gazebo sim or use RViz to control them.

So, on the Desktop machine:

```
cd
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src
git clone https://github.com/slgrobotics/articubot_one.git
cd ~/robot_ws

sudo rosdep init    # do it once, if you haven't done it before
rosdep update
rosdep install --from-paths src --ignore-src -r -y

colcon build
```

If all goes well, we can now:
1. Use joystick to control the robots - simulated or physical.
2. Run any robot in Gazebo with RViz to monitor simulated robot.
3. Run RViz alone to monitor a physical robot.

## Bringing up robot simulation in Gazebo

Once the *articubot_one* package is built on the Desktop machine, we can run simulation in Gazebo.

```
source /opt/ros/${ROS_DISTRO}/setup.bash
cd ~/robot_ws
colcon build
source ~/robot_ws/install/setup.bash
ros2 launch articubot_one dragger_sim.launch.py

or
ros2 launch articubot_one plucky_sim.launch.py

or
ros2 launch articubot_one turtle_sim.launch.py
```
You should see Gazebo and RViz GUI coming up. You may see a white 150m x 150m square ("empty_map" from *map_server*) or SLAM Toolbox or Cartographer should be building map as you move the robot around with joystick. If you zoom a bit out in RViz, for an outdoor robot you will see aerial map. 

The simulated robot should respond to Joystick via teleop. Make sure that your "enable" and "turbo" buttons are assigned correctly in _~/robot_ws/src/articubot_one/launch/joystick.launch.py_

You must do _"colcon build"_ in _~/robot_ws_ every time you change anything.

## Running a physical robot

**Note:** Consult physical robot pages:
- Dragger: https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Dragger
- Plucky: https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Plucky
- Turtle:https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Create1

You are unlikely to have an exact clone of Dragger or Plucky, although Create 1 Turtlebot might be still alive in your collection. Or you could've made files for your own robot in the _robots/_ directory.

There are launch files in _robots/_ directories for real robots, intended to *run on their Raspbery Pi's*. So, once the Pi is up and is running robot's nodes, this is how to launch RViz alone on the Desktop computer:
```
cd ~/robot_ws
source ~/robot_ws/install/setup.bash
ros2 launch articubot_one launch_rviz.launch.py
```
While Raspberry Pi 5 on Dragger and Plucky run all robots' Nodes (including Nav2), the Turtle's Raspberry Pi 3B only runs Create 1 _base_ and sensors Nodes.
So, the turtle launch file must run all the remaining robot's nodes on the Desktop. It is still started the same way (and brings up RViz):
```
cd ~/robot_ws
source ~/robot_ws/install/setup.bash
ros2 launch articubot_one turtle.launch.py
```

--------------------------------------------

## Optional: running Cartographer

Cartographer is just one of localaizers - *slam_toolbox* and *map_server* being the other two I use. It can be run from a robot's launch file or separately, as described here.

When running Cartographer, specify simulated time (true or false):
```
source ~/robot_ws/install/setup.bash
ros2 launch articubot_one cartographer.launch.py use_sim_time:=true
```

## Optional: Install Husarnet VPN

If you want Cyclone DDS to work across LAN _subnets_, 5G/LTE phone hotspots etc. - you need Husarnet VPN.

See https://github.com/slgrobotics/robots_bringup/blob/main/Docs/ROS-Jazzy/README-Husarnet.md

## Useful links

_Gazebo Harmonic - Differential Drive project template_ : https://gazebosim.org/docs/latest/ros_gz_project_template_guide

_and its GitHub repository_ : https://github.com/gazebosim/ros_gz_project_template/tree/main

_How to Use ROS2 Jazzy and Gazebo Harmonic for Robot Simulation_: https://www.youtube.com/watch?v=b8VwSsbZYn0

_ros2_control documentation_ : https://control.ros.org/jazzy/index.html

_IMU tools for Rviz_ : https://github.com/CCNYRoboticsLab/imu_tools/tree/humble


**Articulated Robotics (Josh Newans):**

https://articulatedrobotics.xyz/category/build-a-mobile-robot-with-ros

https://articulatedrobotics.xyz/mobile-robot-1-project-overview/

https://articulatedrobotics.xyz/mobile-robot-13-ros2-control-real/

https://www.youtube.com/@ArticulatedRobotics/videos

https://www.facebook.com/ArticulatedRobotics/

**GPS - localization and navigation:**

https://automaticaddison.com/sensor-fusion-using-the-robot-localization-package-ros-2/

https://docs.ros.org/en/melodic/api/robot_localization/html/integrating_gps.html

https://docs.nav2.org/tutorials/docs/navigation2_with_gps.html

https://index.ros.org/p/robot_localization/#jazzy

**SLAM Toolbox**

https://www.youtube.com/watch?v=0G6LDuslqmA

https://www.youtube.com/watch?v=hMTxb8Y2cxI

https://github.com/SteveMacenski/slam_toolbox/tree/jazzy

https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html

https://roboticsbackend.com/ros2-nav2-generate-a-map-with-slam_toolbox/  <- DDS change to CycloneDDS

https://stevengong.co/notes/slam_toolbox

