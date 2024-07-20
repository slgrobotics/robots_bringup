# Setup Desktop ROS Jazzy "Clean Machine" from scratch

Here we set up a new Desktop PC to become a ROS playground machine, and a Ground Station for our robots. 

## Installing Ubuntu 24.04 LTS ("Noble")

Install "server" image from DVD (Desktop image doesn't fit on DVD):

       https://ubuntu.com/download/server

Or, if your machine can boot up from USB media, go for the Desktop image and skip the next section

       https://ubuntu.com/download/desktop

### Expanding "server" edition to "desktop" (just add GUI):
```
sudo apt update
sudo apt upgrade
sudo apt install ubuntu-desktop
sudo apt install gdm3
sudo reboot now
locale
sudo apt install software-properties-common
```

### (...setup Samba, Chrony and other OS things here...).

Take a look at Raspberry Pi setup and choose what is relevant to your Desktop installation:

https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Ubuntu-RPi/README.md

## Installing ROS Jazzy Jalisco Desktop LTS:

https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html

### Install rqt and rqt_graph with all plugins:
```
sudo apt install ~nros-jazzy-rqt*
```

### for joystick operation - make sure your "ros" account has access to ports:
```
sudo adduser <your account> dialout
sudo apt install ros-jazzy-joy*
```

## Installing Gasebo Harmonic (using "Default Gazebo/ROS Pairing"):

https://gazebosim.org/docs/harmonic/ros_installation#installing-the-default-gazebo-ros-pairing
```
sudo apt-get install ros-${ROS_DISTRO}-ros-gz
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
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers

# this puts in place /opt/ros/jazzy/lib/libgz_ros2_control-system.so - a plugin for Gazebo:
sudo apt install ros-jazzy-gz-ros2-control

# some extras just in case:
sudo apt install ros-jazzy-joint-state-publisher ros-jazzy-joint-state-publisher-gui
```

### xacro processor and Joystick mixer:
```
sudo apt install ros-jazzy-xacro ros-jazzy-twist-mux
```

## Testing it all with my version of Articubot:

https://github.com/slgrobotics/robots_bringup

https://github.com/slgrobotics/articubot_one
```
cd
mkdir robot_ws
cd ~/robot_ws/
mkdir src
cd src
git clone https://github.com/slgrobotics/articubot_one.git
git clone https://github.com/joshnewans/twist_stamper.git
cd ..
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

## Bringing up Dragger robot simulation in Gazebo:
```
source /opt/ros/jazzy/setup.bash
source ~/robot_ws/install/setup.bash
ros2 launch articubot_one launch_sim.launch.py
```
You should see Gazebo and RViz GUI coming up. The simulated robot should respond to Joystick via teleop.

Make sure that your "enable" and "turbo" buttons are assigned correctly in ~/robot_ws/src/articubot_one/launch/joystick.launch.py

You must do "colcon build" in ~/robot_ws every time you change anything.

## Useful links

_Gazebo Harmonic - Differential Drive project template_ : https://gazebosim.org/docs/latest/ros_gz_project_template_guide

_and its GitHub repository_ : https://github.com/gazebosim/ros_gz_project_template/tree/main

_How to Use ROS2 Jazzy and Gazebo Harmonic for Robot Simulation_: https://www.youtube.com/watch?v=b8VwSsbZYn0

_ros2_control documentation :_ https://control.ros.org/jazzy/index.html

**Articulated Robotics (Josh Newans):**

https://articulatedrobotics.xyz/category/build-a-mobile-robot-with-ros

https://articulatedrobotics.xyz/mobile-robot-1-project-overview/

https://articulatedrobotics.xyz/mobile-robot-13-ros2-control-real/

https://www.youtube.com/@ArticulatedRobotics/videos

https://www.facebook.com/ArticulatedRobotics/

