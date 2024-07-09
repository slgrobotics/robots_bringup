# ROS Jazzy "Clean Machine" - setup from scratch

## Installing Ubuntu 24.04 LTS ("Noble")

Install "server" from DVD

       https://ubuntu.com/download/server

Or, go for Desktop and skip this section (if your machine can boot up from USB media)

       https://ubuntu.com/download/desktop

### expanding "server" edition to "desktop":
```
sudo apt update
sudo apt upgrade
sudo apt install ubuntu-desktop
sudo apt install gdm3
sudo reboot now
locale
sudo apt install software-properties-common
```

### (setup Samba and other things here).

## installing ROS Jazzy Jalisco LTS:

https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html
```
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt install ros-dev-tools
sudo apt upgrade
sudo apt install ros-jazzy-desktop
```

### rqt and rqt_graph with all plugins:
```
sudo apt install ~nros-jazzy-rqt*
```

### for joystick operation (access to ports):
```
sudo adduser <your account> dialout
```

## Installing Gasebo Harmonic (using "Default Gazebo/ROS Pairing) ":

https://gazebosim.org/docs/harmonic/ros_installation#installing-the-default-gazebo-ros-pairing
```
sudo apt-get install ros-${ROS_DISTRO}-ros-gz
sudo apt-get install ros-${ROS_DISTRO}-gz-cmake-vendor
gz sim
```

### solving Gazebo crash on startup:
```
export QT_QPA_PLATFORM=xcb
gz sim
gz sim -v 4 shapes.sdf
```
now Gazebo UI can show up on a different machine (Windows with VcXsrv):
```
export DISPLAY=<machine>.local:0.0
```
Running Gnome on Windows with VcXsrv:
```
export DISPLAY=<machine>.local:0.0
gnome-shell --x11 --replace
```

## Installing Controller Manager and its infrastructure:
```
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers
# this puts in place /opt/ros/jazzy/lib/libgz_ros2_control-system.so :
sudo apt install ros-jazzy-gz-ros2-control
```

### xacro processor and Joystick mixer:
```
sudo apt install ros-jazzy-xacro
sudo apt install ros-jazzy-twist-mux
```

## Testing it all with my version of Articubot:

https://github.com/slgrobotics/robots_bringup

https://github.com/slgrobotics/articubot_one
```
cd
mkdir robot_ws
cd robot_ws/
mkdir src
cd src
git clone https://github.com/slgrobotics/diffdrive_arduino.git
git clone https://github.com/joshnewans/serial.git
git clone https://github.com/slgrobotics/articubot_one.git
cd ..
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

## Bringing up Dragger robot in Gazebo:

```
source /opt/ros/jazzy/setup.bash
source ~/robot_ws/install/setup.bash
ros2 launch articubot_one launch_sim.launch.py
```
