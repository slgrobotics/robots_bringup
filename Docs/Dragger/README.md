# Dragger Notes
```
/ˈdrægər/ Other forms: draggers. Definitions of dragger. noun.
 someone who pulls or tugs or drags in an effort to move something.
 well, this robot drags his casters across the lawn...
```

Dragger photos are here: https://photos.app.goo.gl/eAdKiD7YYnL9Vou6A

If you want to browse robot's code look here: https://github.com/slgrobotics/articubot_one

Dragger is a *"larger Turtlebot"* - running all ROS2 nodes on-board. Only RViz2 runs on a desktop "ground station" computer. On board Dragger has a Raspberry Pi 5 ("dragger") [and an "FPV Drone" TV camera. - TBD].

Robot's Raspberry Pi 5 (8GB) runs sensors drivers (GPS, LD14 LIDAR and MPU9250), Differential Drive Control (inspired by Articulated Robotics), SLAM Toolkit and Nav2. Arduino Mega 2650 drives wheels and reports encoder readings over serial.

**Note:** You can just run Dragger robot in Gazebo sim on a Desktop machine (no robot hardware required) - 
refer to [this section](https://github.com/slgrobotics/robots_bringup/blob/main/Docs/ROS-Jazzy/README.md#build-articubot_one-robot-codebase)

## Raspberry Pi 5 ("dragger") Build and Run Instructions

Dragger has _Ubuntu 24.04 Server (64 bit)_ and _ROS2 Jazzy Base_ installed - see https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Ubuntu-RPi for instructions.

Dragger and Plucky both have two Arduino boards, one drives the Ultrasonic Parking Sensor - while the main Arduino Mega 2560 drives the wheels and combines all sensors data into a single serial stream for Raspberry Pi Dragger.

Dragger and Plucky use parking sensor (four ultrasonic rangers) for obstacle avoidance and mapping, converting its data to *sensor_msgs/msg/Range* messages.

Arduino Mega 2560 code - wheels/sensors driver, talking to Articulated Robotics ROS node: 

https://github.com/slgrobotics/Misc/tree/master/Arduino/Sketchbook/DraggerROS

Parking sensor info and driver:

https://photos.app.goo.gl/WsqkA4XpYSLrVDX59

https://github.com/slgrobotics/Misc/tree/master/Arduino/Sketchbook/ParkingSensorI2C

MPU9250 and GPS Drivers come standard with ROS

### LD14 LIDAR setup

Follow this guide: https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/LD14.md

### GPS setup

Dragger has a _"BE-880 GPS Receiver Module with Flash HMC5883L Compass 10th Chip GPS Antenna"_, available on Amazon. A u-blox NEO-M10N module is part of it.

Follow this guide: https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/GPS.md

### Making USB devices persistent

Dragger has three USB-to-Serial devices: Arduino "wheels/base", GPS and LIDAR.

Follow this guide: https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/USB.md

### MPU9250 Driver

See https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/MPU9250.md

## Installing additional navigation components

To allow GPS operation in sim install localization package:
```
sudo apt install ros-${ROS_DISTRO}-robot-localization ros-${ROS_DISTRO}-imu-tools ros-${ROS_DISTRO}-slam-toolbox
sudo apt install ros-${ROS_DISTRO}-navigation2 ros-${ROS_DISTRO}-nav2-bringup ros-${ROS_DISTRO}-rmw-cyclonedds-cpp
```
More info - see "Useful Links" below.

### Differential Drive Control setup

See https://github.com/slgrobotics/diffdrive_arduino (inspired by Articulated Robotics)

See https://github.com/hiwad-aziz/ros2_mpu9250_driver

#### Note: Arduino Mega 2560 should be on /dev/ttyACM0

Prerequisites:
```
sudo apt install ros-${ROS_DISTRO}-ros2-control ros-${ROS_DISTRO}-ros2-controllers
sudo apt ros-${ROS_DISTRO}-range-sensor-broadcaster
sudo apt install ros-${ROS_DISTRO}-xacro ros-${ROS_DISTRO}-twist-mux libi2c-dev
sudo adduser ros dialout
```
Now, to the business:
```
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src
git clone https://github.com/slgrobotics/diffdrive_arduino.git
git clone https://github.com/joshnewans/serial.git
git clone https://github.com/slgrobotics/articubot_one.git
```
Dragger, Plucky and any other robot using _[diffdive_arduino](https://github.com/slgrobotics/diffdrive_arduino)_ package need a _BatteryStateBroadcaster_ component.
In my code it is configured to use all 9 available State interfaces, not only "voltage" available in official distribution at this time.

Please follow [this guide](https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/BatteryStateBroadcaster.md) to install modified version,
until the [official distribution](https://github.com/ipa320/ros_battery_monitoring/pull/3) catches up with latest changes and you could just install it using _apt_:
```
git clone https://github.com/slgrobotics/ros_battery_monitoring.git
```
ROS nodes produce _robot description_ and needs to  know some basic parameters of the robot. Edit the following to match your values, for example:
```
~/robot_ws/src/articubot_one/robots/dragger/description/ros2_control.xacro

           look for  <param name="enc_counts_per_rev">13730</param>

~/robot_ws/src/articubot_one/robots/dragger/description/robot_core.xacro

           look for <xacro:property name="wheel_radius" value="0.192"/>
                    <xacro:property name="wheel_offset_y" value="0.290"/>

~/robot_ws/src/articubot_one/robots/dragger/config/controllers.yaml

           look for "wheel_separation" and "wheel_radius", "Velocity and acceleration limits"
```
### Environment variables setup

Make sure that the "tail" of your _~/.bashrc_ file looks like this:
```
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
```

### Build and deploy robot's components

```
cd ~/robot_ws
sudo rosdep init    # do it once, if you haven't done it before
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -r -y
colcon build
```
**Note:** For _rosdep_ see https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Rosdep.html

### Run the robot (on-board Raspberry 5 8GB)

For convenience, create and populate _launch_ directory:
```
mkdir ~/launch
cp ~/robot_ws/src/articubot_one/launch/bootup_launch.sh ~/launch/.
chmod +x ~/launch/bootup_launch.sh    
```
The script looks like this:
```
#!/bin/bash

source /opt/ros/jazzy/setup.bash

cd /home/ros/robot_ws
colcon build
cd /home/ros/launch

source /home/ros/robot_ws/install/setup.bash

ros2 launch /home/ros/robot_ws/src/articubot_one/robots/dragger/launch/dragger.launch.py
```
Robot can be started with:
```
cd ~/launch
./bootup_launch.sh
```
On the Desktop, run the following:
```
cd ~/robot_ws
source ~/robot_ws/install/setup.bash
colcon build; ros2 launch articubot_one launch_rviz.launch.py use_sim_time:=false
```
For diagnostics, run *rqt* and *rqt_graph*

If you choose to save maps (using _slam_toolkit_ RViz2 control) - the files will appear in _~/launch_ directory.

**Note:** See https://github.com/slgrobotics/robots_bringup/tree/main/Docs/ROS-Jazzy  for Desktop machine setup.

### _Optional:_ Create a Linux service for on-boot autostart

With _Dragger_ ROS2 nodes tested, you can set up autostart on boot for hands-free operation.

Follow this guide: https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Ubuntu-RPi/LinuxService.md

-------------------------

**Back to main page:** https://github.com/slgrobotics/robots_bringup
