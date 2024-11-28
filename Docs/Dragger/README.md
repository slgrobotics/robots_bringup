# Dragger Notes

Dragger photos are here: https://photos.app.goo.gl/eAdKiD7YYnL9Vou6A

Dragger is a *"larger Turtlebot"* - running all ROS2 nodes on-board. Only RViz2 runs on a desktop "ground station" computer. On board Dragger has a Raspberry Pi 5 ("dragger") [and an "FPV Drone" TV camera. - TBD].

Robot's Raspberry Pi 5 (8GB) runs sensors drivers (GPS, LD14 LIDAR and MPU9250), Differential Drive Control (inspired by Articulated Robotics), SLAM Toolkit and Nav2. Arduino Mega 2650 drives wheels and reports encoder readings over serial.

## Raspberry Pi 5 ("dragger") Build and Run Instructions

Dragger has _Ubuntu 24.04 Server (64 bit)_ and _ROS2 Jazzy Base_ installed - see https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Ubuntu-RPi for instructions.

Dragger has several Arduino boards, some drive the sensors - GPS, Ultrasonic Parking Sensor, IMU, - while the main Arduino Mega 2560 drives the wheels and combines all sensors data into a single serial stream for Raspberry Pi Dragger. This setup appeared historically through different experiments and at this time is mostly just an over-enginered legacy. The Dragger RPi makes full use of wheels driving ability and odometry info. Parking sensors data will be likely used later, as ROS2 supports rangers for obstacle avoidance and mapping.

Arduino Mega 2560 code - wheels/sensors driver, talking to Articulated Robotics ROS node: 

https://github.com/slgrobotics/Misc/tree/master/Arduino/Sketchbook/DraggerROS

Parking sensors info and driver (to be used later):

https://photos.app.goo.gl/WsqkA4XpYSLrVDX59

https://github.com/slgrobotics/Misc/tree/master/Arduino/Sketchbook/ParkingSensorI2C

MPU9250 and GPS Drivers come standard with ROS

### Dragger LD14 LIDAR setup

Follow this guide: https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/LD14.md

### GPS setup

Dragger has a _"BE-880 GPS Receiver Module with Flash HMC5883L Compass 10th Chip GPS Antenna"_, available on Amazon. A u-blox NEO-M10N module is part of it.

Follow this guide: https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/GPS.md

### Making USB devices persistent on Dragger

Dragger has three USB-to-Serial devices: Arduino "wheels/base", GPS and LIDAR.

Follow this guide: https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/USB.md

### MPU9250 Driver

see https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/MPU9250.md

## Installing additional navigation components

To allow GPS operation in sim install localization package:
```
sudo apt install ros-${ROS_DISTRO}-robot-localization ros-${ROS_DISTRO}-imu-tools ros-${ROS_DISTRO}-slam-toolbox
sudo apt install ros-${ROS_DISTRO}-navigation2 ros-${ROS_DISTRO}-nav2-bringup
sudo apt install ros-${ROS_DISTRO}-rmw-cyclonedds-cpp
```
More info - see "Useful Links" below.

### Dragger Differential Drive Control setup

See https://github.com/slgrobotics/diffdrive_arduino (inspired by Articulated Robotics)

See https://github.com/hiwad-aziz/ros2_mpu9250_driver

#### Note: Dragger Arduino Mega 2560 should be on /dev/ttyACM0

Prerequisites:
```
sudo apt install ros-${ROS_DISTRO}-ros2-control ros-${ROS_DISTRO}-ros2-controllers
sudo apt install ros-${ROS_DISTRO}-xacro
sudo apt install ros-${ROS_DISTRO}-twist-mux
sudo apt install libi2c-dev
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
ROS nodes produce _robot description_ and needs to  know some basic parameters of the robot. Edit the following to match your values:
```
~/robot_ws/src/articubot_one/description/ros2_control.xacro

           look for  <param name="enc_counts_per_rev">13730</param>

~/robot_ws/src/articubot_one/description/robot_core.xacro

           look for <xacro:property name="wheel_radius" value="0.192"/>
                    <xacro:property name="wheel_offset_y" value="0.290"/>
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
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
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

cd /home/ros/launch
source /opt/ros/jazzy/setup.bash
source /home/ros/robot_ws/install/setup.bash

ros2 launch /home/ros/robot_ws/src/articubot_one/launch/dragger.launch.py
```
Robot can be started with:
```
cd ~/launch
./bootup_launch.sh
```
There are two ways to control the robot **from a Desktop machine**: 
```
ros2 launch articubot_one launch_rviz.launch.py use_sim_time:=false
  -- or --
ros2 launch robots_bringup rviz_launch.py
```
As Nav2 is started on the robot with _autostart=false_ you need to click on _Startup_ button in RViz to start navigation.

If you choose to save maps (using _slam_toolkit_ RViz2 control) - the files will appear in _~/launch_ directory.

**Note:** See https://github.com/slgrobotics/robots_bringup/tree/main/Docs/ROS-Jazzy  for Desktop machine setup.

## Creating a Linux service for on-boot autostart

With Dragger base (Arduino wheels driver), Laser Scanner and IMU ROS2 nodes tested, it is time to set up autostart on boot for hands-free operation.

We need launch files (residing in this repository): _https://github.com/slgrobotics/articubot_one/tree/main/launch_

As we already cloned this repository, these files should be here: _~/robot_ws/src/articubot_one/launch_

1. Create and populate launch folder: _/home/ros/launch_ (see above)

You may edit the _bootup_launch.sh_ file to match your robot launch file and related folders.

Try running the _bootup_launch.sh_ from the command line to see if anything fails.

2. Create service description file - _/etc/systemd/system/robot.service_ :
```
#### /etc/systemd/system/robot.service
[Unit]
Description=robot
StartLimitIntervalSec=60
StartLimitBurst=5

[Service]
Type=simple
User=ros
Group=ros
WorkingDirectory=/home/ros/launch
ExecStart=/home/ros/launch/bootup_launch.sh
Restart=always

[Install]
WantedBy=multi-user.target
```

3. Enable service:
```
sudo systemctl daemon-reload
sudo systemctl enable robot.service
sudo systemctl start robot.service
```
If all went well, the service will start automatically after you reboot the RPi, and all related nodes will show up on _rpt_ and _rpt_graph_ on the Desktop.

Here are some useful commands:
```
systemctl status robot.service
systemctl cat robot.service
sudo systemctl reload-or-restart robot.service
sudo journalctl -xeu robot.service

sudo ls -al /etc/systemd/system/robot.service
sudo ls -al /etc/systemd/system/robot.service.d/override.conf
sudo ls -al /etc/systemd/system/multi-user.target.wants/robot.service
ps -ef | grep driver
```
You can now reboot Raspberry Pi, and the three drivers will start automatically. Nodes should show up in **rqt** and **rqt_graph**

**Back to main page:** https://github.com/slgrobotics/robots_bringup

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

