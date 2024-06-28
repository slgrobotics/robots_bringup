# Dragger Notes

Dragger photos are here: https://photos.app.goo.gl/eAdKiD7YYnL9Vou6A

Dragger is a *"larger Turtlebot"* - running, basically, standard ROS2 Turtlebot 3 binaries for navigation (on the desktop "ground station" computer). Onboard it has a Raspberry Pi 4 ("dragger") [and an "FPV Drone" TV camera. - TBD] The RPi runs sensors drivers (GPS, LD14 LIDAR and MPU9250), and Differential Drive Control, inspired by Articulated Robotics.

## Useful Links:

Articulated Robotics (Josh Newans):

https://articulatedrobotics.xyz/mobile-robot-1-project-overview/

https://articulatedrobotics.xyz/mobile-robot-13-ros2-control-real/

https://articulatedrobotics.xyz/mobile-robot-full-list/

https://www.facebook.com/ArticulatedRobotics/


### FPV Camera and receiver Setup [TBD]:

https://www.amazon.com/dp/B06VY7L1N4

https://www.amazon.com/dp/B07Q5MPC8V

Camera and transmitter, of course, resides on Dragger. The receiver, when plugged into a Ubuntu 22.04 **Desktop USB port**, shows up as _/dev/video0_ and _video1_

It works with Cheese app and can be read by Python/OpenCV scripts, including custom ROS nodes written in Python.

Here is the code I use for the camera **on the Desktop side**: https://github.com/slgrobotics/camera_publisher

Having the video link separated from WiFi and experiencing minimal delay allows driving the robot FPV-style and/or performing video stream processing on the Desktop.

## Dragger Raspberry Pi 4 Build and Run Instructions:

Dragger has Ubuntu 22.04 Server 64 bit and ROS2 Humble Base installed - https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

Dragger has several Arduino boards, some drive the sensors - GPS, Ultrasonic Parking Sensor, IMU, - while the main Arduino Mega 2560 drives the wheels and combines all sensors data into a single serial stream for Raspberry Pi "dragger". This setup appeared historically through different experiments and at this time is mostly just an over-enginered legacy. The "dragger" RPi makes full use of wheels driving ability and odometry info. Parking sensors data will be likely used later, as ROS2 supports rangers for obstacle avoidance and mapping.

Arduino Mega 2560 code - wheels/sensors driver, talking to Articulated Robotics ROS node: 

https://github.com/slgrobotics/Misc/tree/master/Arduino/Sketchbook/DraggerROS

Parking sensors info and driver (to be used later):

https://photos.app.goo.gl/WsqkA4XpYSLrVDX59

https://github.com/slgrobotics/Misc/tree/master/Arduino/Sketchbook/ParkingSensorI2C

MPU9250 and GPS Drivers come standard with ROS

### "dragger" LD14 LIDAR setup:

See https://github.com/ldrobotSensorTeam/ldlidar_sl_ros2    (Google Translate works here)
```
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src
// LiDAR LD14 (on /dev/ttyUSB0):
git clone  https://github.com/ldrobotSensorTeam/ldlidar_sl_ros2.git

// For rviz2: time shift corrections :  /home/ros/robot_ws/src/ldlidar_sl_ros2/src/demo.cpp
// Line 24:
#define TIME_SHIFT_SEC 0
#define TIME_SHIFT_NSEC 400000000

// Lines 204, 310 correct:
  output.header.stamp = start_scan_time - rclcpp::Duration(TIME_SHIFT_SEC, TIME_SHIFT_NSEC);
  //output.header.stamp = start_scan_time;

```
### "dragger" Differential Drive Control setup:

See https://github.com/slgrobotics/diffdrive_arduino (inspired by Articulated Robotics)

See https://github.com/hiwad-aziz/ros2_mpu9250_driver

#### Note: Dragger Arduino Mega 2560 should be on /dev/ttyACM0

Prerequisites:
```
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
sudo apt install ros-humble-xacro
sudo apt install ros-humble-twist-mux
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

# MPU9250 Driver:
git clone https://github.com/hiwad-aziz/ros2_mpu9250_driver.git
vi ~/robot_ws/src/ros2_mpu9250_driver/src/mpu9250driver.cpp   - line 48:   message.header.frame_id = "imu_link"; (was "base_link")

cd ~/robot_ws
### Note: See https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build

```
Now we need to put it all together, the same way the Create 1 Turtlebot has been set up here: https://github.com/slgrobotics/turtlebot_create/tree/main/RPi_Setup

### Create a Linux service for on-boot autostart

With Dragger base (Arduino wheels driver), Laser Scanner and IMU ROS2 nodes tested, it is time to set up autostart on boot for hands-free operation.

We need launch files (residing in this repository): https://github.com/slgrobotics/articubot_one/tree/main/launch

As we already cloned this repository, these files should be here: ~/robot_ws/src/articubot_one/launch

1. Create and populate launch folder: /home/ros/launch
```
mkdir ~/launch
cp ~/robot_ws/src/articubot_one/launch/dragger.launch.py ~/launch/.
cp ~/robot_ws/src/articubot_one/launch/bootup_launch.sh ~/launch/.
chmod +x ~/launch/bootup_launch.sh    
```
You must edit the _bootup_launch.sh_ file to match your robot launch file and related folders:
```
#!/bin/bash

cd /home/ros/launch
source /opt/ros/humble/setup.bash
source /home/ros/robot_ws/install/setup.bash

ros2 launch /home/ros/launch/dragger.launch.py
```
Try running the _bootup_launch.sh_ from the command line to see if anything fails.

2. Create service description file - /etc/systemd/system/robot.service :
```
# /etc/systemd/system/robot.service
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
If all went well, the service will start automatically after you reboot the RPi, and all related nodes will show up on _rpt_ and _rpt_graph_

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
You can now reboot Raspberry Pi, and the three drivers will start automatically. Nodes (at least _robot_state_publisher_) should show up in **rqt** and **rqt_graph**

**Now you can proceed to Desktop setup (all Desktop operations are the same for all Turtlebot3 based robots):** https://github.com/slgrobotics/robots_bringup
