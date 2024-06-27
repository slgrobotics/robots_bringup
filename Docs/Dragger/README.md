# Dragger Notes

Dragger photos are here: https://photos.app.goo.gl/eAdKiD7YYnL9Vou6A

Dragger is a *"larger Turtlebot"* - running, basically, standard ROS2 Turtlebot 3 binaries for navigation (on the desktop "ground station" computer). Onboard it has a Raspberry Pi 4 ("dragger") [and an "FPV Drone" TV camera. - TBD] The RPi runs sensors drivers (GPS, LD14 LIDAR and MPU9250), and Differential Drive Control, inspired by Articulated Robotics.

## Useful Links:

Articulated Robotics (Josh Newans):

https://articulatedrobotics.xyz/mobile-robot-1-project-overview/

https://articulatedrobotics.xyz/mobile-robot-13-ros2-control-real/

https://articulatedrobotics.xyz/mobile-robot-full-list/

https://www.facebook.com/ArticulatedRobotics/


## Build and Run Instructions:

Dragger has several Arduino boards, some drive sensors - GPS, Ultrasonic Parking Sensor, IMU, - while the main Arduino Mega 2560 drives the wheels and combines all sensors data into a single serial stream for Raspberry Pi "dragger". This setup appeared historically through different experiments and at this time is mostly just an over-enginered legacy. The "dragger" RPi makes full use of wheels driving ability and odometry info. Parking sensors data will be likely used later, as ROS2 supports rangers for obstacle avoidance and mapping.

Arduino Mega 2560 code - wheels/sensors driver, talking to Articulated Robotics ROS node: 

https://github.com/slgrobotics/Misc/tree/master/Arduino/Sketchbook/DraggerROS

Parking sensors info and driver (to be used later):

https://photos.app.goo.gl/WsqkA4XpYSLrVDX59

https://github.com/slgrobotics/Misc/tree/master/Arduino/Sketchbook/ParkingSensorI2C

MPU9250 and GPS Drivers come standard with ROS

FPV Setup [TBD]:

https://www.amazon.com/dp/B06VY7L1N4

https://www.amazon.com/dp/B07Q5MPC8V

The receiver above, when plugged into a Ubuntu 22.04 Desktop USB port, shows up as /dev/video0 and video1

It works with Cheese app and can be read by Python/OpenCV scripts, including custom ROS nodes written in Python.

A sample will be published here later.

### "dragger" Raspberry Pi 4 sensors setup (ROS2 LD14 LIDAR and MPU9250 Drivers):

https://github.com/ldrobotSensorTeam/ldlidar_sl_ros2    (Google Translate works here)

https://github.com/hiwad-aziz/ros2_mpu9250_driver

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

git clone https://github.com/hiwad-aziz/ros2_mpu9250_driver.git
vi ~/robot_ws/src/ros2_mpu9250_driver/src/mpu9250driver.cpp   - line 48:   message.header.frame_id = "imu_link"; (was "base_link")

cd ~/robot_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build

### Note: Dragger Arduino Mega 2560 should be on /dev/ttyACM0
```

### Continue "dragger" Raspberry Pi 4 setup (ROS2 Differential Drive Control):

https://github.com/slgrobotics/diffdrive_arduino

Prerequisites: 
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers

```
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src
git clone https://github.com/slgrobotics/diffdrive_arduino.git
git clone https://github.com/joshnewans/serial.git
git clone https://github.com/slgrobotics/articubot_one.git

cd ~/robot_ws
### Note: See https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html
rosdep update
rosdep install --from-paths src --ignore-src -r -y
### Note: build takes ~10 minutes
colcon build

### Note: Dragger Arduino Mega 2560 on /dev/ttyACM0
```
Now we need to put it all together, the same way the Create 1 Turtlebot has been set up here: https://github.com/slgrobotics/turtlebot_create/tree/main/RPi_Setup

### Create a Linux service for on-boot autostart

With Dragger base (Arduino wheels driver), Laser Scanner and IMU ROS2 nodes tested, it is time to set up autostart on boot for hands-free operation.

We need launch files (residing in this repository): https://github.com/slgrobotics/articubot_one/tree/main/launch

As we already cloned this repository, these files should be here: ~/robot_ws/src/articubot_one/launch

1. Create and populate launch folder: /home/ros/launch
```
mkdir ~/launch
cp ~/robot_ws/src/articubot_one/launch/dragger.py ~/launch/.
cp ~/robot_ws/src/articubot_one/launch/bootup_launch.sh ~/launch/.
chmod +x ~/launch/bootup_launch.sh    
```
You must edit the _bootup_launch.sh_ file to match your robot launch file and related folders.

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
