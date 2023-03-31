# Plucky Notes

```
pluck·y /'pl?ke/ adjective
having or showing determined courage in the face of difficulties.
Synonyms of plucky: SPIRITED, BRAVE
```
Plucky photos are here: https://photos.app.goo.gl/YdYQ8kQrNmLkVXTM7

Plucky is a *"larger Turtlebot"* - running, basically, standard ROS2 Turtlebot 3 binaries for navigation (on the desktop "ground station" computer). Onboard it has two Raspberry Pi 3B ("plucky" and "pluckysens") and an "FPV Drone" TV camera. One RPi runs sensors drivers (LD14 LIDAR and MPU9250), the other - Differential Drive Control, inspired by Articulated Robotics.

## Useful Links:

Articulated Robotics (Josh Newans):

https://articulatedrobotics.xyz/mobile-robot-1-project-overview/

https://articulatedrobotics.xyz/mobile-robot-13-ros2-control-real/

https://articulatedrobotics.xyz/mobile-robot-full-list/

https://www.facebook.com/ArticulatedRobotics/


## Build and Run Instructions:

Plucky has several Arduino boards, some drive sensors - GPS, Ultrasonic Parking Sensor, MPU, - while the main Arduino Mega 2560 drives the wheels and combines all sensors data into a single serial stream for Raspberry Pi "plucky". This setup appeared historically through different experiments and at this time is mostly just an over-enginered legacy. "pluckysens" RPi has its own MPU9250, and GPS isn't useful indoors. But "plucky" RPi makes full use of wheels driving ability and odometry info. Parking sensors data will be likely used later, as ROS2 supports rangers for obstacle avoidance and mapping.

Arduino Mega code - wheels/sensors driver: 

https://github.com/slgrobotics/Misc/tree/master/Arduino/Sketchbook/PluckyWheelsROS

Parking sensors info and driver:

TBD

GPS Driver:

TBD

Arduino MPU9250 Driver:

TBD

### "pluckysens" Raspberry Pi 3B code (ROS2 LD14 LIDAR and MPU9250 Drivers):

```
mkdir -p ~/plucky_ws/src
cd ~/plucky_ws/src
// LiDAR LD14 (on /dev/ttyUSB0):
git clone  https://github.com/ldrobotSensorTeam/ldlidar_sl_ros2.git

// For rviz2: time shift corrections :  /home/ros/plucky_ws/src/ldlidar_sl_ros2/src/demo.cpp
// Line 24:
#define TIME_SHIFT_SEC 0
#define TIME_SHIFT_NSEC 400000000

// Lines 204, 310 correct:
  output.header.stamp = start_scan_time - rclcpp::Duration(TIME_SHIFT_SEC, TIME_SHIFT_NSEC);
  //output.header.stamp = start_scan_time;

git clone https://github.com/hiwad-aziz/ros2_mpu9250_driver.git
vi ~/plucky_ws/src/ros2_mpu9250_driver/src/mpu9250driver.cpp   - line 48:   message.header.frame_id = "imu_link"; (was "base_link")

cd ~/plucky_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build

### Note: Plucky Arduino Mega 2560 should be on /dev/ttyACM0
```

### "plucky" Raspberry Pi 3B code (ROS2 Differential Drive Control):

https://github.com/slgrobotics/diffdrive_arduino

Prerequisites: 
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers

```
mkdir -p ~/plucky_ws/src
cd ~/plucky_ws/src
git clone https://github.com/slgrobotics/diffdrive_arduino.git
git clone https://github.com/joshnewans/serial.git
git clone https://github.com/slgrobotics/articubot_one.git

cd ~/plucky_ws
### Note: See https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html
rosdep update
rosdep install --from-paths src --ignore-src -r -y
### Note: build takes ~10 minutes
colcon build

### Note: Plucky Arduino Mega 2560 on /dev/ttyACM0
```
