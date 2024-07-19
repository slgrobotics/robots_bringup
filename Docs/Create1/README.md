# Turtlebot Create 1 Notes (Jazzy version)

This is an updated and streamlined version of the prior document ( https://github.com/slgrobotics/turtlebot_create/tree/main/RPi_Setup ) - adjusted for ROS Jazzy.

Familiarity with https://github.com/slgrobotics/turtlebot_create/blob/main/README.md is highly recommended.

My Create 1 has a Raspberry Pi 3B ("turtle"). The RPi runs sensors drivers (XV11 LIDAR and BNO055), and Differential Drive Control, inspired by Articulated Robotics.

## Useful Links:

Articulated Robotics (Josh Newans):

https://articulatedrobotics.xyz/category/build-a-mobile-robot-with-ros

https://articulatedrobotics.xyz/mobile-robot-1-project-overview/

https://articulatedrobotics.xyz/mobile-robot-13-ros2-control-real/

https://control.ros.org/humble/index.html

https://www.youtube.com/@ArticulatedRobotics/videos

https://www.facebook.com/ArticulatedRobotics/

## Turtle Raspberry Pi 3B Build and Run Instructions:

Turtle has _Ubuntu 24.04 Server 64 bit_ and _ROS2 Jazzy Base_ installed - https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html

### "Turtle" XV11 LIDAR setup:

Surreal XV Lidar controller v1.2 (Neato Lidar) - connected via USB

    https://github.com/getSurreal/XV_Lidar_Controller  - Teensy software
    https://www.getsurreal.com/product/lidar-controller-v2-0/   - hardware (Teensy 2.0)

Connect to the USB port at 115200 baud. (minicom -D /dev/ttyACM0 -b 115200)

ROS2 driver port (by Mark Johnston): https://github.com/mjstn/xv_11_driver

The following file needs editing (as  declare_parameter() now requires a default value as a second parameter):

    /home/sergei/xv_11_ws/src/xv_11_driver/src/xv_11_driver.cpp

```
    int main(int argc, char * argv[])
    {
      rclcpp::init(argc, argv);

      auto node = rclcpp::Node::make_shared("xv11_laser");

      node->declare_parameter("port",XV11_PORT_DEFAULT);
      auto port_param      = rclcpp::Parameter("port", XV11_PORT_DEFAULT);

      node->declare_parameter("baud_rate", XV11_BAUD_RATE_DEFAULT);
      auto baud_rate_param = rclcpp::Parameter("baud_rate", XV11_BAUD_RATE_DEFAULT);

      node->declare_parameter("frame_id", XV11_FRAME_ID_DEFAULT);
      auto frame_id_param  = rclcpp::Parameter("frame_id", XV11_FRAME_ID_DEFAULT);

      node->declare_parameter("firmware_version", XV11_FIRMWARE_VERSION_DEFAULT);
      auto firmware_param  = rclcpp::Parameter("firmware_version", XV11_FIRMWARE_VERSION_DEFAULT);
```

Commands to compile and install:

    mkdir -p ~/xv_11_ws/src
    cd ~/xv_11_ws/src
    git clone https://github.com/mjstn/xv_11_driver.git
    
      (edit the xv_11_driver/src/xv_11_driver.cpp here - also define XV11_PORT_DEFAULT as /dev/ttyACM0)

    cd ..
    colcon build
    source ~/xv_11_ws/install/setup.bash
    ros2 run xv_11_driver xv_11_driver &

```
### 5. Compile ROS2 driver for BNO055 IMU

Connect to I2C: SCL - pin 05, SDA - pin 03 of Raspberry Pi

Info and tests:

    https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor
    https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/webserial-visualizer
    https://hackmd.io/@edgesense/ryzAq3IFs

BNO055 IMU (via UART or I2C - Python) - seems well supported, active development, ROS2 node

    https://github.com/flynneva/bno055

    mkdir -p ~/bno055_ws/src
    cd ~/bno055_ws/src/
    git clone https://github.com/flynneva/bno055.git
    cd ..
    colcon build
    vi ~/bno055_ws/src/bno055/bno055/params/bno055_params_i2c.yaml   - change i2c_bus to 1. Use i2cdetect -y 1
    sudo pip3 install smbus
 
 Try running it, see IMU messages in rqt:
 
    source ~/bno055_ws/install/setup.bash
    ros2 run bno055 bno055  --ros-args --params-file ~/bno055_ws/src/bno055/bno055/params/bno055_params_i2c.yaml


### "turtle" Differential Drive Control setup:

See https://github.com/slgrobotics/diffdrive_arduino (inspired by Articulated Robotics)

See https://github.com/hiwad-aziz/ros2_mpu9250_driver

#### Note: turtle _Create 1 base_ should be on /dev/ttyACM0

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
git clone https://github.com/joshnewans/twist_stamper.git

# BNO055 Driver:
git clone https://github.com/hiwad-aziz/ros2_mpu9250_driver.git

```
ROS nodes produce _robot description_ and needs to  know some basic parameters of the robot. Edit the following to match Create 1 values:
```
~/robot_ws/src/articubot_one/description/ros2_control.xacro

           look for  <param name="enc_counts_per_rev">13730</param>

~/robot_ws/src/articubot_one/description/robot_core.xacro

           look for <xacro:property name="wheel_radius" value="0.192"/>
                    <xacro:property name="wheel_offset_y" value="0.290"/>
```
Now you can build and deploy robot's components:
```
cd ~/robot_ws
### Note: See https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build

```
Now we need to put it all together, the same way the Create 1 Turtlebot has been set up here: https://github.com/slgrobotics/turtlebot_create/tree/main/RPi_Setup

### Create a Linux service for on-boot autostart

With turtle base (Arduino wheels driver), Laser Scanner and IMU ROS2 nodes tested, it is time to set up autostart on boot for hands-free operation.

We need launch files (residing in this repository): https://github.com/slgrobotics/articubot_one/tree/main/launch

As we already cloned this repository, these files should be here: ~/robot_ws/src/articubot_one/launch

1. Create and populate launch folder: /home/ros/launch
```
mkdir ~/launch
cp ~/robot_ws/src/articubot_one/launch/turtle.launch.py ~/launch/.
cp ~/robot_ws/src/articubot_one/launch/bootup_launch.sh ~/launch/.
chmod +x ~/launch/bootup_launch.sh    
```
You must edit the _bootup_launch.sh_ file to match your robot launch file and related folders:
```
#!/bin/bash

cd /home/ros/launch
source /opt/ros/humble/setup.bash
source /home/ros/robot_ws/install/setup.bash

ros2 launch /home/ros/launch/turtle.launch.py
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
