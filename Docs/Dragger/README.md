# Dragger Notes

Dragger photos are here: https://photos.app.goo.gl/eAdKiD7YYnL9Vou6A

Dragger is a *"larger Turtlebot"* - running all ROS2 nodes on-board. Only RViz2 runs on a desktop "ground station" computer. On board Dragger has a Raspberry Pi 5 ("dragger") [and an "FPV Drone" TV camera. - TBD].

The RPi runs sensors drivers (GPS, LD14 LIDAR and MPU9250), Differential Drive Control (inspired by Articulated Robotics), SLAM Toolkit and Nav2. Arduino Mega 2650 drives wheels and reports encoder readings over serial.

#### Articulated Robotics (by Josh Newans):

https://articulatedrobotics.xyz/category/build-a-mobile-robot-with-ros

https://articulatedrobotics.xyz/mobile-robot-1-project-overview/

https://articulatedrobotics.xyz/mobile-robot-13-ros2-control-real/

https://control.ros.org/jazzy/index.html

https://www.youtube.com/@ArticulatedRobotics/videos

https://www.facebook.com/ArticulatedRobotics/


### FPV Camera and receiver Setup

https://www.amazon.com/dp/B06VY7L1N4

https://www.amazon.com/dp/B07Q5MPC8V

Camera and transmitter, of course, resides on Dragger. The receiver, when plugged into a Ubuntu 22.04 **Desktop USB port**, shows up as _/dev/video0_ and _video1_

It works with Cheese app and can be read by Python/OpenCV scripts, including custom ROS nodes written in Python.

Here is the code I use for the camera **on the Desktop side**: https://github.com/slgrobotics/camera_publisher

Having the video link separated from WiFi and experiencing minimal delay allows driving the robot FPV-style and/or performing video stream processing on the Desktop.

## "dragger" Raspberry Pi 5 Build and Run Instructions

Dragger has _Ubuntu 24.04 Server (64 bit)_ and _ROS2 Jazzy Base_ installed - see https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Ubuntu-RPi for instructions.

Dragger has several Arduino boards, some drive the sensors - GPS, Ultrasonic Parking Sensor, IMU, - while the main Arduino Mega 2560 drives the wheels and combines all sensors data into a single serial stream for Raspberry Pi Dragger. This setup appeared historically through different experiments and at this time is mostly just an over-enginered legacy. The Dragger RPi makes full use of wheels driving ability and odometry info. Parking sensors data will be likely used later, as ROS2 supports rangers for obstacle avoidance and mapping.

Arduino Mega 2560 code - wheels/sensors driver, talking to Articulated Robotics ROS node: 

https://github.com/slgrobotics/Misc/tree/master/Arduino/Sketchbook/DraggerROS

Parking sensors info and driver (to be used later):

https://photos.app.goo.gl/WsqkA4XpYSLrVDX59

https://github.com/slgrobotics/Misc/tree/master/Arduino/Sketchbook/ParkingSensorI2C

MPU9250 and GPS Drivers come standard with ROS

### Making USB devices persistent on Dragger

Dragger has three USB-to-Serial devices: Arduino "wheels/base", GPS and LIDAR.

While the Arduino Mega is the only one at _/dev/ttyACM0_, GPS and LIDAR can take any name under _/dev/ttyUSB*_ pattern.

To avoid reassigning device names in the launch file after reboots, symlinks are created using the following recipe:

https://unix.stackexchange.com/questions/705570/setting-persistent-name-for-usb-serial-device-with-udev-rule-without-symlink
```
$ cat /etc/udev/rules.d/99-robot.rules
SUBSYSTEM=="tty",ENV{ID_PATH}=="platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0",SYMLINK+="ttyUSBLDR"
SUBSYSTEM=="tty",ENV{ID_PATH}=="platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.2:1.0",SYMLINK+="ttyUSBGPS"
```
After reboot, you should see something similar to this (which devices symbolic links point to is random):
```
$ ll /dev/ttyU*
crw-rw---- 1 root dialout 188, 0 Nov 21  2023 /dev/ttyUSB0
crw-rw---- 1 root dialout 188, 1 Nov 21  2023 /dev/ttyUSB1
lrwxrwxrwx 1 root root         7 Nov 21  2023 /dev/ttyUSBGPS -> ttyUSB1
lrwxrwxrwx 1 root root         7 Nov 21  2023 /dev/ttyUSBLDR -> ttyUSB0
```
Check the GPS stream:
```
sudo apt-get install picocom
picocom /dev/ttyUSBGPS -b 115200
```
Press _Ctrl_ button and then without releasing it press **a** and then **q**. It will exit the **picocom** application.

### Dragger LD14 LIDAR setup

Original code: https://github.com/ldrobotSensorTeam/ldlidar_sl_ros2    (Google Translate works here)

We will be using my fork https://github.com/slgrobotics/ldlidar_sl_ros2.git with minor corrections.

**Note:** Original code delivers various beam counts (378...393) between messages, and ROS (i.e. SLAM Toolbox) doesn't tolerate this.
So, I added code to ensure that the number of points in a scan is constant between LIDAR head revolutions.

LiDAR LD14 should appear on /dev/ttyUSB0

```
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src
git clone https://github.com/slgrobotics/ldlidar_sl_ros2.git
```

### GPS setup

Dragger has a _"BE-880 GPS Receiver Module with Flash HMC5883L Compass 10th Chip GPS Antenna"_, available on Amazon. A u-blox NEO-M10N module is part of it.

Features:
```
Chip: M10050
Frequency: GPS L1 C/A,QZSS L1 C/A/S,BDS B1I/B1C,Galileo E1B/C,SBAS L1 C/A:WAAS,EGNOS,MSAS,GAGAN
Operation Mode: GPS+BDS+GALILEO+SBAS+QZSS
Sensitivity: Track -166dBm, Re-arrest -160dBm, Cold Start -148dBm, Hot Start -160dBm
Horizontal Accuracy: 1.5m CEP 2D RMS SBAS Auxiliary(for open sky)
Speed Accuracy: 0.05m/s
Dynamic Heading Angle Accuracy: 0.3 deg
1PPS Time Accuracy: RMS 30ns, 99% 60ns
Start Time: Cold Start 27s, Hot Start 1s, Assisted Start 1s
Baud Rate: 4800bps - 921600bps, default 38400bps
Output Level: TTL level
Output Protocol: NMEA, UBX
NMEA Sentences: RMC, VTG, GGA, GSA, GSV, GLL
Update Frequency: 0.25Hz - 18Hz, default 1Hz
Second Pulse: Configurable from 0.25Hz to 10MHz, default period 1s, high level last for 100ms
Graviational Acceleration: <4g
Voltage: DC 3.6V - 5.5V, Typical 5.0V
Current: Normal 50mA/5.0V
Size: 28*28*11mm
Connector: 1.25mm 6pin
```
Default baud rate is 38400, and it works out of the box at 1 Hz.

You can use _u-center_ Windows app to set up the device initially, saving settings to its Flash memory. Specifically, set NMEA at 115200 baud at 10 Hz to be streaming on USB. Check it:
```
picocom /dev/ttyUSBGPS -b 115200

(Use Ctrl/A + Ctrl/X to exit)
```
We need to install standard ROS Jazzy support for NMEA messages:
```
[Ubuntu 22.04 only]:
sudo pip3 install transforms3d
[Ubuntu 24.04 only]:
sudo apt install python3-transforms3d

sudo apt install ros-${ROS_DISTRO}-nmea-navsat-driver

The following NEW packages will be installed:
  ros-jazzy-nmea-msgs ros-jazzy-nmea-navsat-driver ros-jazzy-tf-transformations
```
The driver code is here ("ros2" branch) - look into _launch_ and _config_ folders:

https://github.com/ros-drivers/nmea_navsat_driver/blob/ros2/config/nmea_serial_driver.yaml

GPS Node will be run as part of the _dragger.launch.py_ process.

>> [NOTE] The process described here didn't work with my NEO-M8N device:
>>
>> https://docs.fictionlab.pl/leo-rover/integrations/positioning-systems/ublox-evk-m8n
>> ```
>> sudo apt install ros-${ROS_DISTRO}-ublox
>> 
>> The following NEW packages will be installed:
>>   libasio-dev ros-jazzy-ublox ros-jazzy-ublox-gps ros-jazzy-ublox-msgs ros-jazzy-ublox-serialization
>> ```
>> Code is here: https://github.com/KumarRobotics/ublox/blob/ros2/README.md
>>
>>  There's a lot of chatter on the Internet about this problem.

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

### Run the robot (on-board Raspberry 5)

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

**GPS - localization and navigation:**

https://automaticaddison.com/sensor-fusion-using-the-robot-localization-package-ros-2/

https://docs.ros.org/en/melodic/api/robot_localization/html/integrating_gps.html

https://docs.nav2.org/tutorials/docs/navigation2_with_gps.html

https://index.ros.org/p/robot_localization/#jazzy

