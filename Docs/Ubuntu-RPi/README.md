# This is how to set up your Raspberry Pi 3B (or better)

Once you set up ROS2-Desktop Humble on your desktop computer (see **Desktop_Setup folder**) - you can now set up your on-board Raspberry Pi machine.

We need to set up a Ubuntu 22.04 Server 64 bit, install ROS2 Humble, compile drivers for Create 1, Laser scanner, BNO055 IMU and create a Linux service for on-boot autostart.

### 1. Set up a Ubuntu 22.04 Server 64 bit

Follow this guide, selecting Ubuntu 22.04.1 LTS Server 64 bit:

    https://docs.ros.org/en/humble/How-To-Guides/Installing-on-Raspberry-Pi.html

    login:  ubuntu / ubuntu

You need to make sure that your network is set up, better use Ethernet cable initially, for WiFi - see /etc/netplan/*

Create a "ros" account with sudo privileges. Use ros account for further setup and work.

    sudo adduser ros sudo
    sudo adduser ros dialout

    hostnamectl set-hostname turtle

It is a good idea to update:

    sudo apt update
    sudo apt upgrade
    sudo apt full-upgrade
    sudo apt clean    (purges packages from SD   https://www.raspberrypi.org/forums/viewtopic.php?f=66&t=133691)

You may want to disable unattended updates to avoid high CPU loads after booting:

    sudo dpkg-reconfigure unattended-upgrades      (say No)

Have some extra networking packages installed:

    sudo apt install raspi-config
    sudo apt install winbind samba smbclient net-tools wget
    sudo apt install python3-pip

You should be able to ping your "turtle.local" machine and ssh into it ("ssh ros@turtle.local" from your Desktop machine).

    sudo apt install i2c-tools
    sudo i2cdetect -y 1

Set up Python GPIO: https://ubuntu.com/tutorials/gpio-on-raspberry-pi#1-overview

    sudo apt install python3-lgpio
    sudo pip3 install RPi.GPIO

Set up WiringPi GPIO: https://projects.drogon.net/raspberry-pi/wiringpi/download-and-install/

    sudo apt install build-essential
    mkdir tmp
    cd tmp
    git clone https://github.com/WiringPi/WiringPi.git
    cd WiringPi
    bash ./build
    gpio -h
    sudo gpio readall
    
You may need to set up a swap file to compensate for small RAM on RPi 3B:

    sudo swapon --show     (if nothing shows up, swap isn't set up - https://www.linuxtut.com/en/71e3874cb83ed12ec405/)
    sudo fallocate -l 2G /swapfile
    sudo chmod 600 /swapfile
    sudo mkswap /swapfile
    sudo swapon /swapfile    (now "top" shows MiB Swap: 2048.0)
    echo '/swapfile swap swap defaults 0 0' | sudo tee -a /etc/fstab

Samba creates shared folder, accessible from Windows machines:

    sudo mkdir /home/shared; sudo chmod og+rwx /home/shared   (+x matters for readability)
    sudo usermod -aG sambashare ros
    cp  /etc/samba/smb.conf .
    vi smb.conf  <- add Samba share - see below
    sudo cp smb.conf /etc/samba/smb.conf
    sudo systemctl restart smbd

Samba share - add to the end of smb.conf, use any Share name you like, e.g "ShareAll"
```
[ShareAll]
   path = /home/shared
   read only = no
   public = yes
   writable = yes
   guest ok = yes
   guest only = yes
   force create mode = 777
   force directory mode = 777
```

### 2. Continue with ROS2 installation

Follow these guides, selecting binary ROS2-Base Humble ("Bare bones" ros-humble-ros-base). Also, install Development tools:

    https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

    https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html

for "ros" account:

    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

Try some basic things from Tutorials:

    sudo apt install ros-humble-demo-nodes-py
    sudo apt install ros-humble-demo-nodes-cpp

now you can run these commands in separate terminals (even across your LAN machines):

    ros2 run demo_nodes_cpp talker
    ros2 run demo_nodes_py listener

### 3. Compile ROS2 driver for Create 1 base

For iRobot Create 1 (released in 2004, based on Roomba 500 series) and other roombas of 400, 500 and 600 series (https://en.wikipedia.org/wiki/IRobot_Create).

It can be connected via FTDI USB Serial (/dev/ttyUSB0 on turtle) or via TTL Serial (pins 1-RXD and 2-TXD on the DB25 connector). A desktop machine can connect via RS232 serial, using special iRobot Create serial cable (/dev/ttyS0 on desktop).

Generally, we follow this guide:

    https://github.com/girvenavery2022/create_robot/tree/galactic
    
Review the following. Create 1 requires analog gyro, connected to pin 4 of its Cargo Bay DB25:

    https://github.com/AutonomyLab/create_robot/issues/28
    https://github.com/slgrobotics/create_robot/tree/foxy
    https://github.com/slgrobotics/libcreate
    https://github.com/slgrobotics/Misc/tree/master/Arduino/Sketchbook/MPU9250GyroTurtlebot

here are all commands:

    mkdir -p ~/create_robot_ws/src
    cd ~/create_robot_ws/src
    git clone https://github.com/slgrobotics/create_robot.git --branch foxy
    git clone https://github.com/slgrobotics/libcreate.git
    colcon build

    cd ~/create_robot_ws
    vi src/create_robot/create_bringup/config/default.yaml    (edit port - /dev/ttyS0 on desktop, /dev/ttyUSB0 on turtle)
    colcon build

    source ~/create_ws/install/setup.bash
    ros2 launch create_bringup create_1.launch
        or, for Roomba 500/600 series:
    ros2 launch create_bringup create_2.launch
    
At this point you should be able to use teleop **from your desktop machine:**

(skip it if you don't have a joystick on the desktop machine)

Joystick teleop friendly blog:

    https://articulatedrobotics.xyz/mobile-robot-14a-teleop/

To test joystick:

	ros2 run joy joy_enumerate_devices
	ros2 run joy joy_node      # <-- Run in first terminal
	ros2 topic echo /joy       # <-- Run in second terminal

    https://index.ros.org/p/teleop_twist_joy/github-ros2-teleop_twist_joy/

**__Note:__** *joy_node sends cmd_vel messages ONLY when enable_button is pressed (Usually btn 1)
	you MUST set enable_button to desired value (0 for btn 1, the "front trigger").
	ros2 param get /teleop_twist_joy_node enable_button  - to see current value*

**On your desktop machine:**

    sudo apt-get install ros-humble-teleop-twist-joy

    ros2 launch teleop_twist_joy teleop-launch.py     - also runs joy_node
    ros2 param set /teleop_twist_joy_node enable_button 0   - in separate terminal

To make joystick startup more convenient, create a shell script:

```
$ cat teleop.sh

ros2 launch teleop_twist_joy teleop-launch.py &
sleep 3
ros2 param set /teleop_twist_joy_node enable_button 0
ros2 param set /teleop_twist_joy_node enable_turbo_button 3
```

### 4. Compile ROS2 driver for Laser scanner

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

Rviz **on your desktop machine** needs at least a static transform, to relate the grid to the laser frame ("neato_laser" in this case).

    rviz2 &
    ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map neato_laser &

For Rviz you need:

    Global Options to have Fixed Frame set to a known TF ("map")
  
    Grid reference frame - set to "map"
  
    Add LaserScan, topic "/scan", Style :Spheres" size 0.02
  
    ros2 run tf2_ros tf2_echo map neato_laser            -- to see published TF


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


### 6. Create a Linux service for on-boot autostart

With Create base, XV11 Laser Scanner and BNO055 IMU ROS2 nodes tested, it is time to set up autostart on boot for hands-free operation.

We need some files (copy them from this repository, under and around https://github.com/slgrobotics/turtlebot_create/tree/main/RPi_Setup/launch):

1. Create and populate launch folder: /home/ros/launch
```
mkdir ~/launch
 -- place myturtle.py and bootup_launch.sh here --
chmod +x ~/launch/bootup_launch.sh    
```
Try running the _bootup_launch.sh_ from the command line to see if anything fails.

2. Create service description file - /etc/systemd/system/robot.service :
```
# /etc/systemd/system/robot.service
[Unit]
Description=turtle
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
sudo systemctl enable turtle.service
sudo systemctl start turtle.service
```
If all went well, the service will start automatically after you reboot the RPi, and all related nodes will show up on _rpt_ and _rpt_graph_

Here are some useful commands:
```
systemctl status turtle.service
systemctl cat turtle.service
sudo systemctl reload-or-restart turtle.service
sudo journalctl -xeu turtle.service

sudo ls -al /etc/systemd/system/turtle.service
sudo ls -al /etc/systemd/system/turtle.service.d/override.conf
sudo ls -al /etc/systemd/system/multi-user.target.wants/turtle.service
ps -ef | grep driver
```
You can now reboot Raspberry Pi, and the three drivers will start automatically and show up in **rqt** and **rqt_graph**

**Now you can proceed to Desktop (Turtle_Setup) folder:**   https://github.com/slgrobotics/turtlebot_create/tree/main/Turtle_Setup
