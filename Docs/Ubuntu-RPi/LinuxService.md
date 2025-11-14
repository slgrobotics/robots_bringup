## _Optional:_ Create a Linux service for on-boot autostart

**Tip:** this complicates making changes to robot software, use with caution. 

**Note:** This guide is written for _Create 1 Turtlebot_, but it applies to any other robot with minor changes.

See https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Create1

With _Create base_, _XV11 Laser Scanner_ and _BNO055 IMU_ ROS2 nodes tested, it is time to set up your Raspberry Pi to autostart on boot for hands-free operation.

We need some files - copy them from this repository, under and around https://github.com/slgrobotics/turtlebot_create/tree/main/RPi_Setup/launch

### Here are the steps:

1. Create and populate launch folder: /home/ros/launch
```
mkdir ~/launch
cd ~/launch
# place myturtle.py and bootup_launch.sh here:
cp ~/robot_ws/src/create_robot/create_bringup/launch/bootup_launch.sh .
cp ~/robot_ws/src/create_robot/create_bringup/launch/myturtle.py .
chmod +x ~/launch/bootup_launch.sh    
```
Try running the _bootup_launch.sh_ from the command line to see if anything fails.

2. Create service description file (as _"sudo"_) - _/etc/systemd/system/robot.service_ :
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
sudo systemctl enable robot.service
sudo systemctl start robot.service
```
If all went well, the service will start automatically after you reboot the RPi, and all related nodes will show up on _rpt_ and _rpt_graph_

**Note:** 
1. Logs are stored in _/home/ros/.ros/log_ folder - these can grow if things go wrong.
2. Raspberry Pi 3B is adequate for the _"headless"_ Turtlebot (except for the 14+ hours compilation) - it runs at <30% CPU load and low memory consumption.

### Here are some useful commands:
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
You can now reboot Raspberry Pi, and the three drivers will start automatically and show up in **rqt** and **rqt_graph** on the Desktop.

----------------

**Back to** [Wiki](https://github.com/slgrobotics/articubot_one/wiki) or [Docs Folder](https://github.com/slgrobotics/robots_bringup/tree/main/Docs)
