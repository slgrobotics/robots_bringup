**Back to** [Wiki](https://github.com/slgrobotics/articubot_one/wiki)

**Note:**
- this is a legacy document, related to my previous version of Turtle (*Create 1* with Raspberry Pi 3B).
For related code refer to any snapshot dated before November 1, 2025 (for example, Commit [34c7d2c](https://github.com/slgrobotics/articubot_one/commit/34c7d2ca4b689123a60aec02c53af94c93cc77d3)).
- Currently my Turtle robot has *Raspberry Pi 4/8GB* and related notes are [here](https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Create1).

# Turtlebot Create 1 with Raspberry Pi 3B Notes (Jazzy version)

This is an updated and streamlined version of the prior document ( [RPi_Setup](https://github.com/slgrobotics/turtlebot_create/tree/main/RPi_Setup) ) - adjusted for ROS Jazzy.

Familiarity with (now outdated) [Old README](https://github.com/slgrobotics/turtlebot_create/blob/main/README.md) is highly recommended.

My Create 1 has a Raspberry Pi 3B ("turtle"). The RPi runs sensors drivers (XV11 LIDAR and BNO055 IMU), and [Autonomy Lab _base_ code](https://github.com/slgrobotics/create_robot). 

The rest of the robot nodes run on the Desktop, using my *articubot_one* codebase, inspired by Articulated Robotics.

**Note:** You can just run Turtle robot in Gazebo sim on a Desktop machine (no robot hardware required) - 
refer to [this section](https://github.com/slgrobotics/robots_bringup/blob/main/Docs/ROS-Jazzy/README.md#build-articubot_one-robot-codebase)

## Turtle Raspberry Pi 3B Build and Run Instructions:

Turtle has _Ubuntu 24.04 Server (64 bit)_ and _ROS2 Jazzy Base_ installed - see https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Ubuntu-RPi for instructions.

### XV11 LIDAR setup

Refer to https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/XV11_LIDAR.md

Alternatively, use LD14 LIDAR: https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/LD14.md

### ROS2 driver for BNO055 9DOF IMU

Refer to https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/BNO055%20IMU.md

Alternatively, use MPU9250 sensor: https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/MPU9250.md

### Compile forked Autonomy Lab ROS2 driver for Create 1 base

For iRobot Create 1 (released in 2004, based on Roomba 500 series) and other roombas of 400, 500 and 600 series (https://en.wikipedia.org/wiki/IRobot_Create).

It can be connected via FTDI USB Serial (_/dev/ttyUSB0_ on turtle) or via TTL Serial (pins 1-RXD and 2-TXD on the DB25 connector). A desktop machine can connect via RS232 serial, using special iRobot Create serial cable (/dev/ttyS0 on desktop).

Generally, we follow this guide:

https://github.com/girvenavery2022/create_robot/tree/galactic
    
Review the following. Create 1 requires analog gyro, connected to pin 4 of its Cargo Bay DB25:

https://github.com/AutonomyLab/create_robot/issues/28

https://github.com/slgrobotics/create_robot   (branch _"jazzy"_ is default there, forked from upstream _"iron"_)

https://github.com/slgrobotics/libcreate

https://github.com/slgrobotics/Misc/tree/master/Arduino/Sketchbook/MPU9250GyroTurtlebot

Here are all commands on the Raspberry Pi:
```
# mkdir -p ~/robot_ws/src
cd ~/robot_ws/src
git clone https://github.com/slgrobotics/create_robot.git
git clone https://github.com/slgrobotics/libcreate.git

# you may edit Create Base port here - /dev/ttyS0 if on desktop, /dev/ttyUSB0 on turtle:
vi ~/robot_ws/src/create_robot/create_bringup/config/default.yaml

cd ~/robot_ws
### Note: See https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html
# sudo rosdep init     -- do it once
rosdep update
# this will take a while, many additional packages installed:
rosdep install --from-paths src --ignore-src -r -y

# On an RPi 3B build will take VERY long time (over 14 hours in my case) and needs at least 2GB swap space.
# For RPi 3B and 4 you must limit number of parallel threads, no need to do it for RPi 5 8GB:
export MAKEFLAGS="-j 1"
colcon build --parallel-workers=1 --executor sequential
(or, just "colcon build" on a Raspberry Pi 4 or 5 - which takes minutes)
```
This is how it looked on my Raspberry Pi 3B with 1 GB RAM:

**Note:** use HDMI monitor and USB keyboard, as SSH will be interrupted on RPi 3 for the lack of resources.
```
ros@turtle:~/robot_ws$ export MAKEFLAGS="-j 1"
ros@turtle:~/robot_ws$ colcon build --parallel-workers=1 --executor sequential
[5.063s] WARNING:colcon.colcon_ros.prefix_path.ament:The path '/home/ros/robot_ws/install/create_robot'
                  in the environment variable AMENT_PREFIX_PATH doesn't exist
...
Starting >>> bno055
Finished <<< bno055 [14.3s]
Starting >>> create_description
Finished <<< create_description [2.29s]
Starting >>> create_msgs
Finished <<< create_msgs [6min 3s]
Starting >>> libcreate
Finished <<< libcreate [7min 48s]
Starting >>> xv_11_driver
Finished <<< xv_11_driver [4min 24s]
Starting >>> create_driver
Finished <<< create_driver [14h 14min 39s]
Starting >>> create_bringup
Finished <<< create_bringup [16.5s]
Starting >>> create_robot
Finished <<< create_robot [10.9s]

Summary: 8 packages finished [14h 33min 43s]
ros@turtle:~/robot_ws$
```
Test it on _turtle_:
```
source ~/robot_ws/install/setup.bash
ros2 launch create_bringup create_1.launch
    or, for Roomba 500/600 series:
ros2 launch create_bringup create_2.launch
```
This is how the robot comes up on my screen:
```
ros@turtle:~/robot_ws$ ros2 launch create_bringup create_1.launch
[INFO] [launch]: All log files can be found below /home/ros/.ros/log/2024-07-23-09-11-31-032285-turtle-26928
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [create_driver-1]: process started with pid [26934]
[INFO] [robot_state_publisher-2]: process started with pid [26935]
[robot_state_publisher-2] [INFO] [1721743897.733288931] [robot_state_publisher]: Robot initialized
[create_driver-1] [INFO] [1721743897.817175928] [create_driver]: [CREATE] gyro_offset: 0    gyro_scale: 1
[create_driver-1] [INFO] [1721743897.818827061] [create_driver]: [CREATE] "CREATE_1" selected
[create_driver-1] [INFO] [1721743898.881177255] [create_driver]: [CREATE] Connection established.
[create_driver-1] [INFO] [1721743898.881650532] [create_driver]: [CREATE] Battery level 100.00 %
[create_driver-1] [INFO] [1721743899.054224697] [create_driver]: [CREATE] Ready.
```
### Populate _launch_ folder: _/home/ros/launch_
```
mkdir ~/launch
cd ~/launch
# place myturtle.py and bootup_launch.sh here:
cp ~/robot_ws/src/create_robot/create_bringup/launch/bootup_launch.sh .
cp ~/robot_ws/src/create_robot/create_bringup/launch/myturtle.py .
# You might need a helper program to adjust "gyro_offset":
cp ~/robot_ws/src/create_robot/create_bringup/launch/roomba.py .
chmod +x ~/launch/bootup_launch.sh    
```
Now, when you need to run the on-board nodes on the robot using SSH, just type:
```
cd ~/launch
./bootup_launch.sh
```
## Test-driving Create 1 Turtlebot with _teleop_

Check out https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/Joystick.md

Keep in mind, that Create 1 base expects */diff_cont/cmd_vel* - while joystick without tweaking produces */cmd_vel_joy*

You may want to temporarily modify *joystick.launch.py* for this test.

## _Optional:_ Create a Linux service for on-boot autostart

With _Create base_, _XV11 Laser Scanner_ and _BNO055 IMU_ ROS2 nodes tested, it is time to set up autostart on boot for hands-free operation.

See https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Ubuntu-RPi/LinuxService.md

## On the Desktop:

Once you have Turtlebot on-board nodes (hardware "drivers") running, it is time to run the rest of the nodes and RViz on the Desktop.

Consult [running-a-physical-robot](https://github.com/slgrobotics/robots_bringup/tree/main/Docs/ROS-Jazzy#running-a-physical-robot)

## Tuning your Gyro (only for Create 1)

refer to this [guide](https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Create1#tuning-your-gyro-only-for-create-1).

----------------

**Back to** [Wiki](https://github.com/slgrobotics/articubot_one/wiki) or [Docs Folder](https://github.com/slgrobotics/robots_bringup/tree/main/Docs)
