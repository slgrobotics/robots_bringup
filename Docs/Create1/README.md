**Note:**
- Familiarity with (now outdated) [Old README](https://github.com/slgrobotics/turtlebot_create/blob/main/README.md) is highly recommended.
- For the legacy RPi 3B version refer to [this guide](https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Create1_RPi3B).

## Turtlebot Create 1 with Raspberry Pi 4/8Gb (ROS2 Jazzy)

My Create 1 now has a Raspberry Pi 4/8Gb ("turtle").
The RPi runs sensors drivers (XV11 LIDAR and BNO055 IMU), [Autonomy Lab _base_ code](https://github.com/slgrobotics/create_robot), and the usual localization and Nav2 nodes. 
The code (inspired by Articulated Robotics) is [here](https://github.com/slgrobotics/articubot_one) (*dev* branch is usually more current).

On the Desktop just run RViz2 as described below.

**Note:** You can just run Turtle robot in Gazebo sim on a Desktop machine (no robot hardware required) - 
refer to [this section](https://github.com/slgrobotics/robots_bringup/blob/main/Docs/ROS-Jazzy/README.md#build-articubot_one-robot-codebase)

#### Optional: Use REAL-TIME kernel on Raspberry Pi 4

Refer to [this guide](https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Ubuntu-RPi/UbuntuRealTime.md).

#### Optional: Overclocking Raspberry Pi 4 to 2 GHz

Here is a script to see clock-related status of RPi 4 (you need to `sudo usermod -a -G video $(whoami)`)
```
#!/bin/bash
echo "CPU: " `gawk '{print $1/1000," C"}' /sys/class/thermal/thermal_zone0/temp`
echo "GPU: " `vcgencmd measure_temp` | sed 's/temp=//'
lscpu|grep Hz
cpu_khz=`cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq`
result=$(echo "scale=2; $cpu_khz / 1000000" | bc -l)
echo "Current CPU Clock:                    $result GHz"
#result=$((cpu_khz / 1000))
#echo "Current CPU Clock: $result MHz"
vcgencmd get_throttled
vcgencmd measure_volts
free -h
#
# Note: for ongoing clock monitoring
#        watch -n 1 "vcgencmd measure_clock arm"
```
Most RPi 4's can run at 2 GHz with a minimal heat sink.

See https://forums.raspberrypi.com/viewtopic.php?t=313280 for detailed info.

**Warning:** When I had "*arm_freq_min=100*" my overclocked RPi 4's were showing wrong "*date*" - system clock has slowed to a crawl.
Chrony was running, but not synching. So - keep *arm_freq_min* at 600. See [this](https://github.com/MichaIng/DietPi/issues/4455).

To enable overclocking, edit (sudo) */boot/firmware/config.txt* - the "[pi4]" section should look like this:
```
[pi4]
max_framebuffers=2
arm_boost=1
initial_turbo=60
#hdmi_enable_4kp60=1
over_voltage=6
arm_freq_min=600
arm_freq=2000
#gpu_freq=750
#gpu_mem=256
```

### Turtle Raspberry Pi 4/8Gb Build and Run Instructions:

Turtle has _Ubuntu 24.04 Server (64 bit)_ and _ROS2 Jazzy Base_ installed - see https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Ubuntu-RPi for instructions.

#### XV11 LIDAR setup

Refer to https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/XV11_LIDAR.md

Alternatively, use LD14 LIDAR: https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/LD14.md

#### ROS2 driver for BNO055 9DOF IMU

Refer to https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/BNO055%20IMU.md

Alternatively, use MPU9250 sensor: https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/MPU9250.md

#### Compile forked Autonomy Lab ROS2 driver for Create 1 base

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
# Using "dev" or "main" branch:
git clone https://github.com/slgrobotics/articubot_one.git -b dev

# you may edit Create Base default port here - */dev/ttyS0* if on desktop, */dev/ttyUSB0* on turtle
#              (it will be set as a parameter in the launch file anyway):
vi ~/robot_ws/src/create_robot/create_bringup/config/default.yaml
```
Your *src* folder should look like this:
```
ros@turtle:~/robot_ws$ ls src
articubot_one  bno055  create_robot  libcreate  xv_11_driver
```
Continue with _rosdep_:
```
cd ~/robot_ws
### Note: See https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html
# sudo rosdep init     -- do it once
rosdep update
# this will take a while, many additional packages installed:
rosdep install --from-paths src --ignore-src -r -y
```
For the build on RPi 4/2Gb/4Gb you may limit number of parallel threads, no need to do it for RPi 5 8GB:
```
export MAKEFLAGS="-j 1"
colcon build --parallel-workers=1 --executor sequential
```
Or, just build it on an RPi 4/8Gb (build will take about 5-7 minutes, less on a RPi 5):
```
colcon build
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
[INFO] [launch]: All log files can be found below /home/ros/.ros/log/2025-11-03-06-58-11-525224-turtle-80979
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [create_driver-1]: process started with pid [80983]
[INFO] [robot_state_publisher-2]: process started with pid [80984]
[robot_state_publisher-2] [INFO] [1762174692.789162930] [robot_state_publisher]: Robot initialized
[create_driver-1] [INFO] [1762174692.795454666] [create_driver]: [CREATE] gyro_offset: 0    gyro_scale: 1    distance_scale: 1
[create_driver-1] [INFO] [1762174692.796039697] [create_driver]: [CREATE] "CREATE_1" base model selected
[create_driver-1] [INFO] [1762174693.814961240] [create_driver]: [CREATE] Connection established.
[create_driver-1] [INFO] [1762174693.815369771] [create_driver]: [CREATE] Battery level 47.52 %
[create_driver-1] [INFO] [1762174693.882721650] [create_driver]: [CREATE] Ready.
```
You can see appropriate ROS2 topics appear in _rqt_, among them:
- */battery/battery_state*
- */joint_states*
- */odom*

**Tip:** Any time you need to produce a robot URDF from ```.xacro``` files, use "_xacro_" command, for example:
```
source ~/robot_ws/install/setup.bash
xacro ~/robot_ws/install/articubot_one/share/articubot_one/robots/turtle/description/robot.urdf.xacro sim_mode:=true > /tmp/robot.urdf
```

#### Populate _launch_ folder: _/home/ros/launch_
```
mkdir ~/launch
cd ~/launch
# place bootup_launch.sh here:
cp ~/robot_ws/src/articubot_one/robots/turtle/launch/bootup_launch.sh .
chmod +x bootup_launch.sh
# You might need a helper program to adjust "*gyro_offset*":
cp ~/robot_ws/src/articubot_one/robots/turtle/launch/roomba.py .
```
Now, when you need to run the on-board nodes on the robot using SSH, just type:
```
cd ~/launch
./bootup_launch.sh
```
### Optional: Test-driving Create 1 Turtlebot with _teleop_

Check out https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/Joystick.md

Keep in mind, that Create 1 base expects */diff_cont/cmd_vel* - while joystick without tweaking produces */cmd_vel_joy*

You may want to temporarily modify *joystick.launch.py* for this test.

### _Optional:_ Create a Linux service for on-boot autostart

With _Create base_, _XV11 Laser Scanner_ and _BNO055 IMU_ ROS2 nodes tested, it is time to set up autostart on boot for hands-free operation.

See https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Ubuntu-RPi/LinuxService.md

## On the Desktop:

Once you have Turtlebot nodes running, it is time to run _RViz_ **on the Desktop**:
```
cd ~/robot_ws
colcon build
ros2 launch articubot_one launch_rviz.launch.py use_sim_time:=false
```
The command above also runs _joystick_ node, so that you can drive the robot using _Xbox 360 Controller_ or _Logitech Gamepad F710_.

Consult [running-a-physical-robot](https://github.com/slgrobotics/robots_bringup/tree/main/Docs/ROS-Jazzy#running-a-physical-robot)

## Tuning your Gyro (only for Create 1)

If you have an original *Create 1* base - it needs a gyro to compensate for a firmware bug (see https://github.com/AutonomyLab/create_robot/issues/28).

Any analog gyro will do. The original accessory interface board which plugs into the Cargo Bay DB25 connector has an analog gyro, ADXR613. 

I had to create an _"analog gyro emulator"_ by connecting arduino mini to MPU9250 - and producing the same analog signal via PWM (https://github.com/slgrobotics/Misc/tree/master/Arduino/Sketchbook/MPU9250GyroTurtlebot).

*Create base* does not read the gyro (or gyro emulator) directly - it just passes it through from the analog input (DB25 pin 4 of Cargo Bay) to the serial stream (which sends all sensor data every 15ms).

When stationary, the gyro output will produce an ADC value between 0 and 1024, hopefully around 512, and that value will vary with rotation (reflecting, naturally, robot's turn rate).

Autonomy Lab *Create driver* [with my modifications](https://github.com/slgrobotics/create_robot) reads this value, adds gyro_offset and multiplies it by gyro_scale - and then integrates it (by dt) to produce turn angle.

https://github.com/slgrobotics/libcreate/blob/master/src/create.cpp : 148
```
// This is a fix involving analog gyro connected to pin 4 of Cargo Bay:
uint16_t angleRaw = GET_DATA(ID_CARGO_BAY_ANALOG_SIGNAL);
float angleF = -((float)angleRaw - 512.0 + getGyroOffset()) * dt;
angleF = angleF * 0.25 * getGyroScale(); // gyro calibration factor
//std::cout<< "dt: " << dt << " distanceRaw: " << distanceRaw << " angleRaw: " << angleRaw << " angleF: " << angleF << std::endl;
deltaYaw = angleF * (util::PI / 180.0); // D2R
```

Create driver needs *angle* to correctly publish *diff_cont/odom* topic, which is important for robot localization as it moves.
Correct wheel joints rotation is the best indication of normal operation of odometry calculations.

You will need to calibrate your gyro, and adjust related parameters in Turtle [launch file](https://github.com/slgrobotics/articubot_one/blob/dev/robots/turtle/launch/turtle.launch.py).

**Tuning gyro_offset, gyro_scale and distance_scale**

There are three parameters in Turtle [launch file](https://github.com/slgrobotics/articubot_one/blob/dev/robots/turtle/launch/turtle.launch.py).
By adjusting them you make odometry (reported by *Create base [driver](https://github.com/slgrobotics/create_robot/blob/jazzy/create_driver/src/create_driver.cpp)*) work properly.
```
'gyro_offset': 0.0,     - compensates for gyro drift
'gyro_scale': 1.19,     - adjusts gyro sensitivity in turns
'distance_scale': 1.02  - adjusts for wheel encoders discrepancy
```
First, **compensate for _gyro dift_** - make sure that the ROS2 gyro driver can adjust it to zero when Turtle is stationary.

The "*Cargo Bay Analog Signal*", as read by *Create 1 base* on DB25 pin 4, is connected (in our case) to gyro output.

The ROS2 [driver](https://github.com/slgrobotics/create_robot) expects it to be 512 when the robot is stationary.
If it differs (say, reads 202 when robot doesn't move) - the `'gyro_offset'` must compensate for that (say, 512-202=310 - the 310 is your `'gyro_offset'`).

Read it using an autonomous helper program:
```
ros@turtle:~$ cd ~/launch; python3 roomba.py

Sending Reset
bl-start
Sending Start
Flushing serial
b'2006-09-12-1137-L   \r\nRDK by iRobot!\r\nMC9S12E128\r\n2006-11-20-1731-L   \r\nbattery-current-quiescent-raw 530  battery-current-zero 516\r\n'
Reading DB25 pin 4:
202
201
202
```

Adjust it accordingly in the launch file.

**Note:** as I am using a [gyro emulator](https://github.com/slgrobotics/Misc/tree/master/Arduino/Sketchbook/MPU9250GyroTurtlebot), my reading is actually quite stable at "512". Yours may be different.

With *gyro drift* compensated, proceed to **calibrating gyro sensitivity** (a.k.a. *gyro turn rate*).

Start Turtle in ROS2. You need to bring up Rviz2 to see odometry vector. The best way is to follow "On the Desktop" section above.

The turn rate scale, as reported by gyro, usually needs adjustment. Drive the robot using joystick - turn it 360 degrees and see if the Odometry vector (showing *diff_cont/odom*) is lagging behind or gaining over the robot's orientation. Adjust `'gyro_scale'` to have them match.

The *distance_scale* can be adjusted so that *diff_cont/odom* **pose** reports proper distance when robot is driven forward or backward.

As a **final test**, you need to drive the robot forward a couple meters and watch the odom point in Rviz to stay at the launch point.
Adjust the RViz as shown in the screenshot below - showing *odom* and *map* frames.
Then turn the robot and watch the *odom* point move. You should strive for minimal *odom* displacement from *map* during straight runs and rotations:

<img width="2133" height="1176" alt="Screenshot from 2025-11-06 13-57-44" src="https://github.com/user-attachments/assets/bd486e40-1d16-418b-b4e1-9bee1a1084b3" />

Once the parameters are adjusted, your robot will be able to map the area, and the _odom_ point will not move drastically when the robot drives and turns in any direction.

----------------

**Back to** [Wiki](https://github.com/slgrobotics/articubot_one/wiki) or [Docs Folder](https://github.com/slgrobotics/robots_bringup/tree/main/Docs)
