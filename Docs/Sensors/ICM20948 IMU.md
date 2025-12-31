**Back to** [Wiki](https://github.com/slgrobotics/articubot_one/wiki)

## ROS2 driver for ICM20948 IMU

SparkFun: "...The [ICM-20948](https://www.sparkfun.com/sparkfun-9dof-imu-breakout-icm-20948-qwiic.html) is an extremely low-powered, I2C and SPI-enabled 9-axis motion tracking device..."

Also, GY-ICM20948 is available from [Amazon](https://www.amazon.com/5pcs-GY-ICM20948V2-ICM20948-9-DOF-Sensor/dp/B0G4MN81T4).

### Connections:

Connect to Raspberry Pi  **I2C**: **SCL** - pin 05, **SDA** - pin 03 **VCC** - pin 01 (3.3V) **GND** - pin 06 or 09

RPi GPIO header pinout: https://www.raspberrypi.com/documentation/computers/images/GPIO-Pinout-Diagram-2.png

_Optional:_ Consider 3 KOhm to 6.8 KOhm pull-up resistors from SDA and SCL to 3.3 V bus.

Use ```i2cdetect -y 1``` to see **address 0x68** or **0x69**

### Info and tests

Sensor information: https://www.sparkfun.com/sparkfun-9dof-imu-breakout-icm-20948-qwiic.html

My fork of ROS2 driver code is here: https://github.com/slgrobotics/ros2_icm20948

### Trying it (on Raspberry Pi)

First, clone the repository:
```
mkdir -p ~/tmp_ws/src
cd ~/tmp_ws/src
git clone https://github.com/slgrobotics/ros2_icm20948
```
install dependencies:
```
cd ~/tmp_ws; rosdep install --from-paths src --ignore-src -r -y
```
Make sure your device responds on address 0x69 (SparkFun) or 0x68 (generic GY-ICM20948):
```
$ i2cdetect -y 1
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:                         -- -- -- -- -- -- -- -- 
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
60: -- -- -- -- -- -- -- -- 68 -- -- -- -- -- -- -- 
70: -- -- -- -- -- -- -- --                         
```
adjust parameters, if needed:
```
vi ~/tmp_ws/src/ros2_icm20948/launch/icm20948_node_launch.py
```
build it:
```
cd ~/tmp_ws
colcon build
```
Run the sensor node:
```
$ colcon build; ros2 launch ros2_icm20948 icm20948_node_launch.py
...
[INFO] [icm20948_node-1]: process started with pid [26690]
[icm20948_node-1] [INFO] [1767203297.007680023] [icm20948_node]: IP: ICM20948 IMU Sensor node has been started
[icm20948_node-1] [INFO] [1767203297.008591936] [icm20948_node]:    i2c_addr: 0x68
[icm20948_node-1] [INFO] [1767203297.009334793] [icm20948_node]:    frame_id: imu_link
[icm20948_node-1] [INFO] [1767203297.010039020] [icm20948_node]:    pub_rate: 200 Hz
[icm20948_node-1] [INFO] [1767203297.010877303] [icm20948_node]:    madgwick_beta: 0.08
[icm20948_node-1] [INFO] [1767203297.011447215] [icm20948_node]:    madgwick_use_mag: True
[icm20948_node-1] [INFO] [1767203300.978213904] [icm20948_node]:    accel_fsr=3 mul=0.0047884 m/s^2 per LSB, gyro_fsr=3 mul=0.00106423 rad/s per LSB
[icm20948_node-1] [INFO] [1767203301.088602318] [icm20948_node]: OK: ICM20948 Node: init successful
```
You should see the `/imu/data` topic in *rqt* and can display publishing rate:
```
ros2 topic hz /imu/data
average rate: 200.040
	min: 0.005s max: 0.005s std dev: 0.00004s window: 202
```
To see IMU orientation in RViz2, you need to:
- bring up RViz2 (IMU is already setup in my RViz config):
```
ros2 launch articubot_one launch_rviz.launch.py use_sim_time:=false
```
- run static transformer node to relate frames, raising *IMU frame* 1 meter up on the *map*:
```
ros2 run tf2_ros static_transform_publisher 0 0 1 0 0 0 map imu_link
```
Enable the *IMU checkbox* and adjust topic to show `/imu_data`:

<img width="1387" height="625" alt="Screenshot from 2025-12-31 12-46-55" src="https://github.com/user-attachments/assets/c2657cb1-19fb-4ea2-bad1-7b7a42067470" />

### Running ICM20948 node on the robot and tuning EKF filter

EKF [filter](https://github.com/slgrobotics/articubot_one/blob/main/launch/ekf_odom.launch.py) fuses wheels odometry with IMU orientation data,
deriving reliable orientation quaternion in turns and straight runs (topic *"odometry/local"*).

It is very important to have EKF filter tuned properly, as shown [here](https://github.com/slgrobotics/articubot_one/blob/main/robots/seggy/config/ekf_odom_params.yaml), for example.

More EKF tuning tips [here](https://chatgpt.com/s/t_691b80a57e588191b1528a238588b87a).

### Real-time monitoring of IMU orientation data

Install [PlotJuggler](https://plotjuggler.io) to monitor IMU data:
```
sudo apt install ros-${ROS_DISTRO}-plotjuggler-ros
```
Run *PlotJugger* as a ROS2 process. It subscribes to topic's values (i.e. "*imu_data*" *yaw*) and shows them in real time:
```
ros2 run plotjuggler plotjuggler
```
**Tips:**
- drag the desired value ("yaw") from the left panel to the main (plot) panel. Right-click on the plot to adjust *Min*, *Max* etc.
- make sure that the EKF output *yaw* follows IMU *yaw* without spikes or jumps.
- IMU *frame* in RViz should smoothly follow the robot, red axis looking forward, green - left and blue - up:

<img width="907" height="881" alt="Screenshot from 2025-11-21 11-42-32" src="https://github.com/user-attachments/assets/422fb7be-42c8-46bc-a3c3-850f0cb7a479" />

### Useful Links

Sensor available here: https://www.amazon.com/5pcs-GY-ICM20948V2-ICM20948-9-DOF-Sensor/dp/B0G4MN81T4 

or from SparkFun: https://www.sparkfun.com/sparkfun-9dof-imu-breakout-icm-20948-qwiic.html

----------------

**Back to** [Wiki](https://github.com/slgrobotics/articubot_one/wiki) or [Docs Folder](https://github.com/slgrobotics/robots_bringup/tree/main/Docs)
