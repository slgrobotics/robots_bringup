**Back to** [Wiki](https://github.com/slgrobotics/articubot_one/wiki)

## ROS2 driver for ICM20948 IMU

[ICM20948](https://www.ceva-ip.com/wp-content/uploads/BNO080_085-Datasheet.pdf) "...integrates a triaxial accelerometer,
triaxial gyroscope, magnetometer and a 32-bit ARM® Cortex™-M0+ microcontroller running CEVA's SH-2
firmware. The SH-2 includes the *MotionEngine™* software, which provides sophisticated signal processing
algorithms to process sensor data and provide precise real-time 3D orientation, heading, calibrated acceleration
and calibrated angular velocity..."

### Connections:

Connect to Raspberry Pi  **I2C**: **SCL** - pin 05, **SDA** - pin 03

RPi GPIO header pinout: https://www.raspberrypi.com/documentation/computers/images/GPIO-Pinout-Diagram-2.png

_Optional:_ Consider 3 KOhm to 6.8 KOhm pull-up resistors from SDA and SCL to 3.3 V bus.

Use ```i2cdetect -y 1``` to see **address 0x68** or **0x69**

### Info and tests

Information: https://www.adafruit.com/product/4754

Datasheet: https://www.ceva-ip.com/wp-content/uploads/BNO080_085-Datasheet.pdf

My fork of ROS2 driver code is here: https://github.com/slgrobotics/bno08x_ros2_driver
(original [here](https://github.com/bnbhat/bno08x_ros2_driver))

**Note:** the original driver does not publish orientation covariences (or any other),
which causes extreme confusion of the EKF filter, which fuses wheels odometry and UMU.
What happens and why it matters is explained [here](https://chatgpt.com/s/t_691b60f38e1c8191a0a309cbcf99e478).
I created an issue [here](https://github.com/bnbhat/bno08x_ros2_driver/issues/16).

### Trying it

First, clone the repository:
```
mkdir -p ~/tmp_ws/src
cd ~/tmp_ws/src
git clone https://github.com/slgrobotics/bno08x_ros2_driver.git
```
install dependencies:
```
cd ~/tmp_ws; rosdep install --from-paths src --ignore-src -r -y --skip-keys ament_python
```
adjust parameters:
```
vi ~/tmp_ws/src/bno08x_ros2_driver/config/bno085_i2c.yaml
# Modify the following:
#    i2c:
#      enabled: true
#      bus: "/dev/i2c-1"   <- bus "1" here
#      address: "0x4B"     <- actual I2C address
```
build it:
```
cd ~/tmp_ws
colcon build
```
Run the sensor node:
```
ros2 launch bno08x_driver bno085_i2c.launch.py
```
You should see the `/imu` topic in *rqt* and can display publishing rate:
```
ros2 topic hz /imu
average rate: 101.149
	min: 0.000s max: 0.019s std dev: 0.00167s window: 102
```
To see IMU orientation in RViz2, you need to:
- bring up RViz2 (IMU is already setup in my RViz config):
```
ros2 launch articubot_one launch_rviz.launch.py use_sim_time:=false
```
- run static transformer node to relate frames, raising *IMU frame* 1 meter up on the *map*:
```
ros2 run tf2_ros static_transform_publisher 0 0 1 0 0 0 map bno085
```
Enable the *IMU checkbox* and adjust topic to show `/imu`:

<img width="1593" height="1088" alt="Screenshot from 2025-11-16 11-15-49" src="https://github.com/user-attachments/assets/405656a8-ca31-4bec-8351-71ad57412543" />

### Running ICM20948 node on the robot and tuning EKF filter

Refer to [this file](https://github.com/slgrobotics/articubot_one/blob/main/robots/seggy/launch/seggy.sensors.launch.py) for real-life parameters for running ICM20948 node.

EKF [filter](https://github.com/slgrobotics/articubot_one/blob/main/launch/ekf_odom.launch.py) fuses wheels odometry with IMU orientation data,
deriving reliable orientation quaternion in turns and straight runs (topic *"odometry/local"*).

It is very important to have EKF filter tuned properly, as shown [here](https://github.com/slgrobotics/articubot_one/blob/main/robots/seggy/config/ekf_odom_params.yaml), for example.

More EKF tuning tips [here](https://chatgpt.com/s/t_691b80a57e588191b1528a238588b87a).

### Positioning sensor on the robot

This is the orientation of the *Teyleten Robot GY-ICM20948* carrier board on my Seggy robot: the chip faces upward and the VCC pin toward the front.

<img width="1285" height="1533" alt="Screenshot from 2025-11-16 18-53-23" src="https://github.com/user-attachments/assets/ae3ba654-4029-458f-ad4b-39567f0236bb" />

This is the orientation of the *Teyleten Robot GY-ICM20948* carrier board on my Dragger robot: the chip faces upward and the VCC pin toward the front.

<img width="1319" height="1654" alt="Screenshot from 2025-11-28 19-28-48" src="https://github.com/user-attachments/assets/de29ac72-b68f-481a-919f-650343a7b77b" />


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

Sensor available here: https://www.amazon.com/dp/B0CL26J81F

----------------

**Back to** [Wiki](https://github.com/slgrobotics/articubot_one/wiki) or [Docs Folder](https://github.com/slgrobotics/robots_bringup/tree/main/Docs)
