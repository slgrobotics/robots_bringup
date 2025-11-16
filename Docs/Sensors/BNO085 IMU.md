## ROS2 driver for BNO085 IMU

BNO085 is a 9-DOF Absolute Orientation IMU Fusion sensor module combining accelerometer, gyroscope, and magnetometer data for precise 9 degrees of freedom measurements.

### Connections:

Connect to Raspberry Pi  **I2C**: **SCL** - pin 05, **SDA** - pin 03

RPi GPIO header pinout: https://www.raspberrypi.com/documentation/computers/images/GPIO-Pinout-Diagram-2.png

_Optional:_ Consider 3 KOhm to 6.8 KOhm pull-up resistors from SDA and SCL to 3.3 V bus.

Use ```i2cdetect -y 1``` to see **address 0x4b**

### Info and tests

Information: https://www.adafruit.com/product/4754

The ROS2 driver code is here: https://github.com/bnbhat/bno08x_ros2_driver

### Trying it

First, clone the repository:
```
mkdir -p ~/tmp_ws/src
cd ~/tmp_ws/src
git clone https://github.com/bnbhat/bno08x_ros2_driver.git
```
install dependencies:
```
cd ~/tmp_ws; rosdep install --from-paths src --ignore-src -r -y
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
You should see the `/imu` topic in *rqt*. To see it in RViz2, you need to:
- bring up RViz2 (IMU is already setup in my RViz config):
```
ros2 launch articubot_one launch_rviz.launch.py use_sim_time:=false
```
- run static transformer node to relate frames, raising *IMU frame* 1 meter up on the *map*:
```
ros2 run tf2_ros static_transform_publisher 0 0 1 0 0 0 map bno085
```
<img width="1593" height="1088" alt="Screenshot from 2025-11-16 11-15-49" src="https://github.com/user-attachments/assets/405656a8-ca31-4bec-8351-71ad57412543" />

### Positioning sensor on the robot

Here is how Adafruit BNO085 carrier board is oriented on Seggy robot:



Refer to [this file](https://github.com/slgrobotics/articubot_one/blob/dev/robots/seggy/launch/seggy.sensors.launch.py) for real-life parameters for running BNO085 node.

### Real-time monitoring IMU orientation data

Install [PlotJuggler](https://plotjuggler.io) to monitor IMU data:
```
sudo apt install ros-${ROS_DISTRO}-plotjuggler-ros
```
Run *PlotJugger* as a ROS2 process. It subscribes to topic's values (i.e. "*imu_data*" *yaw*) and shows them in real time:
```
ros2 run plotjuggler plotjuggler
```
**Tip:** drag the desired value ("yaw") from the left panel to the main (plot) panel. Right-click on the plot to adjust *Min*, *Max* etc.

----------------

**Back to** [Wiki](https://github.com/slgrobotics/articubot_one/wiki) or [Docs Folder](https://github.com/slgrobotics/robots_bringup/tree/main/Docs)
