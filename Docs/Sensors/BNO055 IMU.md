## ROS2 driver for BNO055 IMU

"Bosch BNO055 is a 9-DOF Absolute Orientation IMU Fusion sensor module combining accelerometer, gyroscope, and magnetometer data for precise 9 degrees of freedom measurements."

### Connections:

Connect to Raspberry Pi  **I2C**: **SCL** - pin 05, **SDA** - pin 03

RPi GPIO header pinout: https://www.raspberrypi.com/documentation/computers/images/GPIO-Pinout-Diagram-2.png

_Optional:_ Consider 3 KOhm to 6.8 KOhm pull-up resistors from SDA and SCL to 3.3 V bus.

Use ```i2cdetect -y 1``` to see **address 0x28**

### Info and tests:

https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor

https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/webserial-visualizer

https://www.bosch-sensortec.com/products/smart-sensor-systems/bno055/

BNO055 IMU (via UART or I2C - Python) - seems well supported, active development, ROS2 node

ROS2 driver code is here:

https://github.com/flynneva/bno055

### Installation
```
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src/
git clone https://github.com/flynneva/bno055.git
cd ~/robot_ws
vi ~/robot_ws/src/bno055/bno055/params/bno055_params_i2c.yaml   - *** Here you change i2c_bus to 1
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -r -y
sudo pip3 install smbus
(sudo apt install python3-smbus   -- this might work instead)
colcon build
``` 
Try running it, see IMU messages in rqt:
``` 
source ~/robot_ws/install/setup.bash
ros2 run bno055 bno055  --ros-args --params-file ~/robot_ws/src/bno055/bno055/params/bno055_params_i2c.yaml
```
Refer to [this file](https://github.com/slgrobotics/articubot_one/blob/dev/robots/turtle/launch/myturtle.py) for real-life parameters for running BNO055 node.

### Useful Links

BNO055 calibration utility: https://github.com/fm4dd/pi-bno055

https://robofoundry.medium.com/lessons-learned-while-working-with-imu-sensor-ros2-and-raspberry-pi-a4fec18a7c7

https://github.com/adafruit/Adafruit_CircuitPython_BNO055/blob/main/README.rst


