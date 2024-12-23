## ROS2 driver for BNO055 IMU

"Bosch BNO055 is a 9-DOF Absolute Orientation IMU Fusion sensor module combining accelerometer, gyroscope, and magnetometer data for precise 9 degrees of freedom measurements."

### Connections:

Connect to Raspberry Pi  **I2C**: **SCL** - pin 05, **SDA** - pin 03

_Optional:_ Consider 3 KOhm to 6.8 KOhm pull-up resistors to 3.3 V bus.

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
cd ..
colcon build
vi ~/robot_ws/src/bno055/bno055/params/bno055_params_i2c.yaml   - *** Here you change i2c_bus to 1
sudo pip3 install smbus
``` 
Try running it, see IMU messages in rqt:
``` 
source ~/robot_ws/install/setup.bash
ros2 run bno055 bno055  --ros-args --params-file ~/robot_ws/src/bno055/bno055/params/bno055_params_i2c.yaml
```

