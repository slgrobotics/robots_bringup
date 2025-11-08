## ROS2 driver for BNO055 IMU

"Bosch BNO055 is a 9-DOF Absolute Orientation IMU Fusion sensor module combining accelerometer, gyroscope, and magnetometer data for precise 9 degrees of freedom measurements."

### Connections:

Connect to Raspberry Pi  **I2C**: **SCL** - pin 05, **SDA** - pin 03

RPi GPIO header pinout: https://www.raspberrypi.com/documentation/computers/images/GPIO-Pinout-Diagram-2.png

_Optional:_ Consider 3 KOhm to 6.8 KOhm pull-up resistors from SDA and SCL to 3.3 V bus.

Use ```i2cdetect -y 1``` to see **address 0x28**

**Note:** Adafruit sensor will show 0x28, [GY-BNO055 clone](https://www.amazon.com/dp/B0D47G672B) - 0x29 (with both jumpers closed). Match that in Node startup code.

### Info and tests:

https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor

https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/webserial-visualizer

https://www.bosch-sensortec.com/products/smart-sensor-systems/bno055/

BNO055 IMU (via UART or I2C - Python) - seems well supported, active development, ROS2 node

The "standard" ROS2 driver code is here:

https://github.com/flynneva/bno055

### Issues When Using the “Standard” ROS 2 Driver

Here are my observations while using the "*standard*" ROS 2 driver on Turtle:

1. I have three *BNO055* sensors — one from Adafruit with the original Bosch chip, and two “generic carrier boards” using the GY-BNO055 chip.
Apart from the I²C address, there’s no practical difference in how they behave — same data, same issues.
2. The "standard" ROS 2 driver relies on *smbus*, while a more advanced Python package, *[smbus2](https://pypi.org/project/smbus2/)*,
offers improved integration with Linux and better overall reliability.
3. When monitoring */imu/data/orientation/yaw* using [PlotJuggler](https://www.plotjuggler.io), I noticed frequent (~1..3 seconds) spikes or jumps in the data.
It appears that neither I²C communication nor the code’s exception handling are responsible for this.

I created a [fork](https://github.com/slgrobotics/bno055) of the driver, switched to *smbus2* and implemented a “sanity check” for incoming data — this fully resolved the issue.

### Position on the robot

Here is how Adafruit BNO055 carrier board is oriented on Create 1 [Turtlebot](https://photos.app.goo.gl/ED3YbmNRm4kKt3V5A) (view from the rear):

![Screenshot from 2025-05-27 21-48-13](https://github.com/user-attachments/assets/861db7a1-cb60-4215-b6bc-52361d92a453)

If you are using a [generic carrier board](https://www.amazon.com/dp/B0D47G672B), X axis should point backwards, and Y - to the right of the robot.

### Installation
```
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src/
git clone https://github.com/slgrobotics/bno055.git  # using my fork of the driver
cd ~/robot_ws
vi ~/robot_ws/src/bno055/bno055/params/bno055_params_i2c.yaml   # *** Here you may change default i2c_bus to 1 and check the address
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -r -y
sudo pip3 install smbus2
(sudo apt install python3-smbus2   # this might work instead)
colcon build
``` 
Try running it, see IMU messages in rqt:
``` 
source ~/robot_ws/install/setup.bash
ros2 run bno055 bno055  --ros-args --params-file ~/robot_ws/src/bno055/bno055/params/bno055_params_i2c.yaml
```
Refer to [this file](https://github.com/slgrobotics/articubot_one/blob/dev/robots/turtle/launch/myturtle.py) for real-life parameters for running BNO055 node.

### Useful Links

Pinout and BNO055 varieties: https://gr33nonline.wordpress.com/2019/04/19/dont-get-the-wrong-bno055

BNO055 calibration utility: https://github.com/fm4dd/pi-bno055

https://robofoundry.medium.com/lessons-learned-while-working-with-imu-sensor-ros2-and-raspberry-pi-a4fec18a7c7

https://github.com/adafruit/Adafruit_CircuitPython_BNO055/blob/main/README.rst

Datasheet: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf

Quick start guide: https://www.bosch-sensortec.com/media/boschsensortec/downloads/application_notes_1/bst-bno055-an007.pdf

Other docs: https://www.bosch-sensortec.com/products/smart-sensors/bno055/

----------------

**Back to** [Docs Folder](https://github.com/slgrobotics/robots_bringup/tree/main/Docs)

