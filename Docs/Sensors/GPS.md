# GPS setup

**Note:** this guide can be used for other NMEA capable GPS (in fact, almost any GPS)

Dragger has a _"BE-880 GPS Receiver Module with Flash HMC5883L Compass 10th Chip GPS Antenna"_, available on Amazon. A u-blox NEO-M10N module is part of it.

Features:
```
Chip: M10050
Frequency: GPS L1 C/A,QZSS L1 C/A/S,BDS B1I/B1C,Galileo E1B/C,SBAS L1 C/A:WAAS,EGNOS,MSAS,GAGAN
Operation Mode: GPS+BDS+GALILEO+SBAS+QZSS
Sensitivity: Track -166dBm, Re-arrest -160dBm, Cold Start -148dBm, Hot Start -160dBm
Horizontal Accuracy: 1.5m CEP 2D RMS SBAS Auxiliary(for open sky)
Speed Accuracy: 0.05m/s
Dynamic Heading Angle Accuracy: 0.3 deg
1PPS Time Accuracy: RMS 30ns, 99% 60ns
Start Time: Cold Start 27s, Hot Start 1s, Assisted Start 1s
Baud Rate: 4800bps - 921600bps, default 38400bps
Output Level: TTL level
Output Protocol: NMEA, UBX
NMEA Sentences: RMC, VTG, GGA, GSA, GSV, GLL
Update Frequency: 0.25Hz - 18Hz, default 1Hz
Second Pulse: Configurable from 0.25Hz to 10MHz, default period 1s, high level last for 100ms
Graviational Acceleration: <4g
Voltage: DC 3.6V - 5.5V, Typical 5.0V
Current: Normal 50mA/5.0V
Size: 28*28*11mm
Connector: 1.25mm 6pin
```
Default baud rate is 38400, and it works out of the box at 1 Hz.

You can use _u-center_ Windows app to set up the device initially, saving settings to its Flash memory. Specifically, set NMEA at 115200 baud at 10 Hz to be streaming on USB.

Check it (Use Ctrl/A + Ctrl/X to exit):
```
picocom /dev/ttyUSBGPS -b 115200

```
We need to install standard ROS Jazzy support for NMEA messages:
```
[Ubuntu 22.04 only]:
sudo pip3 install transforms3d
[Ubuntu 24.04 only]:
sudo apt install python3-transforms3d

sudo apt install ros-${ROS_DISTRO}-nmea-navsat-driver

The following NEW packages will be installed:
  ros-jazzy-nmea-msgs ros-jazzy-nmea-navsat-driver ros-jazzy-tf-transformations
```
The driver code is here ("ros2" branch) - look into _launch_ and _config_ folders:

https://github.com/ros-drivers/nmea_navsat_driver/blob/ros2/config/nmea_serial_driver.yaml

GPS Node will be run as part of the _dragger.launch.py_ process.

>> [NOTE] The process described here didn't work with my NEO-M8N device:
>>
>> https://docs.fictionlab.pl/leo-rover/integrations/positioning-systems/ublox-evk-m8n
>> ```
>> sudo apt install ros-${ROS_DISTRO}-ublox
>> 
>> The following NEW packages will be installed:
>>   libasio-dev ros-jazzy-ublox ros-jazzy-ublox-gps ros-jazzy-ublox-msgs ros-jazzy-ublox-serialization
>> ```
>> Code is here: https://github.com/KumarRobotics/ublox/blob/ros2/README.md
>>
>>  There's a lot of chatter on the Internet about this problem.

----------------

**Back to** [Docs Folder](https://github.com/slgrobotics/robots_bringup/tree/main/Docs)
