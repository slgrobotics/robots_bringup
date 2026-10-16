**Back to** [Wiki](https://github.com/slgrobotics/articubot_one/wiki)

### Current Monitoring Sensors

it's useful to keep an eye on total current consumption in a robot - especially if you can program a behavior to back out of "wheels stuck" situation and other magic grey smoke events.

Some motor controllers provide feedback values for monitoring, others do not.

There are dedicated sensors with analog and I2C interfaces, for "high side" and "low side" current monitoring:

- https://www.amazon.com/dp/B08R5RBNFK
- https://www.amazon.com/dp/B07PNN4X6P
- https://www.amazon.com/dp/B0CDWXB8PG

The sensor signal can then go to Arduino's A/D or I2C, and then be forwarded to ROS2 to become "current" member of the [BatteryState message](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/BatteryState.html). Or, could be published via MicroROS or any other way.

For those who already have a _low side_ [digital V/A meter](https://www.amazon.com/Aideepen-Digital-Voltmeter-Multimeter-Red-Blue/dp/B0CP593Z2P) on a robot
and don't want to add another shunt resistor to the circuit - there is a way to use that meter's shunt directly.
All you need is an amplifier and a free A/D pin on your MC.

Here's my artistic rendering of such device for my robot Plucky (hey, robotics is art, not engineering ;-)):

[Plucky Photos and Schematics](https://photos.app.goo.gl/YdYQ8kQrNmLkVXTM7)

For ROS2 side of things refer to https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/BatteryStateBroadcaster.md

Plucky Arduino Mega 2560 code is here: https://github.com/slgrobotics/Misc/tree/master/Arduino/Sketchbook/PluckyWheelsROS

----------------

**Back to** [Wiki](https://github.com/slgrobotics/articubot_one/wiki) or [Docs Folder](https://github.com/slgrobotics/robots_bringup/tree/main/Docs)
