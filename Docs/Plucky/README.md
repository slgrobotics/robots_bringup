# Plucky Notes

```
pluck·y /'pl?ke/ adjective
having or showing determined courage in the face of difficulties.
Synonyms of plucky: SPIRITED, BRAVE
```
Plucky photos are here: https://photos.app.goo.gl/YdYQ8kQrNmLkVXTM7

Plucky is a *"larger Turtlebot"* - running, basically, standard ROS2 Turtlebot 3 binaries for navigation (on the desktop "ground station" computer). Onboard it has two Raspberry Pi 3B and an "FPV Drone" TV camera. One RPi runs sensors drivers (LD14 LIDAR and MPU9250), the other - Differential Drive Control, inspired by Articulated Robotics (https://articulatedrobotics.xyz/mobile-robot-full-list/).

## Links and Instructions:

Arduino Mega code - wheels/sensors driver: https://github.com/slgrobotics/Misc/tree/master/Arduino/Sketchbook/PluckyWheelsROS

"pluckysens" Raspberry Pi 3B code (ROS2 LD14 LIDAR and MPU9250 Drivers):

"plucky" Raspberry Pi 3B code (ROS2 Differential Drive Control): https://github.com/slgrobotics/diffdrive_arduino

## Build and Run Instructions:

