# Lawnmower Notes

Lawnmower photos are here: https://photos.app.goo.gl/jwYZRtTi1LVshQoW8

Lawnmower (a.k.a. *Crazed Rhino*) is an actual Husqvarna Z254F zero-turn riding mower, robotized for, well, mowing my lawn.

It is running my version (fork) of PX4 Autopilot code. I control it using QGroundControl app on a desktop machine.

Code is [here](https://github.com/slgrobotics/PX4-Autopilot/tree/dev)

**Note:** 
1. This is **not** a ROS2 project, PX4 is a completely different system.
2. You can just run Lawnmower robot in Gazebo sim on a Desktop machine (no robot hardware required) - 
use the following command:
```
make px4_sitl gz_lawnmower_lawn
```

## Hardware

Onboard, Lawnmower has a Raspberry Pi 4 8GB ("pipx4"). 

## Build and Run Instructions (on the robot's Raspberry Pi 5):


## Running the robot

Robot can be started with:

For diagnostics, run 



----------------

**Back to** [Docs Folder](https://github.com/slgrobotics/robots_bringup/tree/main/Docs)
