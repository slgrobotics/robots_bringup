# Lawnmower Notes

Lawnmower (a.k.a. *Crazed Rhino*) is an actual [Husqvarna Z254F](https://www.husqvarna.com/us/zero-turn-mowers/z254f-special-edition) zero-turn riding mower, robotized for, well, mowing my lawn.

It is running my version (fork) of [PX4 Autopilot](https://docs.px4.io/) code. I control it using [QGroundControl app](https://qgroundcontrol.com/) on a desktop machine.

Code is [here](https://github.com/slgrobotics/PX4-Autopilot/tree/dev)

Lawnmower photos are here: https://photos.app.goo.gl/jwYZRtTi1LVshQoW8

**Note:** 
1. This is **not** a ROS2 project, PX4 is a completely different system.
2. Once you set up your machine for [PX4 development](https://docs.px4.io/main/en/dev_setup/getting_started.html), you can just run Lawnmower robot in Gazebo sim on a Desktop machine (no robot hardware required) - 
use the following command: `make px4_sitl gz_lawnmower`
3. While a lawnmower model is included in standard PX4 codebase, my fork contains customizations needed for tighter control of vehicle speed and additional functions.
4. PX4 plays nice with ROS2, coexists with Jazzy and shares Gazebo Harmonic installation. I use the same machine for ROS2 and PX4 development without any issues.

## Hardware

Onboard, Lawnmower has a Raspberry Pi 4 8GB ("pipx4"). 

## Build and Run Instructions (on the robot's Raspberry Pi 5):

TBD

## Running the robot

Robot can be started with: TBD

For diagnostics, run TBD



----------------

**Back to** [Docs Folder](https://github.com/slgrobotics/robots_bringup/tree/main/Docs)
