![Screenshot from 2025-07-05 17-55-44](https://github.com/user-attachments/assets/c6746dc2-1af0-4046-aaf6-c68273ef6fa1)

# Lawnmower Notes

Lawnmower (a.k.a. *Crazed Rhino*) is an actual [Husqvarna Z254F](https://www.husqvarna.com/us/zero-turn-mowers/z254f-special-edition) zero-turn riding mower, robotized for, well, mowing my lawn.

It is running my version (fork) of [PX4 Autopilot](https://docs.px4.io/) code. I control it using [QGroundControl app](https://qgroundcontrol.com/) on a desktop machine.

Code is [here](https://github.com/slgrobotics/PX4-Autopilot/tree/dev)

Lawnmower photos are here: https://photos.app.goo.gl/jwYZRtTi1LVshQoW8

**Note:** 
1. This is **not** a ROS2 project, PX4 (https://px4.io/) is a completely different system.
2. Once you set up your machine for [PX4 development](https://docs.px4.io/main/en/dev_setup/getting_started.html), you can just run Lawnmower robot in Gazebo sim on a Desktop machine (no robot hardware required) - 
use the following command: `make px4_sitl gz_lawnmower`
3. While a lawnmower model is included in standard PX4 codebase, my fork contains customizations needed for tighter control of vehicle speed and additional functions.
4. PX4 plays nice with ROS2, coexists with Jazzy and shares Gazebo Harmonic installation. I use the same machine for ROS2 and PX4 development without any issues.

## Hardware

Onboard, Lawnmower has a Raspberry Pi 4 8GB ("pipx4") - an upgrade from RPi 3B which I've been using for a while.

There's:
- an MPU9250 on SPI,
- two SparkFun u-blox ZED-9P boards on USB-serial,
- a Teensy for interfacing R/C receiver on I2C - code here,
- Pixhawk color LED (I2C)
- a Teensy emulating a PCA9685 with optocouplers (I2C) - code here.

## Build and Run Instructions

Set up your machine for [PX4 development](https://docs.px4.io/main/en/dev_setup/getting_started.html)

Clone the [repository](https://github.com/slgrobotics/PX4-Autopilot/tree/dev) - use *dev* branch:
```
mkdir ~/lawnmow
cd ~/lawnmow
git clone https://github.com/slgrobotics/PX4-Autopilot.git --recursive --single_branch -b dev
```

### Run simulation in Gazebo:
```
cd ~/lawnmow/PX4-Autopilot
make px4_sitl gz_lawnmower
```
In *QGroundControl* use one of the mission plans from the *~/lawnmow/PX4-Autopilot/plans* folder.

There are many other vehicle models available for simulation, for example:
```
make px4_sitl gz_rover_differential_lawn
make px4_sitl gz_r1_rover
```

### Build it for Raspberry Pi:
```
make emlid_navio2_arm64
  or, with upload:
make emlid_navio2_arm64 upload
```

### On the robot's Raspberry Pi:

The upload creates directory *~/px4* on the RPi, which serves as a "read-only" code storage.

You need to copy some help scripts from it once for creating and maintaining a "work storage" - *px4wrk* folder.
```
ssh pi@pipx4.local

# do it once:
--- TBD

sudo ./run.sh
```
While running, PX4 creates another "work storage" - */fs* and puts *.ulg* files in */fs/log* folder

**Note:**
If you modify any of the C++ source files, make sure that *astyle* formatting checkups are enforced:
```
cd ~/lawnmow/PX4-Autopilot
make check_format
  or
make format
```

There are other RPi-based target *boards* and corresponding builds:
```
make scumaker_pilotpi_arm64
make px4_raspberrypi_arm64
```
----------------

**Back to** [Docs Folder](https://github.com/slgrobotics/robots_bringup/tree/main/Docs)
