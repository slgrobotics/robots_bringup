
### A perfect summer day is when the sun is shining, the breeze is blowing, the birds are singing, and the lawn mower is broken.

<img width="1536" height="1024" alt="ChatGPT Image Sep 8, 2025, 12_23_28 PM" src="https://github.com/user-attachments/assets/afbfa7ed-81ef-4a9e-b931-8bec7c6ddef3" />

# Grassroamer Notes

*Grassroamer* is an actual
[Walmart](https://www.walmart.com/ip/PowerSmart-Gas-Lawn-Mower-21-inches-144cc-2-in-1-Walk-Behind-Side-Discharge-Push-Lawn-Mower/475461862) 
PowerSmart Gas Lawn Mower, 21 inches 144cc, robotized for, well, mowing my lawn.

It is running my version (fork) of [PX4 Autopilot](https://docs.px4.io/) code. I control it using [QGroundControl app](https://qgroundcontrol.com/) on a desktop machine.

Code is [here](https://github.com/slgrobotics/PX4-Autopilot/tree/dev)

Lawnmower photos are here: https://photos.app.goo.gl/3sNqAYzAbU29Zv7r7

**Note:** 
1. This is **not** a ROS2 project, PX4 (https://px4.io/) is a completely different system.
2. My now retired legacy lawnmower (a.k.a. Crazed Rhino) is described [here](https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Lawnmower/CrazedRhino.md).
3. Once you set up your machine for [PX4 development](https://docs.px4.io/main/en/dev_setup/getting_started.html), you can just run Lawnmower robot in Gazebo sim on a Desktop machine (no robot hardware required) - 
use the following command: `make px4_sitl gz_lawnmower`
4. While a lawnmower model is included in standard PX4 codebase, my fork contains customizations needed for tighter control of vehicle speed and additional functions.
5. PX4 plays nice with ROS2, coexists with Jazzy and shares Gazebo Harmonic installation. I use the same machine for ROS2 and PX4 development without any issues.
6. PX4 has a very active [Discord](https://discord.com/channels/1022170275984457759) community.

## Hardware

Onboard, Lawnmower has a Raspberry Pi 4 8GB ("pipx4") - an upgrade from RPi 3B which I've been using for a while.

There's also:
- an [MPU9250](https://www.amazon.com/HiLetgo-Gyroscope-Acceleration-Accelerator-Magnetometer/dp/B01I1J0Z7Y) on SPI,
- two SparkFun u-blox ZED-9P [boards](https://www.sparkfun.com/sparkfun-gps-rtk-sma-breakout-zed-f9p-qwiic.html) on USB-serial,
- a Teensy 3.2 for interfacing R/C receiver on I2C - code [here](https://github.com/slgrobotics/Misc/tree/master/Arduino/Sketchbook/RC_PPM_Receiver),
- Pixhawk color [LED](https://www.amazon.com/TYXTTGY-Pixhawk-PX4-External-Extension-Module/dp/B0F29BV7WC) (I2C)
- PCA9685 [board](https://www.amazon.com/dp/B0CRV3MK14) (I2C) (you can use a Teensy emulator instead - code is [here](https://github.com/slgrobotics/Misc/tree/master/Arduino/Sketchbook/Teensy_PCA9685)).
- a RoboteQ [AX2850](https://www.bipom.com/documents/roboteq/ax2550quickstart19b-060107.pdf) motor controller.
- a point-to-point [Ethernet link](https://www.amazon.com/dp/B09WMBD1S7).

My [photo blog](https://photos.app.goo.gl/jwYZRtTi1LVshQoW8) contains some schematics and pictures of the components.

## Power source

Grassroamer is powered by a 100AH LiFrPO4 13.4 V (4 cell) [battery](https://www.amazon.com/dp/B09L89LW3P).

If using a "_self-propelled_" lawnmower, it is possible to power wheels and electronics with an alternator, driven by the belt:

<img width="2117" height="960" alt="Screenshot from 2025-09-17 09-59-44" src="https://github.com/user-attachments/assets/12ce69de-cb28-46ce-9242-91d7ff73a370" />

In this case a much smaller battery would suffice.

**Note:** Larger engines use an integrated _magneto+generator+regulator_ "stator" hardware with varying power output.
The "rotor" part is a set of magnets on a rotating cup. This is unlikely to be part of a small lawnmower.

<img width="937" height="878" alt="Screenshot from 2025-09-18 11-29-16" src="https://github.com/user-attachments/assets/524fcd95-fa11-490c-b414-430b436e8f0f" />

## Build and Run Instructions

Set up your Ubuntu 24.04 Desktop machine for [PX4 development](https://docs.px4.io/main/en/dev_setup/getting_started.html)

Clone the [repository](https://github.com/slgrobotics/PX4-Autopilot/tree/dev) - use *dev* branch:
```
mkdir ~/lawnmow
cd ~/lawnmow
git clone https://github.com/slgrobotics/PX4-Autopilot.git --recursive --single-branch -b dev
```

#### Run simulation in Gazebo:
```
cd ~/lawnmow/PX4-Autopilot
make px4_sitl gz_lawnmower
```
In *QGroundControl* use one of the mission plans from the *~/lawnmow/PX4-Autopilot/plans* folder.

**Note:** the model used in simulation describes my (now retired) [Crazed Rhino](https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Lawnmower/CrazedRhino.md) mower.
There is no model for current project yet.

There are many other vehicle models available for simulation, for example:
```
make px4_sitl gz_rover_differential_lawn
make px4_sitl gz_r1_rover
make px4_sitl gz_x500_forest
```
**Note:** the `_lawn`, `_forest` postfixes define Gazebo Worlds and can be omitted for default behavior.

See `~/lawnmow/PX4-Autopilot/Tools/simulation/gz/worlds` for options.

#### Build it on the Desktop for upload to Raspberry Pi:

Install support for arm64:
```
sudo apt install g++-aarch64-linux-gnu
```
Now you can build the Lawnmower executable tree (see _~/lawnmow/PX4-Autopilot/build/*_):
```
make emlid_navio2_arm64
  or, with upload:
make emlid_navio2_arm64 upload
```

#### On the robot's Raspberry Pi:

The upload creates directory *~/px4* on the RPi, which serves as a "read-only" code storage.

You need to copy some helper scripts from it once for creating and maintaining a "work storage" - *px4wrk* folder.
```
ssh pi@pipx4.local
```
Do it once:
```
cd ~/px4/etc/scripts
bash deploy_scripts.sh
cd ~
bash create_px4wrk.sh
```
Now you can run the PX4 code:
```
sudo ./run.sh
```
While running, PX4 creates another "work storage" - `/fs` and puts *.ulg* files in the `/fs/log` folder.
These files can be viewed with [PlotJuggler](https://plotjuggler.io/) app or analyzed with https://review.px4.io/ - for example, [this plot](https://review.px4.io/plot_app?log=f2393d38-2560-4423-b948-ec367bcb7f20).

**Note:**
If you modify any of the C++ source files, make sure that *astyle* formatting checkups are enforced:
```
cd ~/lawnmow/PX4-Autopilot
make check_format
  or
make format
```

There are other RPi-based target *boards* and corresponding builds - something to keep in mind:
```
make scumaker_pilotpi_arm64
make px4_raspberrypi_arm64
```
----------------

**Back to** [Wiki](https://github.com/slgrobotics/articubot_one/wiki) or [Docs Folder](https://github.com/slgrobotics/robots_bringup/tree/main/Docs)
