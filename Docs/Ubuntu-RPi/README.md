# Set up your Raspberry Pi 3B/4/5 with Ubuntu Server and ROS Base

We need to copy a _Ubuntu 24.04 Server 64 bit_ image to an SD card, install _ROS2 Jazzy Base_ and add necessary ROS packages.

**Note: (Advanced)** Alternatively you can use real-time OS image with ROS2 Jazzy preinstalled - 
described [here](https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Ubuntu-RPi/UbuntuRealTime.md).

### 1. Set up a Ubuntu 24.04 Server 64 bit

Download _Raspberry Pi Imager_ from official Raspberry OS site: https://www.raspberrypi.com/software/

Run it, selecting _"Other General-Purpose OS" -> Ubuntu -> Ubuntu Server 24.04.1 LTS (64 bit)_

The Imager lets you customize the boot - choose network name, setup "ros" account and WiFi credentials.

It will not setup Ethernet (eth0) and might mess up WiFi setup, so you better have USB keyboard and HDMI display connected until your SSH starts working.

For faster set up and updates, better use Ethernet cable initially. Later on, WiFi should be adequate.

_Optional:_ You may want to get better grip on your network setup, using ```/etc/netplan/*``` directly. Scroll to the end for directions.

After your RPi boots up, make sure your other accounts (if any) are privileged and are in _dialout_ group. Fix that - if not already set by Imager. For example:
```
sudo adduser <other account> sudo
sudo adduser ros dialout

hostnamectl set-hostname turtle
```
Wait for the first boot to finish initial maintenance (is it updating in the background?). Update and reboot:
```
sudo apt update
sudo apt upgrade
sudo apt full-upgrade
sudo apt clean    (purges packages from SD   https://www.raspberrypi.org/forums/viewtopic.php?f=66&t=133691)
```
You may want to disable unattended updates to avoid high CPU loads at random times after booting up:
```
sudo dpkg-reconfigure unattended-upgrades      (say No)
```
Have some needed packages installed:
```
sudo apt install raspi-config
sudo apt install winbind samba smbclient net-tools wget
sudo apt install python3-pip
```
You should be able to ping your _turtle.local_ machine and ssh into it (```ssh ros@turtle.local``` from your Desktop machine).

**Note:** if you choose to rely on _/etc/netplan_ and disable network config, you need to `sudo apt install avahi-daemon` to restore Multicast DNS (mDNS) service required for local hostname discovery.

Have the I2C support installed and tested:
```
sudo apt install i2c-tools
sudo adduser ros i2c
sudo i2cdetect -y 1
# Note: you need to re-login to enable group privileges, so that you can skip "sudo" for i2cdetect
```
_Optional:_ Set up commonly used Python GPIO packages: https://ubuntu.com/tutorials/gpio-on-raspberry-pi#1-overview

**Note:** traditional _pip3_ installation method doesn't work on Ubuntu 24.04, as its Python enforces _virtual environment_ use.
The "*python3-*" prefix relates to packages that can be installed globally, avoiding the *venv* inconvenience.
```
sudo apt install python3-lgpio python3-gpiozero python3-RPi.GPIO
```
_Optional:_ Set up [WiringPi GPIO](https://projects.drogon.net/raspberry-pi/wiringpi/download-and-install/).
```
sudo apt install build-essential
mkdir tmp
cd tmp
git clone https://github.com/WiringPi/WiringPi.git
cd WiringPi
bash ./build
gpio -h
sudo gpio readall
``` 
You may need to set up a 2G-4G swap file to compensate for small RAM on RPi 3B. Compiling C++ code takes a lot of RAM and will crash without it:
```
sudo swapon --show     (if nothing shows up, swap isn't set up - https://www.linuxtut.com/en/71e3874cb83ed12ec405/)
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile    (now "top" shows MiB Swap: 2048.0)
echo '/swapfile swap swap defaults 0 0' | sudo tee -a /etc/fstab
```
Now _htop_ will show swap space, and it should persist through reboots.

_Optional:_ Samba creates shared folder, accessible from Windows machines:
```
sudo mkdir /home/shared; sudo chmod og+rwx /home/shared   (+x matters for readability)
sudo usermod -aG sambashare ros
cp  /etc/samba/smb.conf .
vi smb.conf  <- add Samba share - see below
sudo cp smb.conf /etc/samba/smb.conf
sudo systemctl restart smbd
```
Samba share - add to the end of smb.conf, use any Share name you like, e.g "ShareAll". Look for other flags/parameters in that file.
```
[ShareAll]
   path = /home/shared
   read only = no
   public = yes
   writable = yes
   guest ok = yes
   guest only = yes
   force create mode = 777
   force directory mode = 777
```
### 2. Continue with ROS2 installation

Follow these guides, selecting binary ROS2-Base Jazzy ("Bare bones" ros-jazzy-ros-base). Also, install Development tools:

https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html

https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html

for "ros" account, no need to call _setup.bash_ every time - add it to _.bashrc_:
```
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```
_Optional:_ Try some basic things from Tutorials:
```
sudo apt install ros-${ROS_DISTRO}-demo-nodes-py ros-${ROS_DISTRO}-demo-nodes-cpp
```
* now you can run these commands in separate terminals (even across your LAN machines):
```
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_py listener
```

### 3. Make sure your WiFi is up to the task

ROS2 requires good connectivity, and you will have problems if you skip this step.

Refer to [this guide](https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/WiFi_Logger_Visualizer.md#wifi-testing-and-benchmarking) for tuning and benchmarking your WiFi.

As your robot will be using WiFi, having ROS2 transport (DDS) optimized becomes a necessity. Use [this guide](https://docs.ros.org/en/rolling/How-To-Guides/DDS-tuning.html).

### 4. Install _control_ packages needed for our ROS nodes
```
sudo apt install ros-${ROS_DISTRO}-ros2-control ros-${ROS_DISTRO}-ros2-controllers ros-${ROS_DISTRO}-xacro ros-${ROS_DISTRO}-twist-mux
sudo apt install libi2c-dev
```
Now your Raspberry Pi should be ready for installing and compiling your robot's Nodes in ```~/robot_ws``` folder.

Choose one of the robots here: https://github.com/slgrobotics/robots_bringup/tree/main/Docs

*Or, review the* [following](https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Ubuntu-RPi/README-Extra.md) optional steps.

----------------

**Back to** [Wiki](https://github.com/slgrobotics/articubot_one/wiki) or [Docs Folder](https://github.com/slgrobotics/robots_bringup/tree/main/Docs)

