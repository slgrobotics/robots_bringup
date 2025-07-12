# Set up your Raspberry Pi 3B/4/5 with Ubuntu Server and ROS Base

We need to copy a _Ubuntu 24.04 Server 64 bit_ image to an SD card, install _ROS2 Jazzy Base_ and add necessary ROS packages.

**Note: (Advanced)** Alternatively you can use real-time OS image with ROS2 Jazzy preinstalled - described [here](https://github.com/ros-realtime/ros-realtime-rpi4-image).

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

-------------------------------

*Or, review the following:*

### _Optional:_ VS Code Remote Development

You can work on the Desktop machine with repositories/folders on Raspberry Pi.

Follow [this guide](https://code.visualstudio.com/docs/remote/ssh-tutorial).

### _Optional:_ Using UARTs on the Raspberry Pi 5

I don't have a need to use Raspberry Pi UARTs,
so for useful information please refer to the source - Michael Wimble's repository.

To use Raspberry Pi UARTs refer to the following section.

**Warning:** I will not accept any resposibility for damage to any device as a result of you following these instructions. Neither will Michael Wimble, the original author.

https://github.com/wimblerobotics/ros2_roboclaw_driver?tab=readme-ov-file#using-uarts-on-the-raspberry-pi-5

### _Optional:_ Playing sound with _aplay_

You can usually play sound files if you have a _USB Audio Adapter_:
```
aplay ~/wav/cat_meow.wav
```
Here is how to see your audio devices (note _card 2_ in my case):
```
ros@plucky:~$ aplay --list-devices
**** List of PLAYBACK Hardware Devices ****
card 0: vc4hdmi0 [vc4-hdmi-0], device 0: MAI PCM i2s-hifi-0 [MAI PCM i2s-hifi-0]
  Subdevices: 1/1
  Subdevice #0: subdevice #0
card 1: vc4hdmi1 [vc4-hdmi-1], device 0: MAI PCM i2s-hifi-0 [MAI PCM i2s-hifi-0]
  Subdevices: 1/1
  Subdevice #0: subdevice #0
card 2: Set [C-Media USB Headphone Set], device 0: USB Audio [USB Audio]
  Subdevices: 1/1
  Subdevice #0: subdevice #0
```
For some reason, the _aplay_ works with HDMI display connected (even on a "headless" Server), but if not - you have to explicitly specify which "card" you want _aplay_ to use:
```
vi ~/.asoundrc

# Puth the following in this file:
defaults.pcm.card 2
defaults.ctl.card 2
```
Alternatively you can create file _/etc/asound.conf_ with the same content for system-wide settings.

### _Optional:_ Create a Linux service for on-boot autostart

https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Ubuntu-RPi/LinuxService.md

### _Optional:_ Husarnet VPN for better DDS communication 

https://github.com/slgrobotics/robots_bringup/blob/main/Docs/ROS-Jazzy/README-Husarnet.md

### _Optional:_ better network control using _/etc/netplan/_ directly

It is possible, that after the initial boot your network won't be set up properly.

On top of that, your mobile robot will work much better, if it is assigned static network address and it doesn't have to query DHCP service on your router.

Ubuntu uses "_cloud-init_" service to set up your network every time you boot up. Turn it off:

In sudo mode, create a file _/etc/cloud/cloud.cfg.d/99-disable-network-config.cfg_ with the following content:
```
network: {config: disabled}
```

Now, in sudo mode, you can edit _/etc/netplan/50-cloud-init.yaml_ directly - setting your interfaces, addresses, options.

Google "_how to assign static ip address on ubuntu 24.04 server_" for details.

This is an example file I use in _/etc/netplan/50-cloud-init.yaml_ - to assign "192.168.1.137 address (your local network may be other than ```192.168.*.*```):
```	
# This file is generated from information provided by the datasource.  Changes
# to it will not persist across an instance reboot.  To disable cloud-init's
# network configuration capabilities, write a file
# /etc/cloud/cloud.cfg.d/99-disable-network-config.cfg with the following:
# network: {config: disabled}
network:
    version: 2
    ethernets:
      eth0:
        optional: true
        dhcp6: true
        dhcp4: true

    wifis:
      wlan0:
        optional: true
        access-points:
          "<your WiFi network SSID>":
            password: "<your WiFi password here>"
        dhcp6: true
        dhcp4: false
        addresses:
          - 192.168.1.137/24
        routes:
          - to: default
            via: 192.168.1.1/24
        nameservers:
          addresses:
            - 192.168.1.1
```	
**Note:** static addresses you assign should be within the "exclusion zone" of your router's DHCP service. This is easy setup on the router side.

Now reload configuration and check the result:
```
sudo netplan apply

ifconfig -a
  or
ip a
```

### _Optional:_ time sync using _chrony_

ROS nodes rely on tight time synchronization between participating machines. Default setup might not work well for you.

Time sync info:

https://ubuntu.com/server/docs/network-ntp

https://chrony.tuxfamily.org/doc/3.4/chrony.conf.html

https://chrony.tuxfamily.org/examples.html			

Normally preinstalled service is _systemd-timesyncd_ (a lightweight client-only NTP)

When chrony is not installed - check your time setup:
```
sudo systemctl status systemd-timesyncd
timedatectl timesync-status
timedatectl status
timedatectl show-timesync -a
ll /run/systemd/timesync/synchronized     (see timestamp = last successful sync, every 34 minutes)
	
https://wiki.archlinux.org/title/systemd-timesyncd
	
vi /etc/systemd/timesyncd.conf
```

Chrony (full NTP with server option) replaces/masks systemd-timesyncd if installed.
```
sudo apt install chrony ntpdate
```
Check if it works:
```
sudo systemctl status chronyd
chronyc -n tracking
chronyc sources
chronyc sourcestats

```
On the Linux desktop (to be assigned as NTP server, in my case - 192.168.1.130, with static address):
```
vi /etc/chrony/chrony.conf

add to the end : add network range you allow to receive time syncing requests from clients

allow 192.168.1.0/24
```
On the Raspberry Pi:

_Note:_ Offset 0.01 allows RPi to be a bit behind the server, avoiding "lookup into future"
```
vi /etc/chrony/chrony.conf

server 192.168.1.130 iburst offset 0.01
initstepslew 30 192.168.1.130
#pool ntp.ubuntu.com        iburst maxsources 4
#pool 0.ubuntu.pool.ntp.org iburst maxsources 1
#pool 1.ubuntu.pool.ntp.org iburst maxsources 1
#pool 2.ubuntu.pool.ntp.org iburst maxsources 2
```
A shell script to see how your time sync is performing (and see CPU/GPU temperatures):
```
#!/bin/bash
set -x
systemctl status chronyd
chronyc sources
chronyc sourcestats
#timedatectl timesync-status
ntpdate -q time.windows.com
timedatectl status
chronyc tracking
set +x
echo "CPU: " `gawk '{print $1/1000," C"}' /sys/class/thermal/thermal_zone0/temp`
echo "GPU: " `vcgencmd measure_temp` | sed 's/temp=//'
```

### _Optional:_ Making compressed backups of an SD card

Your SD card isn't immortal, and might fail if left without power for several months, for other reasons or without a reason at all. You need to make a backup on a less volatile media - for example, your PC. If you read 16 GB SD card into an image file, it will take the same 16 GB of PC disk space, and compression does not always helps to shrink it.

There are many methods available (Pishrink etc.), some work on a live (mounted) system volume and rely on the file system's ability to recover a "live" image on boot.

I prefer a safer method - fill unused space with zeroes and let the Zip shrink the SD image efficiently:
```
sudo apt autoremove
sudo apt clean

cat /dev/zero > zero.file
(takes about an hour on RPi4, about 5 minutes on RPi5 to write 18.7 GB on 32 GB SD Card)
rm zero.file
```

Then make an img file:
- Windows: using, for example, _Win32DiskImager_, and zip it on a PC. To restore it to an SD card, use _Balena Etcher_ (it accepts compressed images).
- Ubuntu: _gnome-disks_ can be used to create image of SD card or write (restore) an image to SD card. _Balena Etcher_ is also available on Linux. You can compress image file using right-click menu in File Manager.

Here is more info:

https://www.baeldung.com/linux/wipe-free-space

### _Optional:_ Replace SD card with NVM on Raspberry 5

I upgraded my RPi5 on Dragger, replacing the SD card with an SSD.

Components used:
- https://www.amazon.com/dp/B0CHNP7P89  - SSD to USB Adapter, used to image the OS on the PC
- https://www.amazon.com/dp/B0CPPGGDQT  - Geekworm X1001 PCIe to M.2 Hat
- https://www.amazon.com/dp/B0C5D6C1YQ  - EN600 PRO SSD 256GB PCIe 3.0 Gen 3x4, NVMe M.2 drive

Steps:
- First I used _SD Imager_ on a Windows 10 machine to backup my SD card.
- Then ran _Balena Etcher_ on the same machine to transfer that image to SSD, which was placed on the M2/USB adapter.
- Then I moved SSD to the RPi Hat, and it booted right away, automatically extending the root partition to full ~256GB.

I could've done imaging on the Ubuntu desktop using "_Disks_" app or "_dd_". Balena Etcher is available for Ubuntu, if that's your preference. Ubuntu _Disks_ app reads and writes images fine.

_Pros_: The RPi feels a bit faster, and I hope for better reliability in the long run.

_Cons_: I won't be able to save the whole OS image anymore, as imaging whole 256 GB drive isn't practical.

**Note:**
- The cable between M2 Hat and RPi5 is finicky, make sure it is oriented properly and is locked. Refer to Amazon pictures.
- SD card should be removed for RPi to boot from SSD.
- I've read that older RPi 5 firmware had problems booting from the SSD, but mine did not.
- Bonus: RPi5 has a "Power" jumper and Ubuntu 24.04 supports safe shutdown when a button is connected to it.

Here is how it looks:

![Screenshot from 2025-04-14 09-20-49](https://github.com/user-attachments/assets/2a3e53d0-b723-4924-9498-3f4653a00e9b)




----------------

**Back to** [Docs Folder](https://github.com/slgrobotics/robots_bringup/tree/main/Docs)

