## Raspberry Pi 3B/4/5 with Ubuntu - extra steps

*for fine-tuning your Raspberry Pi Ubuntu installation, review the following:*

### Properly feeding your Raspberry Pi 5

Here is a [video](https://youtu.be/PddIeZP-wgw) explaining why our Pi5's on the robots (off the batteries) don't perform properly:

The video describes settings that fix the problem (use caution).

Here is a $30 "*easy solution*":
- https://52pi.com/collections/hat-addons/products/52pi-pd-power-extension-adapter-board-for-raspberry-pi-5
- https://www.amazon.com/GeeekPi-Expansion-Raspberry-Automatic-Function/dp/B0CYPRDY9Q

### _Optional:_ VS Code Remote Development

You can work on the Desktop machine with repositories/folders on Raspberry Pi.

Follow [this guide](https://code.visualstudio.com/docs/remote/ssh-tutorial).

### _Optional:_ Using UARTs on the Raspberry Pi 5

I rarely have a need to use Raspberry Pi UARTs,
so for useful information please refer to the source - Michael Wimble's repository.

To use Raspberry Pi UARTs refer to the following section.

**Warning:** I will not accept any resposibility for damage to any device as a result of you following these instructions. Neither will Michael Wimble, the original author.

https://github.com/wimblerobotics/ros2_roboclaw_driver?tab=readme-ov-file#using-uarts-on-the-raspberry-pi-5

You may also review this [section](https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/GPS.md#gps-on-raspberry-pi-uart-gpio-1415) of my notes.

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

Follow [this guide](https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Ubuntu-RPi/NVM-SSD.md).

----------------

**Back to** [Docs Folder](https://github.com/slgrobotics/robots_bringup/tree/main/Docs)
