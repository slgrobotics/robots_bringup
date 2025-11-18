**Back to** [Wiki](https://github.com/slgrobotics/articubot_one/wiki)

## Ubuntu Server 24.04 Real Time Kernel and ROS2 for Raspberry Pi 3B/4/5

**Why bother?** Real Time [kernel](https://ubuntu.com/blog/enable-real-time-ubuntu) offers better scheduling
to [ros2_control](https://control.ros.org/jazzy/doc/ros2_control/controller_manager/doc/userdoc.html), and therefore is a good choice for us.

### Real-time Ubuntu community kernel from Canonical (did not work for me on Nov 1, 2025)

There is an official release of [Real-time Ubuntu](https://ubuntu.com/real-time), including a version for Raspberry Pi: https://ubuntu.com/blog/enable-real-time-ubuntu

There's also a pre-configured image, which I tried - it worked fine **on a Raspberry Pi 4**  for me, but beware of security risk. See the section at the end.

If you already have a configured Ubuntu 24.04 Server **on a Raspberry Pi 5** - just use the following command:
```
sudo apt update
sudo apt upgrade
sudo apt install ubuntu-realtime
(installation fails:
    /usr/sbin/grub-mkconfig: 279: cannot create /boot/grub/grub.cfg.new: Directory nonexistent
    Using DTB: bcm2712-rpi-5-b.dtb
    Couldn't find DTB bcm2712-rpi-5-b.dtb on the following paths: /etc/flash-kernel/dtbs /usr/lib/linux-image-6.8.1-1015-realtime /lib/firmware/6.8.1-1015-realtime/device-tree/)
```
**Important:** ROS2 *Control Manager* discovers RT kernel and will adjust its scheduling to run at higher priority.
It will fail in subtle ways if not allowed to do so.

To enable scheduling adjustment, follow [this guide](https://control.ros.org/jazzy/doc/ros2_control/controller_manager/doc/userdoc.html) - create *realtime* group:
```
sudo addgroup realtime
sudo usermod -a -G realtime $(whoami)
```
and add the following limits to the realtime group in `/etc/security/limits.conf`:
```
@realtime soft rtprio 99
@realtime soft priority 99
@realtime soft memlock unlimited
@realtime hard rtprio 99
@realtime hard priority 99
@realtime hard memlock unlimited
```
The limits will be applied after you log out and in again.

So, here is *PREEMPT_RT* OS:
```
ros@urt:~$ uname -a
Linux urt 6.8.4-rt11-raspi #1 SMP PREEMPT_RT Mon May  5 16:51:52 UTC 2025 aarch64 aarch64 aarch64 GNU/Linux
```
Opposed to standard *PREEMPT_DYNAMIC* installation:
```
ros@seggy:~$ uname -a
Linux seggy 6.8.0-1040-raspi #44-Ubuntu SMP PREEMPT_DYNAMIC Wed Sep 24 18:30:35 UTC 2025 aarch64 aarch64 aarch64 GNU/Linux
```

So, the RT image is usable for my robots - with very little effort.

----------------

### A third-party image installation - worked for me on **Raspberry Pi 4 and 5**  (use at your own risk)

Repository and instructions [here](https://github.com/ros-realtime/ros-realtime-rpi4-image).

To decompress:
```
zstd ubuntu-24.04.2-rt-ros2-arm64+raspi.img.zst -d -o ubuntu-24.04.2-rt-ros2-arm64+raspi.img
```
I used *balenaEtcher* to flash the 4.3GB image, *Raspberry Pi imager* or Ubuntu *Disks app* can do it too (on the Ubuntu 24.04 Desktop machine).

I put the card in RPi 4, Ethernet cable, HDMI monitor and keyboard/mouse connected. Watch for password change prompts.

Terminal font looks OK on an average large monitor (yes, a bit small).

When running `sudo apt upgrade` it says, among other normal things, something like `generate /boot/...img...rt11-raspi` - I'd guess the RT kernel stays.

Installed missing network configuration packages:
```
sudo apt install net-tools avahi-daemon wsdd
sudo apt install python3-rosdep
```
Enabling ssh:
```
cd /etc/ssh/sshd_config.d
sudo vi 50-cloud-init.conf
```
make sure to change it to: "*PasswordAuthentication yes*"
```
sudo systemctl restart sshd
```
Created user "ros": 
```
sudo adduser ros
sudo adduser ros sudo
sudo adduser ros dialout
```
Logged in as "ros" and added two lines to the end of .bashrc:
```
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
```

#### Next steps 

I loosely followed my notes here: https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Ubuntu-RPi

The "*rosdep*" installed a huge bunch of packages, hard to say what was actually needed. Took very long time.
```
sudo apt install colcon
```
Compiled and ran Plucky "articubot_one" code, starts OK, but my RPi4 isn't on a robot - I see lots of missing hardware run-time messages from the nodes.

**P.S.** I moved the SD card to the RPi5 on Plucky, and it worked well. NVM SSD is still in place, the OS [boots](https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Ubuntu-RPi/NVM-SSD.md#booting-from-an-sd-card-with-nvm-drive-present) off the SD card.

So, the RT image is usable for all my robots - with some elbow grease applied.

I have 56% of 16GB SD card used (81% after additional packages were installed for Plucky robot).

**Note:**
- I initially configured the the SD card on a RPi 4, and then plugged in RPi 5 on Plucky robot. I haven't seen any adverse effects.
- Beware of [boot sequence](https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Ubuntu-RPi/NVM-SSD.md#booting-from-an-sd-card-with-nvm-drive-present) when you have NVM (SSD) and boot from an SD.

#### Compressing the SD card for backup:

I followed the process [here](https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Ubuntu-RPi/README.md#optional-making-compressed-backups-of-an-sd-card)
and got a zipped image of 3.5 GB (from the original 16 GB SD card). That's with all extras I installed in the process. I'm wondering why the original .zst image was 4.3 GB - wasn't washed well? ;-) 

----------------

**Back to** [Wiki](https://github.com/slgrobotics/articubot_one/wiki) or [Docs Folder](https://github.com/slgrobotics/robots_bringup/tree/main/Docs)
