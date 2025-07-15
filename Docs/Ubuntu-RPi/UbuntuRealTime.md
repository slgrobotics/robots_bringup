## Ubuntu Server 24.04 Real Time Kernel and ROS2 for Raspberry Pi 3B/4/5

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
**Important:** ROS2 *Control Manager* discovers RT kernel and will adjust its scheduling to run at higher priority.
It will fail in subtle ways if not allowed to do so.

To enable that, follow [this guide](https://control.ros.org/jazzy/doc/ros2_control/controller_manager/doc/userdoc.html) - create *realtime* group
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

#### Next steps 

Loosely followed my notes here: https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Ubuntu-RPi

The "*rosdep*" installed a huge bunch of packages, hard to say what was actually needed. Took very long time.
```
sudo apt install colcon
```
Compiled and ran Plucky "articubot" code, starts OK, but my RPi4 isn't on a robot - I see lots of missing hardware run-time messages from the nodes.

**P.S.** I moved the SD card to the RPi5 on Plucky, and it worked well. NVM SSD is still in place, the OS [boots](https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Ubuntu-RPi/README.md#booting-from-an-sd-card-with-nvm-drive-present) off the SD card.

So, the RT image is usable for my robots - with some elbow grease applied.

BTW, here is RT OS:
```
ros@urt:~$ uname -a
Linux urt 6.8.4-rt11-raspi #1 SMP PREEMPT_RT Mon May  5 16:51:52 UTC 2025 aarch64 aarch64 aarch64 GNU/Linux
```
Opposed to standard installation:
```
ros@plucky:~$ uname -a
Linux plucky 6.8.0-1028-raspi #32-Ubuntu SMP PREEMPT_DYNAMIC Sat Apr 26 10:05:11 UTC 2025 aarch64 aarch64 aarch64 GNU/Linux
```
I have 56% of 16GB SD card used.

**Note:**
- I initially configured the the SD card on a RPi 4, and then plugged in RPi 5 on Plucky robot. I haven't seen any adverse effects.
- Beware of [boot sequence](https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Ubuntu-RPi#booting-from-an-sd-card-with-nvm-drive-present) when you have NVM (SSD) and boot from an SD.

### Compressing the SD card for backup:

I followed the process [here](https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Ubuntu-RPi/README.md#optional-making-compressed-backups-of-an-sd-card)
and got a zipped image of 3.5 GB (from the original 16 GB SD card). That's with all extras I installed in the process. I'm wondering why the original .zst image was 4.3 GB - wasn't washed well? ;-) 

### Real-time Ubuntu from Canonical

There is an official release of Real-time Ubuntu, including a version for Raspberry Pi 4 and 5: https://ubuntu.com/real-time

----------------

**Back to** [Docs Folder](https://github.com/slgrobotics/robots_bringup/tree/main/Docs)
