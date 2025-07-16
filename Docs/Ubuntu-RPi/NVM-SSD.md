## Replace SD card with NVM on Raspberry 5

I upgraded my RPi5 on Plucky and Dragger, replacing the SD card with an SSD.

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

#### Booting from an SD card with NVM drive present

You might have another Ubuntu 24.04 image on an SD card (in my case, [Real Time OS](https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Ubuntu-RPi/UbuntuRealTime.md)).

Inserting such SD card and booting the RPi5 can be done safely, with the following caveat: the boot sequence **might** use the NVM's `/boot/firmware` partition.
"*Cold*" boot and rebooting seem to behave the same.

As a result, some essential configuration could be inherited from the NVM resident OS - for example, SD card's machine name is "*urt*", and the system booted under the name "*plucky*":
```
ros@plucky:~$ df -k
Filesystem     1K-blocks     Used Available Use% Mounted on
tmpfs             812728     7328    805400   1% /run
/dev/mmcblk0p2  14805116 11399128   2748160  81% /
tmpfs            4063628        4   4063624   1% /dev/shm
tmpfs               5120        0      5120   0% /run/lock
/dev/nvme0n1p1    516204   185136    331068  36% /boot/firmware
tmpfs             812724       16    812708   1% /run/user/1001

ros@plucky:~$ uname -a
Linux plucky 6.8.4-rt11-raspi #1 SMP PREEMPT_RT Mon May  5 16:51:52 UTC 2025 aarch64 aarch64 aarch64 GNU/Linux
```
This is how SD card mount looks:
```
/dev/mmcblk0p1    516204   176165    340040  35% /boot/firmware
```
There might be other less obvious consequences. To find out, ask your AI-enabled browser: 
- *"Raspberry pi 5 ubuntu 24.04 /boot/firmware configuration - what is possible to configure?"*

I didn't see any damage to the original OS after removing SD card and booting from NVM as usual.

To have the *boot* partition from the SD card mounted, you can specify its UUID in `/etc/fstab`, for example:
```
ros@plucky:~$ cat /etc/fstab
LABEL=writable	/	ext4	defaults	0	1
UUID=D595-E956	/boot/firmware	vfat	defaults	0	1
/swapfile swap swap defaults 0 0
```
To see UUIDs of all partitions:
```
ros@plucky:~/sys/nvm$ sudo blkid
/dev/mmcblk0p1: LABEL_FATBOOT="system-boot" LABEL="system-boot" UUID="D595-E956" BLOCK_SIZE="512" TYPE="vfat" PARTUUID="2943a0cf-01"
/dev/mmcblk0p2: LABEL="writable" UUID="5624f709-7cae-4bfd-81cd-3c8faf147752" BLOCK_SIZE="4096" TYPE="ext4" PARTUUID="2943a0cf-02"

/dev/nvme0n1p1: LABEL_FATBOOT="system-boot" LABEL="system-boot" UUID="F526-0340" BLOCK_SIZE="512" TYPE="vfat" PARTUUID="0529037a-01"
/dev/nvme0n1p2: LABEL="writable" UUID="1305c13b-200a-49e8-8083-80cd01552617" BLOCK_SIZE="4096" TYPE="ext4" PARTUUID="0529037a-02"
```
It is also possible to specify UUID of the root system in `/boot/firmware/cmdline.txt`, if NVM's *ext4* partition is wrongly mounted as "/" when booting from the SD card.

Useful commands:
- rpi-eeprom-config
- sudo rpi-eeprom-update

There's some additional info about boot order here:
- https://youtu.be/nJlTxIwRkRw
- https://www.raspberrypi.com/documentation/computers/raspberry-pi.html#nvme-ssd-boot

#### Here is how NVM hardware looks:

![Screenshot from 2025-04-14 09-20-49](https://github.com/user-attachments/assets/2a3e53d0-b723-4924-9498-3f4653a00e9b)

----------------

**Back to** [Docs Folder](https://github.com/slgrobotics/robots_bringup/tree/main/Docs)

