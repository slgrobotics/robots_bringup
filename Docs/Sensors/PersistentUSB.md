# Making USB devices persistent

Typically, a robot has three USB-to-Serial devices: Arduino "wheels/base", GPS and LIDAR.

While the Arduino Mega is the only one at _/dev/ttyACM0_, GPS and LIDAR can take any name under _/dev/ttyUSB*_ pattern.

To avoid reassigning device names in the launch file after reboots, symlinks are created using the following recipe:

https://unix.stackexchange.com/questions/705570/setting-persistent-name-for-usb-serial-device-with-udev-rule-without-symlink

https://unix.stackexchange.com/questions/541156/udev-rules-for-usb-serial-by-path-not-working
```
$ udevadm info /dev/ttyUSB0 | grep "ID_PATH="

$ cat /etc/udev/rules.d/99-robot.rules   (edit it based on the output above)
SUBSYSTEM=="tty",ENV{ID_PATH}=="platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0",SYMLINK+="ttyUSBLDR"
SUBSYSTEM=="tty",ENV{ID_PATH}=="platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.2:1.0",SYMLINK+="ttyUSBGPS"

$ sudo udevadm control --reload-rules && sudo udevadm trigger
```
Now (or after reboot) you should see something similar to this (which devices symbolic links point to is random):
```
$ ll /dev/ttyU*
crw-rw---- 1 root dialout 188, 0 Nov 21  2023 /dev/ttyUSB0
crw-rw---- 1 root dialout 188, 1 Nov 21  2023 /dev/ttyUSB1
lrwxrwxrwx 1 root root         7 Nov 21  2023 /dev/ttyUSBGPS -> ttyUSB1
lrwxrwxrwx 1 root root         7 Nov 21  2023 /dev/ttyUSBLDR -> ttyUSB0
```
Check the GPS stream (note the baud rate, yours might be different):

**To exit picocom:** Press _Ctrl_ button and then without releasing it press **a** and then **q**.

```
sudo apt-get install picocom
picocom /dev/ttyUSBGPS -b 115200
```

----------------

**Back to** [Wiki](https://github.com/slgrobotics/articubot_one/wiki) or [Docs Folder](https://github.com/slgrobotics/robots_bringup/tree/main/Docs)
