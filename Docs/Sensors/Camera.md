## _Native_ Camera Setup

Ubuntu 24.04 introduced breaking changes in the way the video stream is handled ("libcamera" vs. "v4l2"). Cameras on Raspberry Pi 5 don't work anymore under Ubuntu 24.04. 
See our Club discussion [here](https://groups.google.com/g/hbrobotics/c/4VITfijo2cM/m/80LidlKAAgAJ).

Fortinately, Marco Walther did the research and all the hard work of porting the latest changes in _RPi OS libcamera_ code to a set of binaries, which can be easily installed on a Raspberry Pi under Ubuntu 24.04.

His work is here: https://launchpad.net/~marco-sonic/+archive/ubuntu/rasppios

I followed Marco's directions (and his [personal advice](https://groups.google.com/g/hbrobotics/c/d2Ir8ifMFhA), thanks!) to get my camera working.

Here is my setup:
-  "8Mp Camera V5": https://www.amazon.com/dp/B0DLGMT7XN  (originally "[Arducam](https://www.arducam.com/product/8mp-imx219-camera-module-for-raspberry-pi-b0394/)", has Sony IMX219 sensor)
-  Raspberry Pi 5 8GB
-  Ubuntu 24.04 Server, ROS2 Jazzy Base ("headless" configuration, no GUI)

**Note:**
- I plugged the cable in "CAM/DISP 1" port - you might use port 0 instead. I haven't tried.
- [Amazon](https://www.amazon.com/dp/B0DLGMT7XN) has good image showing proper orientation of cable.
- Make sure you use cable "locks" properly and secure the cable 

![arducam-rpi](https://github.com/user-attachments/assets/e03df469-85ec-4ac9-98a2-d751ef0daf2f)

## Installation

**Important Warning:** 
- The contents of [Marco's Personal Package Archives](https://launchpad.net/~marco-sonic/+archive/ubuntu/rasppios) are _not checked or monitored_. You install software from them **at your own risk**. 
```
sudo add-apt-repository ppa:marco-sonic/rasppios
sudo apt update
sudo apt upgrade
# on my machine previously installed "libcamera" package was automatically upgraded with Marco's binary.
sudo apt install libcamera-tools rpicam-apps-lite python3-picamera2
```
Reboot

## Testing

After installation you should see _/dev/video*_ and _/dev/media*_ descriptors.

Try: 
```
libcamera-jpeg -o test.jpg
```




## ROS2 Camera Publisher.

Here is the code I use for the camera **on the Desktop side**: https://github.com/slgrobotics/camera_publisher

**Note:** 
1. The camera and receiver I tested are designed to work outdoors within visual range, without obstructions. Indoors they were performing poorly.
2. Ubuntu 24.04 introduced breaking changes in the way the video stream is handled ("libcamera" vs. "v4l2"). Cameras on Raspberry Pi 5 don't work anymore (at least, without extra setup effort - see discussion here https://groups.google.com/g/hbrobotics/c/4VITfijo2cM/m/80LidlKAAgAJ). This needs further exploration.
