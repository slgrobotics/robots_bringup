## _Native_ Raspberry Pi Camera (_Arducam_) Setup

Ubuntu 24.04 introduced breaking changes in the way the video stream is handled ("libcamera" vs. "v4l2"). 
So, [CSI-connected Cameras](https://www.raspberrypi.com/documentation/accessories/camera.html) on Raspberry Pi 5 don't work anymore under Ubuntu 24.04. 
See our Club discussion [here](https://groups.google.com/g/hbrobotics/c/4VITfijo2cM/m/80LidlKAAgAJ).

Fortinately, Marco Walther did the research and all the hard work of porting the latest changes in _RPi OS libcamera_ code to Ubuntu 24.04.
He created a set of binaries, which can be easily installed on a Raspberry Pi under Ubuntu 24.04.

His work is here: https://launchpad.net/~marco-sonic/+archive/ubuntu/rasppios

I followed Marco's directions (and his [personal advice](https://groups.google.com/g/hbrobotics/c/d2Ir8ifMFhA), thanks!) to get my camera working.

Here is my setup:
-  "8Mp Camera V5": https://www.amazon.com/dp/B0DLGMT7XN  (originally "[Arducam](https://www.arducam.com/product/8mp-imx219-camera-module-for-raspberry-pi-b0394/)", has a Sony IMX219 sensor)
-  Raspberry Pi 5 8GB
-  Ubuntu 24.04 _Server_, ROS2 Jazzy _Base_ ("headless" configuration, no GUI)

**Note:**
- I plugged the cable in "CAM/DISP 1" port - you might use port 0 instead. I haven't tried.
- [Amazon](https://www.amazon.com/dp/B0DLGMT7XN) has good image showing proper orientation of cable.
- Make sure you use cable "locks" properly and secure the cable 

![arducam-rpi](https://github.com/user-attachments/assets/e03df469-85ec-4ac9-98a2-d751ef0daf2f)

## Installation

Following Marco's recommendations, review Arducam guide:

https://docs.arducam.com/Raspberry-Pi-Camera/Native-camera/8MP-IMX219/#software-configuration

edit */boot/firmware/config.txt*
```
#Find the line: camera_auto_detect=1, update it to:
camera_auto_detect=0
#Find the line: [all], add the following item under it:
dtoverlay=imx219
```
The last line can explicitly specify port - _cam0_ or _cam1_ ("CAM/DISP 1" port is located closer to HDMI and power connectors):
```
dtoverlay=imx219,cam0
```

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

## Basic testing

After installation you should see _/dev/video*_ and _/dev/media*_ descriptors: ```v4l2-ctl --list-devices```

Try: 
```
sudo dmesg | fgrep -i imx
    [    2.909225] platform 1f00128000.csi: Fixed dependency cycle(s) with /axi/pcie@120000/rp1/i2c@80000/imx219@10
    [    5.738149] rp1-cfe 1f00128000.csi: found subdevice /axi/pcie@120000/rp1/i2c@80000/imx219@10
    [    5.738170] rp1-cfe 1f00128000.csi: Using sensor imx219 4-0010 for capture

Marco's note: "imx" works for the Sony sensors, but ov<num> potentially for others, ...
              That helps to verify, that the kernel actually found the device.
              If that does not show anything (with a couple different/relevant string
              fragments) or shows errors, chances are, nothing afterwards will work.

LIBCAMERA_LOG_LEVELS=*:DEBUG cam -l"
    # shows the camera:
    Available cameras:
    1: 'imx219' (/base/axi/pcie@120000/rp1/i2c@80000/imx219@10)

libcamera-jpeg -o test.jpg
    # produces a file:
    -rw-rw-r-- 1 ros ros 1318903 Mar 21 14:03 test.jpg

rpicam-hello --list -v
Available cameras
-----------------
0 : imx219 [3280x2464 10-bit RGGB] (/base/axi/pcie@120000/rp1/i2c@80000/imx219@10)
    Modes: 'SRGGB10_CSI2P' : 640x480 [103.33 fps - (1000, 752)/1280x960 crop]
                             1640x1232 [41.85 fps - (0, 0)/3280x2464 crop]
                             1920x1080 [47.57 fps - (680, 152)/1920x2160 crop]
                             3280x2464 [21.19 fps - (0, 0)/3280x2464 crop]
           'SRGGB8' : 640x480 [103.33 fps - (1000, 752)/1280x960 crop]
                      1640x1232 [41.85 fps - (0, 0)/3280x2464 crop]
                      1920x1080 [47.57 fps - (680, 152)/1920x2160 crop]
                      3280x2464 [21.19 fps - (0, 0)/3280x2464 crop]

    Available controls for 3280x2464 SRGGB10_CSI2P mode:
    ----------------------------------------------------
    AeConstraintMode : [0..3]
    AeEnable : [false..true]
    AeExposureMode : [0..3]
    AeFlickerMode : [0..1]
    AeFlickerPeriod : [100..1000000]
    AeMeteringMode : [0..3]
    AnalogueGain : [1.000000..10.666667]
    AwbEnable : [false..true]
    AwbMode : [0..7]
    Brightness : [-1.000000..1.000000]
    CnnEnableInputTensor : [false..true]
    ColourGains : [0.000000..32.000000]
    ColourTemperature : [100..100000]
    Contrast : [0.000000..32.000000]
    ExposureTime : [75..1238765]
    ExposureValue : [-8.000000..8.000000]
    FrameDurationLimits : [47183..1238841]
    HdrMode : [0..4]
    NoiseReductionMode : [0..4]
    Saturation : [0.000000..32.000000]
    ScalerCrop : [(0, 0)/51x39..(0, 0)/3280x2464]
    Sharpness : [0.000000..16.000000]
    StatsOutputEnable : [false..true]
    SyncFrames : [1..1000000]
    SyncMode : [0..2]
```

If you have a Ubuntu with GUI ("Desktop" edition) - 
try [GNOME Snapshot](https://discourse.ubuntu.com/t/whats-happening-in-noble-repositories/43729/40) or _Cheese_.

**Note:** _Cheese_ has been removed in Ubuntu 24.04 - replaced by _gnome-snapshot_. It can be still installed via _snap_.

## Testing Python bindings

Make a file with the following content:
```
ros@plucky:~/camera_ws/tests$ cat test.py

from picamera2 import Picamera2, Preview
import time

picam2 = Picamera2()
# the following line requires GUI, won't work for headless machines:
#picam2.start_preview(Preview.QTGL) # other options include Preview.DRM, Preview.FULLSCREEN
preview_config = picam2.create_preview_configuration()
picam2.configure(preview_config)

picam2.start()
time.sleep(2)

picam2.capture_file("image.jpg")
picam2.close()
```
Run it and see the output file:
```
ros@plucky:~/camera_ws/tests$ python3 test.py
[18:49:01.212852746] [8762]  INFO Camera camera_manager.cpp:327 libcamera v0.4.0+53-29156679
[18:49:01.232339009] [8765]  INFO RPI pisp.cpp:720 libpisp version v1.0.7 28196ed6edcf 14-03-2025 (00:01:06)
[18:49:01.243382919] [8765]  INFO RPI pisp.cpp:1179 Registered camera /base/axi/pcie@120000/rp1/i2c@80000/imx219@10
                             to CFE device /dev/media2 and ISP device /dev/media0 using PiSP variant BCM2712_C0
[18:49:01.246391493] [8762]  WARN V4L2 v4l2_pixelformat.cpp:346 Unsupported V4L2 pixel format RPBP
[18:49:01.246882808] [8762]  INFO Camera camera.cpp:1202 configuring streams:
                             (0) 640x480-XBGR8888 (1) 640x480-BGGR_PISP_COMP1
[18:49:01.246996049] [8765]  INFO RPI pisp.cpp:1484 Sensor: /base/axi/pcie@120000/rp1/i2c@80000/imx219@10
                             - Selected sensor format: 640x480-SBGGR10_1X10 - Selected CFE format: 640x480-PC1B

ros@plucky:~/camera_ws/tests$ ll
total 1328
drwxrwxr-x 2 ros ros    4096 Mar 21 14:03 ./
drwxrwxr-x 7 ros ros    4096 Mar 20 19:55 ../
-rw-rw-r-- 1 ros ros   27835 Mar 21 14:14 image.jpg
-rw-rw-r-- 1 ros ros 1318903 Mar 21 14:03 test.jpg
-rw-rw-r-- 1 ros ros     335 Mar 21 10:51 test.py
ros@plucky:~/camera_ws/tests$ 
```

## ROS2 Camera Publisher.

*this section is work in progress, please visit later*
  
Here is the code I use for the camera before on the Desktop machine: [camera_publisher](https://github.com/slgrobotics/camera_publisher/blob/main/cv_basics/webcam_pub.py)

It relies on OpenCV _V4L2_ bindings, which don't work for IMX219 sensor.
There is some discussion about it [here](https://stackoverflow.com/questions/75463789/error-pipeline-have-not-been-created-in-python-opencv)

I might create a version of my *camera_publisher* to capture the image using *pycamera2*, not OpenCV.
The tricky part is "*CvBridge # Package to convert between ROS and OpenCV Images*" - whatever I capture in Pycamera2 must be converted to ROS2 format.

## Useful links

Here is a document from Ross Lunan with more detail and coverage of other _native_ cameras:

https://github.com/ARLunan/Raspberry-Pi-Camera-ROS



