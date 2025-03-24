## _Native_ Raspberry Pi Camera (_Arducam_) Setup

Ubuntu 24.04 introduced breaking changes in the way the video stream is handled ("libcamera" vs. "V4L2"). 
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
Make sure you are a member of group "_video_", for example my "_ros_" account already is:
```
ros@plucky:~$ sudo adduser ros video
[sudo] password for ros: 
info: The user `ros' is already a member of `video'.
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
...
-rw-rw-r-- 1 ros ros   27835 Mar 21 14:14 image.jpg
```
With a little help from OpenCV (and Marco ;-)) we can get more processing powers:
```
from picamera2 import Picamera2, Preview
import cv2
import time
# see https://datasheets.raspberrypi.com/camera/picamera2-manual.pdf
#     https://docs.opencv.org/3.4/d8/d01/group__imgproc__color__conversions.html
video_w = 640
video_h = 480
with Picamera2() as picam2:
    # Configure and start Picamera2.
    picam2.video_configuration.main.size = (video_w, video_h)
    #conf_main = {'size': (video_w, video_h), 'format': 'XBGR8888'}
    config = picam2.create_preview_configuration()
    #config = picam2.create_preview_configuration(conf_main)
    picam2.configure(config)
    picam2.start()
    while True:
        frame = picam2.capture_array('main')
        cv_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        print("IP: saving frame...")
        cv2.imwrite("image_picamera2_cv2.jpg", cv_rgb)
        print("OK: frame saved")
        exit()
```

## Python, OpenCV and GStreamer

A common way of capturing frames in Python scripts is as follows:
```
import cv2
cap = cv2.VideoCapture(0)
...
```
It doesn't work with Raspberry Pi _native_ cameras and _libcamera_. 
Fortunately, OpenCV can be built with [GStreamer](https://gstreamer.freedesktop.org/features/) support, and _libcamerasrc_ plugin feeds the pipeline, if _libcamera_ is working properly.

First, we need to install required components:
```
sudo apt install gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad
# I am not sure that "gstreamer1.0-plugins-bad" is needed here
sudo apt install gstreamer1.0-libcamera
```
Now you can see if your camera shows up properly in GStreamer queries:
```
gst-device-monitor-1.0 Video
gst-inspect-1.0 libcamerasrc
gst-inspect-1.0 videoconvert
```
Note your camera name:
```
libcamera-hello --list-cameras
   or
gst-device-monitor-1.0 Video 2>/dev/null | grep name
```
Here is a Python script that works for me:
```
import cv2
#print(cv2.getBuildInformation())  # see if your OpenCV was built with GStreamer support (mine was, no need to build from sources)

# camera name and available resolutions: libcamera-hello --list-cameras
camera = "/base/axi/pcie@120000/rp1/i2c@80000/imx219@10"

print("IP: opening camera: ", camera)

cam_pipeline_str = "libcamerasrc camera-name=%s ! video/x-raw,width=640,height=480,framerate=10/1,format=RGBx ! videoconvert ! videoscale ! video/x-raw,width=640,height=480,format=BGR ! appsink" % (camera)
# shorter version, produces 1280x1080 frames:
#cam_pipeline_str = 'libcamerasrc camera-name=%s ! video/x-raw,format=RGBx ! videoconvert ! appsink' % (camera)

cap = cv2.VideoCapture(cam_pipeline_str, cv2.CAP_GSTREAMER)
#cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: open camera failed")
    exit()

print("OK: open camera successful")

while True:
    ret, frame = cap.read()
    if ret:
        print("OK: frame read successful")
        # cv2.imshow('frame', frame)
        # cv2.imwrite("captured_image.jpg", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        print("Error: frame read failed")
        break

cap.release()
cv2.destroyAllWindows()
```
To see discussions about this, follow to these links:
- https://github.com/raspberrypi/libcamera/issues/163
- https://github.com/opencv/opencv/issues/25072
- https://libcamera.org/getting-started.html

## ROS2 Camera Publisher.

Here is the code I used for FPV camera before on the Desktop machine: [camera_publisher](https://github.com/slgrobotics/camera_publisher/blob/main/cv_basics/webcam_pub.py)

It relies on OpenCV _V4L2_ bindings, which don't work for IMX219 sensor with ```cv2.VideoCapture(0)``` - see above chapter.

Using OpenCV allows some image processing and publishing custom topics. 
For example, I could detect color blobs and publish their offsets.
Image processing in a camera node makes sense in FPV scenario (NTSC receiver connected to the Desktop, where resources are plentiful), but not on the robot's resource-limited RPi.

Someday I might create a version of my *camera_publisher* to capture the image using *pycamera2*, not OpenCV - or use GStreamer to feed OpenCV properly.
With Pycamera2, the tricky part is "*CvBridge # Package to convert between ROS and OpenCV Images*" - whatever I capture in Pycamera2 must be converted to ROS2 format.
GStreamer, while relatively simple to use, is rumored to be CPU-hungry.

Meanwhile, Marco recommended https://github.com/christianrauch/camera_ros.

I was able to run it and see the video on my Desktop machine at ~15 FPS:
```
RPi (Plucky):
cd ~/robot_ws/src
git clone https://github.com/christianrauch/camera_ros.git
cd ~/robot_ws
# Do it once. Note the "--skip-keys=libcamera" which preserves Marco's installed package:
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} --skip-keys=libcamera -r -y
colcon build

source install/setup.bash
ros2 run camera_ros camera_node --ros-args -p width:=640 -p height:=480 -p camera:=0

Desktop:
ros2 run image_view image_view --ros-args -r image:=/camera/image_raw -p image_transport:=compressed
```
You can see images published at 15 FPS:
```
ros@plucky:~$ ros2 topic hz /camera/image_raw
average rate: 14.990
min: 0.065s max: 0.069s std dev: 0.00118s window: 16
```
**Note:**

If during a system update or after running rosdep Marco's packages are replaced, you can fix that easily, for example:
```
ros@plucky:~/cam_ws$ apt list --installed |grep libcamera
libcamera-dev/noble,now 0.4.0+rpt20250213-1ubuntu1~marco1 arm64 [installed]
libcamera-ipa/noble,now 0.4.0+rpt20250213-1ubuntu1~marco1 arm64 [installed,automatic]
libcamera-tools/noble,now 0.4.0+rpt20250213-1ubuntu1~marco1 arm64 [installed]
libcamera0.2/noble,now 0.2.0-3fakesync1build6 arm64 [installed,auto-removable] <- this is bad
libcamera0.4/noble,now 0.4.0+rpt20250213-1ubuntu1~marco1 arm64 [installed,automatic]
python3-libcamera/noble,now 0.4.0+rpt20250213-1ubuntu1~marco1 arm64 [installed]
ros-jazzy-libcamera/noble,now 0.4.0-1noble.20250102.132330 arm64 [installed] <- this might be bad

ros@plucky:~/cam_ws$ sudo apt remove libcamera0.2 ros-jazzy-libcamera
```

## Useful links

Here is a document from Ross Lunan with more detail (and coverage of other _native_ cameras):

https://github.com/ARLunan/Raspberry-Pi-Camera-ROS

Note the following ROS2 package intended for image processing (including stereo, depth etc.):

https://github.com/ros-perception/image_pipeline
