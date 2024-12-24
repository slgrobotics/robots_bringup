# FPV Camera and receiver Setup

Here is my attempt to provide a zero-delay video link between robot and ROS2 Camera Publisher.

I used an affordable FPV drone camera with analog (NTSC) video transmitter, and an analog receiver capable of feeding video stream to computer via USB.

"*WT07 Micro 5.8GHz 25mW FPV Transmitter and 600TVL Camera with OSD Interface for FPV Quadcopter Drone Wireless Radio Transmission*":
https://www.amazon.com/dp/B06VY7L1N4

"*FPV Receiver 5.8G 150CH OTG Receiver UVC Video VTX 5dBi SMA Female*":
https://www.amazon.com/dp/B07Q5MPC8V

Camera with transmitter, of course, resides on the robot. The receiver is plugged into a Ubuntu 22.04 **Desktop USB port**, it shows up as _/dev/video0_ and _/dev/video1_

It works with Linux Cheese app and can be read by Python/OpenCV scripts, including custom ROS nodes written in Python.

Here is the code I use for the camera **on the Desktop side**: https://github.com/slgrobotics/camera_publisher

The video link is separated from WiFi and doesn't interfere with normal ROS2 traffic. Experiencing minimal delay allows driving the robot FPV-style and/or performing complex video stream processing on the Desktop.

**Note:** Ubuntu 24.04 introduced breaking changes in the way the video stream is handled ("libcamera" vs. "v4l2"). Cameras on Raspberry Pi 5 don't work anymore (at least, without extra setup effort - see discussion here https://groups.google.com/g/hbrobotics/c/4VITfijo2cM/m/80LidlKAAgAJ). This needs further exploration.
