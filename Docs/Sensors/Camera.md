# FPV Camera and receiver Setup

https://www.amazon.com/dp/B06VY7L1N4

https://www.amazon.com/dp/B07Q5MPC8V

Camera and transmitter, of course, resides on Dragger. The receiver, when plugged into a Ubuntu 22.04 **Desktop USB port**, shows up as _/dev/video0_ and _video1_

It works with Cheese app and can be read by Python/OpenCV scripts, including custom ROS nodes written in Python.

Here is the code I use for the camera **on the Desktop side**: https://github.com/slgrobotics/camera_publisher

Having the video link separated from WiFi and experiencing minimal delay allows driving the robot FPV-style and/or performing video stream processing on the Desktop.


