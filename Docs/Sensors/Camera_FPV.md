**Back to** [Wiki](https://github.com/slgrobotics/articubot_one/wiki)

# FPV Camera and receiver Setup

Here is my attempt to provide a zero-delay video link between robot and ROS2 Camera Publisher.

**Note:** for Raspberry Pi _native_ camera setup see https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/Camera.md

Initially I used an affordable FPV drone camera with analog (NTSC) video transmitter, and an analog receiver capable of feeding video stream to computer via USB.

"*WT07 Micro 5.8GHz 25mW FPV Transmitter and 600TVL Camera with OSD Interface for FPV Quadcopter Drone Wireless Radio Transmission*":
https://www.amazon.com/dp/B06VY7L1N4

"*FPV Receiver 5.8G 150CH OTG Receiver UVC Video VTX 5dBi SMA Female*":
https://www.amazon.com/dp/B07Q5MPC8V

Camera with transmitter, of course, resides on the robot. The receiver is plugged into a Ubuntu 22.04 **Desktop USB port**, it shows up as _/dev/video0_ and _/dev/video1_

It works with Linux Cheese app and can be read by Python/OpenCV scripts, including custom ROS nodes written in Python.

Here is the code I use for the camera **on the Desktop side**: https://github.com/slgrobotics/camera_publisher

The video link is separated from WiFi and doesn't interfere with normal ROS2 traffic. Experiencing minimal delay allows driving the robot FPV-style and/or performing complex video stream processing on the Desktop.

I've chosen to use a more powerful FPV set on my Dragger robot, keeping the same USB receiver:
- Foxeer Razer Mini FPV [Camera](https://www.amazon.com/Foxeer-Camera-Razer-2-1mm-1200TVL/dp/B07ZKPDPLM)
- TS832 48Ch 5.8G FPV [Transmitter](https://www.amazon.com/dp/B06XKQ8466)

**Notes:** 
1. The cameras and receiver I tested are designed to work outdoors within visual range, without obstructions. Indoors they were performing poorly.
2. TS832 transmitter runs _very_ hot at 12V. You better feed it with 7 to 9 volts via a DC-DC [converter](https://www.amazon.com/Converter-DROK-Voltage-Regulator-Waterproof/dp/B0B6PP65S1).
3. Any transmitter will be burned if antenna is not connected or is not of good quality or is not designed for 5.8 GHz.
4. Use "_cheese_" app on your Desktop machine to view video stream. GNOME "_snapshot_" doesn't seem to work with this USB receiver.
5. Video quality is 480x320 - very poor. Good enough for "driving blind" though.
6. Ubuntu 24.04 introduced breaking changes in the way the video stream is handled ("libcamera" vs. "V4L2").
Cameras on Raspberry Pi 5 don't work anymore (at least, without extra [setup effort](https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/Camera.md)).

![Screenshot from 2025-04-26 20-09-23](https://github.com/user-attachments/assets/c83e8411-dd61-4c0d-baf3-018e519d35b5)

![Screenshot from 2025-04-26 20-10-02](https://github.com/user-attachments/assets/b683894a-0616-4cff-bd62-716f62b0d5b9)

![Screenshot from 2025-04-26 20-10-46](https://github.com/user-attachments/assets/f4d87511-a452-47f7-8bdd-15d66b278c60)

----------------

**Back to** [Wiki](https://github.com/slgrobotics/articubot_one/wiki) or [Docs Folder](https://github.com/slgrobotics/robots_bringup/tree/main/Docs)
