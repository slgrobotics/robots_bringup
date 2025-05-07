## OAK-D Lite setup

I have a "_Kickstarter Edition_" OAK-D Lite stereo camera. It doesn't have an IMU, but otherwise should be identical to units produced later.

This guide describes how to run it under Ubuntu 24.04 and ROS2 Jazzy - first on an Intel 5 Desktop, an then on Raspberry 4 or 5 under the Ubuntu 24.04 Server and ROS2 Jazzy Base.

## Basic hardware test

The camera requires a quality USB3 cable and _prefers_ a USB3 socket, but will work if plugged into USB2 socket. 

After plugging it in, see if it shows up:
```
sudo dmesg
[11017.723133] usb 7-1: new high-speed USB device number 3 using xhci_hcd
[11017.916696] usb 7-1: New USB device found, idVendor=03e7, idProduct=2485, bcdDevice= 0.01
[11017.916708] usb 7-1: New USB device strings: Mfr=1, Product=2, SerialNumber=3
[11017.916712] usb 7-1: Product: Movidius MyriadX
[11017.916715] usb 7-1: Manufacturer: Movidius Ltd.
[11017.916717] usb 7-1: SerialNumber: 03e72485

lsusb
Look for "ID 03e7:2485 Intel Movidius MyriadX"
```
Do not expect _/dev/video*_ descriptor appear, OAK-D uses different mechanism for accessing hardware.

Prepare and apply _udev_ rules:
```
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## Power consumption and USB connection requirements

In theory, OAK-D camera can adjust to available USB link speed and will report it (*"LOW", "FULL", "HIGH", "SUPER", "SUPER_PLUS"*).
In practice, low quality or busy USB hubs cause device pipeline crashing.

It seems that my OAK-D Lite was unstable when connected to some USB ports (see *The "luxonis Device crashed..." bug* below).
Either on an Intel PC, or on Raspberry Pi 5, the best results (stability) is reached when there are no other devices connected to the same motherboard USB hub.

On the Raspberry Pi 5 use lower USB3 (blue) slot, leaving the other empty.
Use a [quality USB hub](https://www.amazon.com/dp/B0CJ95CR8X) to extend one of the USB2 (black) slots to connect other devices.

I also used a quality [USB-C cable](https://www.amazon.com/dp/B0BWHTX1R7).

My OAK-D LITE camera consumes 0.3 A when running stereo publishing tasks. It doesn't heat up at all.
Power consumption goes up significantly when running AI models (YOLO etc.), and the camera can easily overheat if mounted in a tight space.

## Luxonis Software Installation

There is a collection of python examples, which is not needed for ROS2, but is useful for testing. 

It can be only installed if you have a **Desktop** edition of Ubuntu 24.04

Note the _virtual environment_ setup and activation.
```
mkdir ~/depthai
cd ~/depthai
git clone https://github.com/luxonis/depthai-python.git
cd ~/depthai/depthai-python/examples
sudo apt install python3.12-venv

python3 -m venv ~/depthai/depthai-python/.env
source ~/depthai/depthai-python/.env/bin/activate
python3 install_requirements.py
```
Here is how to see video preview in a window (note Python _virtual environment_ activation): 
```
ros@machine:~$ source ~/depthai/depthai-python/.env/bin/activate
(.env) ros@machine:~$ cd ~/depthai/depthai-python/examples
(.env) ros@machine:~/depthai/depthai-python/examples$ python3 ColorCamera/rgb_preview.py
Connected cameras: [
    {socket: CAM_A, sensorName: IMX214, width: 4208, height: 3120, orientation: AUTO,
             supportedTypes: [COLOR], hasAutofocus: 0, hasAutofocusIC: 1, name: color},
    {socket: CAM_B, sensorName: OV7251, width: 640, height: 480, orientation: AUTO,
             supportedTypes: [MONO], hasAutofocus: 0, hasAutofocusIC: 0, name: left},
    {socket: CAM_C, sensorName: OV7251, width: 640, height: 480, orientation: AUTO,
             supportedTypes: [MONO], hasAutofocus: 0, hasAutofocusIC: 0, name: right}]
Usb speed: SUPER
Device name: OAK-D-LITE  Product name: OAK-D-LITE
```
Note the "_Usb speed_" value - it will be "_SUPER_" on USB3 and "_HIGH_" if plugged into an USB2 socket.

## Configuration and Troubleshooting

There's a number of [environment variables](https://docs.oakchina.cn/projects/api/components/device.html#environment-variables), which can help reconfiguring camera system.

Use ```export DEPTHAI_LEVEL=info; export XLINK_LEVEL=info``` to display temperatures (default is "_warn_", see https://docs.luxonis.com/software/depthai/debugging/ for options).

Most operation parameters (frame size etc.) are convigured via arguments to launch files.

## ROS2 operation

ROS2 Jazzy has binary distribution:
```
sudo apt install ros-${ROS_DISTRO}-depthai-ros

ros2 launch depthai_ros_driver camera.launch.py
```
By default, the */oak/rgb/image_raw* has 1280x720 size, and can be consumed raw or compressed. It is published at ~10 Hz, if run locally on Intel I7 Desktop.

## ROS2 examples

With *depthai-ros* installation you are getting the following packages (under */opt/ros/jazzy/share*):
```
depthai_bridge
depthai_descriptions
depthai_examples
depthai_filters
depthai-ros
depthai_ros_driver
depthai_ros_msgs
```
If you are working on a Desktop, the following example will show *pointCloud* in RViz2:
```
ros2 launch depthai_examples stereo.launch.py

   or

ros2 launch depthai_ros_driver pointcloud.launch.py use_rviz:=True
```    

Other examples produce topics which can be viewed in _rqt_:
```
ros2 launch depthai_examples rgb_stereo_node.launch.py
ros2 launch depthai_examples tracker_yolov4_node.launch.py
ros2 launch depthai_examples tracker_yolov4_spatial_node.launch.py
ros2 launch depthai_examples mobile_publisher.launch.py
```
There are examples above that publish data from [_yolo_](https://encord.com/blog/yolo-object-detection-guide/) model. Note that camera resolution in the examples is downgraded to 300x300 or 480P, so the frame rate and CPU load becomes reasonable (I saw 10 FPS, 34 FPS).

Refer to [this guide](https://docs.luxonis.com/software/ros/depthai-ros/driver) for more.

## Real life scenario

I have OAK-D Lite camera on my Plucky robot, which runs Ubuntu 24.04 Server - a "headless" configuration, no way to run examples that require Desktop.

So, to see the PointCloud (`ros2 launch depthai_examples stereo.launch.py`) I had to separate _RViz2_ from other nodes.

#### On Plucky Raspberry Pi 5:
```
cd /opt/ros/jazzy/share/depthai_examples/launch
sudo cp stereo.launch.py stereo.launch.node.py
sudo vi stereo.launch.node.py
```
Comment out line 219, which runs RViz2 GUI:
```
#ld.add_action(rviz_node)
```
Launch all nodes except "rviz_node":
```
cd ~/
ros2 launch depthai_examples stereo.launch.node.py monoResolution:=400p
```
`monoResolution:=400p` or `monoResolution:=480p` works. If skipped, a warning will be shown and default 480p will be used.

#### On the Desktop machine:
```
cd /opt/ros/jazzy/share/depthai_examples/launch
sudo cp stereo.launch.py stereo.launch.rviz.py
sudo vi stereo.launch.rviz.py
```
comment out lines 214...218, leaving "rviz_node" action:
```
    #ld.add_action(stereo_node)
    #ld.add_action(urdf_launch)

    #ld.add_action(metric_converter_node)
    #ld.add_action(point_cloud_node)
    ld.add_action(rviz_node)
    return ld
```
Now you can launch *rviz_node* alone:
```
cd ~/
ros2 launch depthai_examples stereo.launch.rviz.py
```
The _PointCloud2_ (_/stereo/points_ topic) is published at ~2 Hz.

A bonus */right/image_rect* topic shows the live stream from one of the OAK-D cameras (at 28..30 FPS).

Overall load on the WiFi link is around 200 Mbits/s (run `nload wlan0` on RPi).

![Screenshot from 2025-04-26 20-54-00](https://github.com/user-attachments/assets/f2c12651-7488-46ef-932a-3578fb49d225)

![Screenshot from 2025-04-26 21-01-24](https://github.com/user-attachments/assets/50a04441-a0a0-4d45-acc5-02794227c632)

## The "*luxonis Device crashed, but no crash dump could be extracted*" bug

At the time of this writing Luxonis ROS Jazzy code (or OAK_D LITE USB hardware) has a bug, which you will very likely experience.

After a short time (between several seconds and several minutes) of opreation, the camera "driver" freezes and its pipelines would stop delivering data.

You can see that topics are not being published anymore.
If you interrupt your launch with _Cntrl/C_, you can see an error message cited in this section's header, for example:
```
[component_container-2] [ERROR] [1746405049.622991459] [oak]: No data on logger queue!
[component_container-2] [ERROR] [1746405049.623271624] [oak]: Camera diagnostics error: Communication exception - possible device error/misconfiguration. Original message 'Couldn't read data from stream: 'sys_logger_queue' (X_LINK_ERROR)'
^C[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)
[component_container-2] [INFO] [1746405049.793764859] [rclcpp]: signal_handler(signum=2)
[component_container-2] [INFO] [1746405049.793949623] [oak]: Stopping camera.
[component_container-2] Failed to publish log message to rosout: publisher's context is invalid, at ./src/rcl/publisher.c:423
[INFO] [rviz2-1]: process has finished cleanly [pid 12852]
[rviz2-1] [INFO] [1746405049.793806129] [rclcpp]: signal_handler(signum=2)
 ***** here it is *****
[component_container-2] [184430102185711200] [7.1] [1746405051.508] [host] [warning] Device crashed, but no crash dump could be extracted.
 **********************
[component_container-2] [INFO] [1746405053.004532725] [oak]: Camera stopped!
[component_container-2] Failed to publish log message to rosout: publisher's context is invalid, at ./src/rcl/publisher.c:423
[INFO] [component_container-2]: process has finished cleanly [pid 12853]
```
This is a known bug, and it looks like Luxonis knows about it:

https://discuss.luxonis.com/d/4835-oak-d-crashes-on-different-examples

In my testing, I experience it a lot, while using quality USB 3.2 cables and external power through a USB 3.2 Hub. 
It is random, sometimes the device "just works" with USB 2.0 connections and very simple cable, and it doesn't seem to be dependent on power.
It can work on Raspberry Pi 5 better than on an Intel Desktop.

I am not equipped to debug it for real, but after trying a lot of combinations of machines, cables and USB connections,
I am convinced that the bug disappears when there's no other USB devices connected to hubs all the way to the machine.
For example, when I have all USB3 sockets unused, except for OAK-D LITE, the camera works fine forever.
But if I connect a card reader to the USB3 socket next to it, random crashes start happening.

To me it looks like the only way to deal with it now is to write a custom node with a monitoring thread, which will recreate whole set of pipelines once data stops coming.
Here us my attempt to do just that: https://github.com/slgrobotics/depthai_rospi

## Useful Links

Basic setup: https://robofoundry.medium.com/oak-d-lite-camera-basic-setup-38a563cd594f

ROS2 Setup: https://robofoundry.medium.com/oak-d-lite-camera-ros2-setup-1e74ed03350d

https://docs.luxonis.com/software/ros/depthai-ros/

https://docs.luxonis.com/software/ros/depthai-ros/driver/

https://github.com/luxonis/depthai-ros

Alternative ROS2 Publisher: https://github.com/jmguerreroh/oak_d_camera  (works, but I had to edit out all IMU stuff in the C++ code)

https://github.com/ARLunan/depthai_desktop

https://github.com/ARLunan/depthai_robot

----------------

**Back to** [Docs Folder](https://github.com/slgrobotics/robots_bringup/tree/main/Docs)
