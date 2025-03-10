## OAK-D Lite setup

I have a "Kickstarter Edition" OAK-D Lite stereo camera. 

This guide describes how to run it under Ubuntu 24.04 and ROS2 Jazzy - first on an Intel 5 Desktop, an then on Raspberry 4 or 5 under the Ubuntu 24.04 Server and ROS2 Jazzy Base.

## Basic hardware test

The camera requires USB3 cable and socket. 

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
Look for "Intel Movidius MyriadX"
```
Do not expect _/dev/video*_ descriptor appear, OAK-D uses different mechanism for accessing hardware.

Prepare and apply _udev_ rules:
```
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## Luxonis Software Installation

There is a collection of python examples, which is not needed for ROS2, but is useful for testing.
```
mkdir ~/depthai
cd ~/depthai
git clone https://github.com/luxonis/depthai-python.git
cd ~/depthai/depthai-python/examples
sudo apt install python3.12-venv
python3 -m venv /home/ros/depthai/depthai-python/.env
source /home/ros/depthai/depthai-python/.env/bin/activate
python3 install_requirements.py
```
Here is how to see video preview in a window: 
```
cd ~/depthai/depthai-python/examples
python3 ColorCamera/rgb_preview.py
```

## ROS2 operation

ROS2 Jazzy has binary distribution:
```
sudo apt install ros-${ROS_DISTRO}-depthai-ros

ros2 launch depthai_ros_driver camera.launch.py
```
By default, the */oak/rgb/image_raw* has 1280x720 size, and can be consumed compressed. It is published at VERY low frame rate, below 1 Hz.

## Useful Links

https://robofoundry.medium.com/oak-d-lite-camera-ros2-setup-1e74ed03350d

https://docs.luxonis.com/software/ros/depthai-ros/

https://docs.luxonis.com/software/ros/depthai-ros/driver/

https://www.sundance.com/oak-d-lite/

https://github.com/luxonis/depthai-ros
