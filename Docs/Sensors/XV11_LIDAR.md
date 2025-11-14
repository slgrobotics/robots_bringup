## "Turtle" XV11 LIDAR setup

Surreal XV Lidar controller v1.2 (Neato Lidar) - connected via USB

https://github.com/getSurreal/XV_Lidar_Controller  - Teensy software

https://www.getsurreal.com/product/lidar-controller-v2-0/   - hardware (Teensy 2.0)

Connect to the USB port at 115200 baud. (test with ```minicom -D /dev/ttyACM0 -b 115200```)

See https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/PersistentUSB.md

### Positioning on the robot

https://photos.app.goo.gl/jvCmoPTPyVJtSjZW9

### ROS2 Driver

Original ROS2 driver port (by Mark Johnston): https://github.com/mjstn/xv_11_driver

In my fork I modified one file, ```.../xv_11_driver/src/xv_11_driver.cpp```, as _declare_parameter()_ now requires a default value as a second parameter.
I also redefined ```XV11_PORT_DEFAULT``` as ```/dev/ttyACM0```

My fork is here: https://github.com/slgrobotics/xv_11_driver

### Compile and install
```
sudo apt install libboost-dev

mkdir -p ~/robot_ws/src
cd ~/robot_ws/src
git clone https://github.com/slgrobotics/xv_11_driver.git

  (edit the xv_11_driver/src/xv_11_driver.cpp here - XV11_PORT_DEFAULT if your port is not /dev/ttyACM0)

cd ..
colcon build
```
### Test

Try running it on _turtle_, see _/scan_ messages in rqt on the Desktop:
```
source ~/robot_ws/install/setup.bash
ros2 run xv_11_driver xv_11_driver &
```

**Note:** Rviz **on your desktop machine** needs at least a static transform, to relate the grid to the laser frame ("_neato_laser_" in this case).

    rviz2 &
    ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map neato_laser &

For Rviz you need:

    Global Options to have Fixed Frame set to a known TF ("map")
  
    Grid reference frame - set to "map"
  
    Add LaserScan, topic "/scan", Style :Spheres" size 0.02
  
    ros2 run tf2_ros tf2_echo map neato_laser            -- to see published TF

----------------

**Back to** [Wiki](https://github.com/slgrobotics/articubot_one/wiki) or [Docs Folder](https://github.com/slgrobotics/robots_bringup/tree/main/Docs)
