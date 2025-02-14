## BatteryStateBroadcaster setup

There are two ways of publishing _BatteryState_ message:
1. Have a node (ROS or, usually, MicroROS) communicate to robot hardware and publish [_BatteryState_ message](https://docs.ros.org/en/jazzy/p/sensor_msgs/interfaces/msg/BatteryState.html) directly

or,
   
2. Use _ros2_control_ infrastructure to pass necessary data through State interfaces and let specialized _BatteryStateBroadcaster_ component publish it.

Here we are interested in using the second option, as it best conforms to a preferred ROS2 pattern.

For traditional (#1) approach, see https://automaticaddison.com/how-to-create-a-battery-state-publisher-in-ros-2/

**Note:** when in Gazebo Sim, _BatteryState_ message originates from [Gazebo System](https://gazebosim.org/api/sim/8/battery.html) and is published by *gz_bridge*, avoiding *ros2_control* framework overall.
So, we are talking only about real hardware here.

### Installation (standard ROS2 Jazzy binary)

For ROS2 Jazzy _BatteryStateBroadcaster_ is part of the standard binary distribution, and can be installed by _apt_:
```
sudo apt install ros-${ROS_DISTRO}-battery-state-broadcaster
```
The source code is [here](https://github.com/ipa320/ros_battery_monitoring)
and configuring it is described [here](https://docs.ros.org/en/jazzy/p/battery_state_broadcaster/)

There is one extra package which is installed this way - *battery_state_rviz_overlay*, enabling battery voltage to show on RViz2 display.

There are limitations of the standard _BatteryStateBroadcaster_, and there are many GitHub repositories with more advanced or specialized code.
The standard Broadcaster only publishes battery _voltage_ and cannot populate other values of _BatteryState_ message. 
So, we will be missing *SOC (State of Charge), Percentage, Capacity, Health, Status* - something that our hardware can sometimes report.

### My version of BatteryStateBroadcaster

It was a relatively trivial task to find a more advanced version on GitHub and copy some code from it to a fork of *ipa320/ros_battery_monitoring* repository.

The donor code is here: https://github.com/HarvestX/h6x_ros2_controllers/tree/humble/battery_state_broadcaster - and all credits for this work go to _HarvestX_ team.

The modified code is, naturally, here: https://github.com/slgrobotics/ros_battery_monitoring

**Note:**
- My code still doesn't populate *cell_voltage[]* and *cell_temperature[]* arrays of individual cell values for each cell in the pack.

### Installation (with my version of BatteryStateBroadcaster)

The repository must be cloned to Raspberry Pi machine, residing next to other packages:
```
ros@plucky:~$ ls ~/robot_ws/src/
articubot_one  diffdrive_arduino  ldlidar_sl_ros2  mpu9250  *ros_battery_monitoring*  serial
```
Here are the steps:
```
cd ~/robot_ws/src
git clone https://github.com/slgrobotics/ros_battery_monitoring.git
cd ~/robot_ws
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -r -y
colcon build
```
After this installation, the package compiled from sources will override the standard binary installation, after ```install/setup.bash``` is sourced.

Here are the files that configure and work with BatteryStateBroadcaster:
- https://github.com/slgrobotics/diffdrive_arduino/blob/main/src/diffdrive_arduino.cpp : line 74
- https://github.com/slgrobotics/diffdrive_arduino/blob/main/src/battery.cpp

**Note:** using the "_dev_" branch:

- https://github.com/slgrobotics/articubot_one/blob/dev/description/battery.xacro
- https://github.com/slgrobotics/articubot_one/blob/dev/robots/plucky/config/controllers.yaml
- https://github.com/slgrobotics/articubot_one/blob/dev/robots/plucky/description/robot.urdf.xacro : line 41
- https://github.com/slgrobotics/articubot_one/blob/dev/robots/plucky/launch/plucky.launch.py : line 113, 156, 257

### Displaying battery state data in RViz2

There are two ROS2 packages that allow battery information to be displayed in an RViz2 "overlay":
- *battery_state_rviz_overlay* (installed with *ros_battery_monitoring* above)
- *rviz-2d-overlay-plugins* - to be installed separately:
```
sudo apt install ros-${ROS_DISTRO}-rviz-2d-overlay-plugins
```
The second [package](https://github.com/teamspatzenhirn/rviz_2d_overlay_plugins)
subscribes to [OverlayText.msg](https://github.com/teamspatzenhirn/rviz_2d_overlay_plugins/blob/main/rviz_2d_overlay_msgs/msg/OverlayText.msg) - which
is published by the first package, subscribing, naturally, to our [BatteryState.msg](https://docs.ros.org/en/jazzy/p/sensor_msgs/interfaces/msg/BatteryState.html)

You have to launch and configure the node as follows (note remappings in my case):
```
rviz_overlay = Node(
    package='battery_state_rviz_overlay',
    executable='battery_state_rviz_overlay',
    #namespace=namespace,
    #parameters=[{'use_sim_time': use_sim_time}],
    output='screen',
    remappings=[('battery_state','battery/battery_state')]
)
```
Then you can just run RViz2 and "Add" a panel "By Topic" (*/battery_display_text->TextOverlay*) or "By Type" (*rviz_2d_overlay_plugins->TextOverlay*)

The overlay should appear on top-left side of the display by default.

**Note:** Text font, size and other parameters are configurable if you install my version.

Here is how parameters look in a launch file:

https://github.com/slgrobotics/articubot_one/blob/dev/launch/launch_rviz.launch.py
```
    rviz_overlay = Node(
        package='battery_state_rviz_overlay',
        executable='battery_state_rviz_overlay',
        namespace=namespace,
        parameters=[{
            'use_sim_time': use_sim_time,
            # see https://github.com/slgrobotics/ros_battery_monitoring/blob/main/battery_state_rviz_overlay/src/battery_state_rviz_overlay_parameters.yaml
            'width' : 450,
            'height' : 10,
            'line_height' : 25,
            'horizontal_distance' : 20,
            'vertical_distance' : 20,
            'font' : 'DejaVu Sans Mono',
            'text_size' : 15.0,
            'horizontal_alignment' : 0,
            'vertical_alignment' : 3,
            'bg_color_a' : 0.1,
            'fg_color_rgba' : '0.0 0.8 0.6 1.0'
        }],
        output='screen',
        remappings=[('battery_state','battery/battery_state')]
    )
```
### Pie Chart Display

The *rviz-2d-overlay-plugins* package (installed above for battery display) contains a "_Circular Gauge Overlay_" (a.k.a. _Pie Chart_)
[plugin](https://github.com/teamspatzenhirn/rviz_2d_overlay_plugins/tree/main/rviz_2d_overlay_plugins#circular-gauge-overlay).
It accepts standard Float32 message, which you can publish, for example, from the command line:
```
ros2 topic pub /piechart std_msgs/Float32 "data: 0.6"
```
You need to add it in RViz2 ("_By Display Type_" or "_By Topic_") and can configure it in the _Displays_ panel (_max_ and _min_ values are especially important).

In real life you can either publish it from your code, or, more conveniently, produce it
by running a [*relay_field*](https://github.com/ros-tooling/topic_tools/blob/jazzy/README.md#relayfield) node:
```
    battery_pie_chart_relay = Node(
        package='topic_tools',
        executable='relay_field',
        name='battery_pie_chart_relay',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        arguments=['battery/battery_state', 'battery/percentage', 'std_msgs/Float32', '{ data: m.percentage }', '--qos-reliability', 'reliable' ]
    )
```

### Credits

Original code: Jonas Otto and "official" ROS2 BatteryStateBroadcaster - [ros_battery_monitoring](https://github.com/ipa320/ros_battery_monitoring)

Ideas and donor code for parameters handling: _HarvestX_ team - [battery_state_broadcaster](https://github.com/HarvestX/h6x_ros2_controllers/tree/humble/battery_state_broadcaster)

### Useful links

https://www.jackery.com/blogs/knowledge/ultimate-guide-to-lifepo4-voltage-chart

https://docs.ros.org/en/jazzy/p/sensor_msgs/interfaces/msg/BatteryState.html

https://control.ros.org/jazzy/doc/ros2_control/doc/index.html

https://docs.ros.org/en/jazzy/p/battery_state_broadcaster

https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_sim_demos/README.md#battery

https://github.com/ipa320/ros_battery_monitoring/issues/2

https://github.com/PickNikRobotics/generate_parameter_library

### Tip: use the following commands for troubleshooting

https://control.ros.org/master/doc/ros2_control/ros2controlcli/doc/userdoc.html
```
ros2 control list_controllers
ros2 control list_controller_types
ros2 control list_hardware_components
ros2 control list_hardware_interfaces
```
