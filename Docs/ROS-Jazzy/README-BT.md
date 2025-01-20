## Behavior Trees in Nav2 package

Your typical *nav2_params.yaml* file contains sections related to behavior trees, the most obvious are:
```
# see https://docs.nav2.org/configuration/packages/configuring-bt-navigator.html
bt_navigator:
  ros__parameters:
    enable_stamped_cmd_vel: true
    global_frame: map
    robot_base_frame: base_link
  ...

# See https://docs.nav2.org/configuration/packages/configuring-behavior-server.html
behavior_server:
  ros__parameters:
    enable_stamped_cmd_vel: true  # default false in Jazzy or older
    local_costmap_topic: local_costmap/costmap_raw
    global_costmap_topic: global_costmap/costmap_raw
    local_footprint_topic: local_costmap/published_footprint
  ...
```
Here are some original docs to help understanding Nav2 architecture and operation:

- https://roscon.ros.org/2019/talks/roscon2019_navigation2_overview_final.pdf
- https://docs.nav2.org/configuration/packages/configuring-bt-navigator.html
- https://www.behaviortree.dev/docs/intro
- https://www.behaviortree.dev/docs/ros2_integration
- https://docs.nav2.org/behavior_trees/overview/detailed_behavior_tree_walkthrough.html
- https://automaticaddison.com/ros-2-navigation-tuning-guide-nav2/

Nav2 source code resides on GitHub: https://github.com/ros-navigation/navigation2

### Using Groot2

_Groot2_ is a visualization tool for Behavior Trees.

First, install it - https://www.behaviortree.dev/groot/

You can choose either of the two Linux installation options, here is how to run *AppImage* after downloading:
```
cd ~/Downloads
chmod +x Groot2-v1.6.1-x86_64.AppImage 
./Groot2-v1.6.1-x86_64.AppImage 
```
Groot2 GUI will come up and can be used now. Try opening the following file (or any other in that directory):
```
/opt/ros/jazzy/share/nav2_bt_navigator/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml
```
Refer to https://docs.nav2.org/tutorials/docs/using_groot.html for instructions

### Behavior Trees and RViz2

Navigator plugins expose _service ports_, which accept requests from other programs.
RViz2 can connect to two "*standard*" ports, and therefore interact with two "*standard*" plugins.

When you click on "*Nav2 Goal*" button or use "*Navigate through poses*" mode in RViz2 left panel, those service ports are called. 

Here is the *nav2_params.yaml* code which allows such interaction:
```
# Two navigators and their trees supporting different RViz2 mouse-click navigation modes: 
navigators: ["navigate_to_pose", "navigate_through_poses"]
# These two lines should be here for the param_substitutions in "*_nav.launch.py" to work:
default_nav_to_pose_bt_xml: "/opt/ros/jazzy/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml"
default_nav_through_poses_bt_xml: "/opt/ros/jazzy/share/nav2_bt_navigator/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml"
navigate_to_pose:
  plugin: "nav2_bt_navigator::NavigateToPoseNavigator"
navigate_through_poses:
  plugin: "nav2_bt_navigator::NavigateThroughPosesNavigator"
```
To substitute either of these trees, the following code in "*_nav.launch.py" can be used:
```
    param_substitutions = {
        # Two navigators and their trees supporting different RViz2 mouse-click navigation modes: 
        #'default_nav_through_poses_bt_xml': os.path.join(robot_path,"behavior_trees","navigate_through_poses_w_replanning_and_recovery.xml"),
        'default_nav_to_pose_bt_xml': os.path.join(robot_path,"behavior_trees","odometry_calibration.xml"),
        #'default_nav_to_pose_bt_xml': os.path.join(robot_path,"behavior_trees","navigate_to_pose_w_replanning_and_recovery.xml")
    }
  ...
```
In the above case, when you click on "*Nav2 Goal*" button and then anywhere on the map area, the "odometry_calibration.xml" will be activated.

Refer to https://github.com/slgrobotics/articubot_one for actual code. _Dev_ branch is usually more current, while _main_ is stable.

### Range sensors in costmap layer

LIDAR scans are not the only possible source of information for Nav2 costmaps. 
You can integrate sonars or any other range sensors into Nav2 using standard plugins.

https://docs.nav2.org/configuration/packages/costmap-plugins/range.html

https://automaticaddison.com/ros-2-navigation-tuning-guide-nav2/  (scroll down to or search for "*range_sensor_layer*")

https://github.com/ros-navigation/navigation2/blob/main/nav2_costmap_2d/plugins/range_sensor_layer.cpp

https://github.com/ros-navigation/navigation2/blob/main/nav2_costmap_2d/include/nav2_costmap_2d/range_sensor_layer.hpp

https://robotics.stackexchange.com/questions/66885/adding-range-sensor-layer-to-layered-costmap-for-global-planning

Either *_costmap_ section in *nav2_params.yaml* should look like this:
```
# see https://docs.nav2.org/configuration/packages/configuring-costmaps.html
local_costmap:
  local_costmap:
    ros__parameters:
      enable_stamped_cmd_vel: true
   ...
      plugins: ["static_layer", "obstacle_layer", "inflation_layer", "sonar_layer"]
      inflation_layer:
   ...
      obstacle_layer:
   ...
      static_layer:
   ...
      sonar_layer:
        plugin: "nav2_costmap_2d::RangeSensorLayer"
        enabled: True
        topics: ["/sonarLeft", "/sonarRight"]
        mark_threshold: 0.7
        inflate_cone: 0.99
      always_send_full_costmap: True
```
Refer to https://github.com/ros-navigation/docs.nav2.org/blob/master/configuration/packages/costmap-plugins/range.rst for configuration details.


