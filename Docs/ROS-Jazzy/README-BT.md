## Behavior trees

Your typical nav2_params.yaml contains sections related to behavior trees, the most obvious are:
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
Here are some original docs:

- https://www.behaviortree.dev/docs/intro
- https://www.behaviortree.dev/docs/ros2_integration

### Using Groot2

Groot2 is a visualization tool for BTs.

First, install it - https://www.behaviortree.dev/groot/

You can choose either of the two Linux installation options, here is how to run *AppImage* after downloading:
```
cd ~/Downloads
chmod +x Groot2-v1.6.1-x86_64.AppImage 
./Groot2-v1.6.1-x86_64.AppImage 
```
Groot2 will come up and can be used now. Try opening the following file (or any other in that directory):

/opt/ros/jazzy/share/nav2_bt_navigator/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml

