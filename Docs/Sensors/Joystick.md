## Joystick setup

I am using _Microsoft Wireless Xbox 360 Controller_ as joystick, which is well supported in ROS2.
If you use other types - it is relatively easy to adjust related files, but you are on your own.
Finding a genuine one might save you some troubleshooting time.

These instructions assume you are using teleop **from your desktop machine** where the joystick is connected via USB.

### Test your joystick
```
ros2 run joy joy_enumerate_devices
ros2 run joy joy_node      # <-- Run in first terminal
ros2 topic echo /joy       # <-- Run in second terminal
```
https://index.ros.org/p/teleop_twist_joy/github-ros2-teleop_twist_joy/#jazzy

See "Useful links" below.

### Installation (with my articubot_one codebase)

**Note:** 
1. The left joystick controls robot's movements, "A" and "Y" buttos serve as "Enable" and "Turbo". Hold "Enable" button while operating left joystick.
2. In ROS2 Jazzy *cmd_vel* messages are now **TwistStamped** type, my fork has been modified to support it.
   I also added remappings to _/diff_cont/*_ to match Jazzy Controller architecture.
   Your regular _teleop_ might not work with it your robot _base_, use instructions below.
3. There's a **twist_mux** node, which is run by each robot _*.launch.py_ file. It combines various *cmd_vel* inputs and delivers filtered message to robot's base. Pay close attention to names of these topics.

![twist_mux](https://github.com/user-attachments/assets/7007fc56-699a-475a-b5d8-a371beb6be31)

To use joystick, you first need to build my *articubot_one* fork, which supports **TwistStamped** type:
```
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src
git clone https://github.com/slgrobotics/articubot_one.git
git clone https://github.com/slgrobotics/ros_battery_monitoring.git
cd ~/robot_ws

sudo rosdep init    # do it once, if you haven't done it before
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -r -y

colcon build
```
Take a look at ```~/robot_ws/src/articubot_one/launch/joystick.launch.py``` - if you don't run *twist_mux*, you may want to tweak *cmd_vel_joy* there for experiments (for example, make it "*cmd_vel*").
Do ```cd ~/robot_ws; colcon build``` after all changes.

Now you can run the driver, or let your robot's desktop-side launch file do it at startup. 

For example, you should be able to use teleop with Create 1 base alone (not running the *turtle.launch.py*)
```
ros2 launch articubot_one joystick.launch.py &
```

**Note:**
1. *joy_node* sends *cmd_vel* messages ONLY when enable_button is pressed.
2. You MUST set *enable_button* parameter to desired value.
3. Try ```ros2 param get /teleop_twist_joy_node enable_button``` to see current value
4. See *~/robot_ws/src/articubot_one/config/joystick.yaml*

### Useful links

Joystick teleop blog:

https://articulatedrobotics.xyz/mobile-robot-14a-teleop/

