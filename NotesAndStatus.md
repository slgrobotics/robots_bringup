## Notes and Current project status

This document lists my recent status reports, observations, problems, and fixes. (Updated frequently.)

### 2025-11-09

#### Status:

- installed ROS2 [Kilted](https://docs.ros.org/en/kilted/Installation/Ubuntu-Install-Debs.html) Desktop on a spare machine (I5, Ubuntu 24.04) alongside Jazzy.
- installed Kilted Base on Turtle (RPi4 overclocked to 2GHz, Ubuntu 24.04) also alongside Jazzy
- Updated `~/bashrc` to allow switching between distributions:
```
source /opt/ros/kilted/setup.bash
#source /opt/ros/jazzy/setup.bash
```

#### Problems and fixes

On the Desktop side:
- Updated *ros_battery_monitoring* to make Jazzy code also Kilted-compatible - details [here](https://github.com/slgrobotics/ros_battery_monitoring/commit/635043eec933e5e9169ab7d871b9397979a27e05)
- Verified that:
  - `ros2 launch articubot_one seggy_sim.launch.py` runs successfully (same for *plucky, dragger, turtle*)
  - `ros2 launch articubot_one launch_rviz.launch.py use_sim_time:=false` works as expected

On-board RPi side (Turtle):
- Received warnings when running: `rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -r -y` (related to *xv_11_driver, create_driver*)
- One parameter in *nav2_params.yaml* was renamed:
```
    error_code_names: # works for Jazzy, must be removed for Kilted
    #error_code_name_prefixes: # works for Kilted
```
- rosdep installed a number of likely unnecessary packages.
- *Turtle* works, navigates, and appears stable and happy.

Interoperability:
- Turtle running under Kilted successfully appeared in RViz2 on a Jazzy desktop and was controllable via its joystick.

#### Plans
- Resolve the remaining build warnings.
- Continue using Jazzy as the main environment, while keeping Kilted installed for compatibility testing and future-proofing.

--------------------

### 2025-11-04

#### Status:
- Create 1 "_Turtle_" now runs on a Raspberry Pi 4, overclocked to 2 GHz. [Documentation](https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Create1) has been updated..
- *articubot_one* code has been cleaned, the _dev_ branch was merged into _main_, and both were tested - everything works.
Use [_dev_](https://github.com/slgrobotics/articubot_one/tree/dev), as it usually includes a few potentially useful tweaks.
- Enabling *composition* and *use_intra_process_comms* in Nav2 launch file seems to work fine.
- [Seggy](https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Seggy) is a new reference platform (a.k.a. "guinea pig"). Plucky has been retired (and gratiously donated some of his parts to Seggy).
- Seggy's BLDC *[Motor wheels](https://github.com/slgrobotics/robots_bringup/tree/main/Docs/MakeYourOwn#bill-of-materials-bom)* are awesome; even with only 60 ticks per revolution from the Hall sensors, control is stable.
- Both of my [52Pi Power Expansion Boards](https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Ubuntu-RPi/README-Extra.md#properly-feeding-your-raspberry-pi-5) are working well (one on Seggy and one on a spare RPi 5).
- Ubuntu [real-time](https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Ubuntu-RPi/UbuntuRealTime.md) is running reliably — now deployed on Turtle’s RPi 4.

#### Problems and fixes
- after the October ROS2 update, the "*diff_cont/odom*" topic appears with empty *child_frame* and *frame_id* fields. Thankfully, EKF2 correctly consumes this topic and republishes a properly formatted */odometry/local*, which is now used system-wide.
- Enabling the compass (*yaw*) component in the IMU configuration for *Nav2* may or may not improve navigation. It’s tricky on Seggy — the MPU9250 needs some tuning — but the BNO055 works fine on Turtle.

#### Plans
- Teach Seggy to squeeze through doorways — requires additional tuning.
- Give Dragger some outdoor exercise; it hasn’t been tested in a while.
- Behavior Trees seem like a nice area to explore.
- Integrate the [Gesture sensor](https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/FaceGesture.md) for robot control — a logical next step.
- Put OAK-D Lite on Seggy to work; its point cloud should be tested with Nav2.

----------------

**Back to** [Main page](https://github.com/slgrobotics/robots_bringup)

#### Status:
#### Problems and fixes
#### Plans

