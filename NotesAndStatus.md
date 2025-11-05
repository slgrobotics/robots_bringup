## Notes and Current project status

This document lists my recent status reports, observations, problems, and fixes. (Updated frequently.)

### 2025-11-04

#### Status:
- Create 1 "_Turtle_" now runs on a Raspberry Pi 4, overclocked to 2 GHz. [Documentation](https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Create1) has been updated..
- *articubot_one* code has been cleaned, the _dev_ branch was merged into _main_, and both were tested - everything works.
Use [_dev_](https://github.com/slgrobotics/articubot_one/tree/dev), as it usually includes a few potentially useful tweaks.
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
