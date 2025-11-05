## Notes and Current project status

This is a list of my status reports and notes related to recent observations, problems and fixes. Updated often.

### 2025-11-04

#### Status:
- Create 1 "Turtle" has Raspberry Pi 4 now, overclocked to 2 GHz. [Docs](https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Create1) updated.
- *articubot_one* code is cleaned, _dev_ merged to _main_ branch, both tested - all works. Use [_dev_](https://github.com/slgrobotics/articubot_one/tree/dev) as some small changes could be useful.
- [Seggy](https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Seggy) is a new reference platform (a.k.a. "guinea pig") - Plucky is retired (and gratiously donated some of his parts to Seggy).

#### Problems and fixes
- after the October ROS2 update the "diff_cont/odom" topic appears with *child_frame* and *frame_id* empty. Hopefully, EKF2 consumes that topic and published properly formatted "odometry/local".
- enabling compass (yaw component in IMU) in Nav2 config may or may not benefit navigation. Very tricky.

#### Plans
- Seggy needs to learn to squeeze through doors - needs further fine-tuning
- Behavior Trees seem like a nice area to explore
- [Gesture sensor](https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/FaceGesture.md) integration for robot control seems like a logical step

----------------

**Back to** [Main page](https://github.com/slgrobotics/robots_bringup)
