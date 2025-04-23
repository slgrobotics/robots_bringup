## Make Your Own Robot

This section describes hardware components you can use to build a robot "_from scratch_" -
similar to my [Plucky](https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Plucky)
and [Dragger](https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Dragger).

The [Turtle](https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Create1) (based on the iRobot Create 1) is a bit different, and can be replicated almost exactly if you have an iRobot Create 1 or 2 base.

Building a robot "from scratch" (including wheels, motors, and electronics) might be overwhelming. Instead, consider a Neato, Create 1/2/3 or any other proven base with available ROS2 driver. 

Once you've assembled all recommended hardware, [my software](https://github.com/slgrobotics/articubot_one) should work with only minimal changes. 
It will run on almost any hardware, once you figured out how to move the wheels with *cmd_vel* topic and publish wheel encoder ticks through a joint broadcaster.

## General Considerations

In the ROS2 ecosystem, **building robots is about combining compatible components** - both in software and hardware. 
The availability of software components (ROS2 _packages_, _Nodes_, _launch_ and _config_ files)  is crucial. 
Even the best sensor or motor controller is useless without a ROS2 driver - turning it into an expensive paperweight.

ROS2 robots are, in general, quite complex. 
It is important to **start with a complete example project** where all configuration and tuning are already done. 
You can then modify and expand it without breaking the structure. 
Starting with a simplistic “tutorial-quality” sample offers no easy path forward.
Refer to [this guide](https://github.com/slgrobotics/robots_bringup?tab=readme-ov-file#how-to-use-this-repository) 
for a solid starting-point robot code.

In my experience, simulation is key.
It is very important to **have your robot work in simulated environment**, with simulated hardware.
If your robot works in simulation using Gazebo, it will likely work in the real world.
Simulation lets you learn the system before spending any money. 
Start by “building” your robot virtually using a URDF/.xacro robot description, then transition to physical hardware.

When working with robots you need a _Workstation_ - typically a gaming-class desktop PC running Ubuntu 24.04 [Desktop](https://github.com/slgrobotics/robots_bringup/tree/main/Docs/ROS-Jazzy).
You will need a [joystick](https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/Joystick.md) connected to it for control.
Confirm your setup by running Plucky, Dragger or Turtle in Gazebo [simulation](https://github.com/slgrobotics/robots_bringup/tree/main/Docs/ROS-Jazzy#bringing-up-robot-simulation-in-gazebo).

Your physical robot will likely have a Raspberry 5 SBC, preferrably with 8 Gb RAM. Install the OS using [this guide](https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Ubuntu-RPi).

Reliable WiFi is essential - test it using [this guide](https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/WiFi_Logger_Visualizer.md#wifi-testing-and-benchmarking).

Note that _VS Code_ IDE works very well with ROS2 code, and _GitHub_ is an excellent tool for collaboration and version control.

**Warning:** ROS, ROS2 and Gazebo have a long [history](https://en.wikipedia.org/wiki/Robot_Operating_System) and many breaking changes.
As a result of this, most of your searches for information, documentation and code will lead to obsolete,
wrong pieces of old documentation and code, and suggested solutions won't work with current release (ROS2 Jazzy).
The ability of AI tools to hallucinate and lie about ROS is legendary.
Support is in general non-existent and some packages are not production quality at all.
Your best friend is a working code example which you've tested yourself and your ability to find _current_ documentation.

## Building Robot in "Layers"

In ROS2 you can build your physical robot step-by-step, adding hardware and software in layers.

**First**, you need to build a "base" (Chassis) - for example, a wheeled cart with motors and a motor controller.  
In ROS2 software it will be defined as *base_link* and other elements in *robot_description* (URDF, xacro) files.
It will be driven by a motor driver node.
iRobot Create 1 is an example of such a "base".
A _Differential drive_ base is easier to configure in ROS2 than an _Ackermann steering_ type.
- **Goal:** Use a joystick to move your base. Describe your base in an _.xacro_ file.

**Second**, you need a collection of [sensors](https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Sensors).
You’ll need at least an IMU and LIDAR - and for outdoors you need a GPS receiver. Skip cameras for now - they add complexity.
In ROS2, each sensor is a node that publishes data other nodes can subscribe to.
- **Goal:** Learn sensor launch files. Use _rqt_ and *rqt_graph* to visualize topics and confirm your sensors are correctly publishing data. Make sure the *frame_id* part corresponds to your _.xacro_ definition of the sensor.

**Third**, you need to configure and launch Localization (SLAM Toolbox) and Navigation (Nav2) packages.
This makes your robot autonomous.
- **Goal:** Learn how to modify config and launch files—this is core to ROS2 development. Make sure your robot works in Gazebo simulation too.

**Finally**, once basic systems are working, program behaviors using Behavior Trees or Python scripts.
You can also experiment with cameras, sonars, point clouds, and AI-based object detection.

## Bill of Materials (BoM)

It is totally up to you how large and complex your robot will be.
Here we would think of a Roomba-size to a wheelchair sized robot.
This list assumes you're building it from scratch (not using Create/Neato bases).
Basic soldering and Arduino-like microcontroller experience is assumed.

Roughly, here is a BoM for it:
- Base
  - Wheels
  - Gearboxes
  - Motors with Quadrature Encoders
  - H-Bridges - (e.g. [IBT-2/BTS7960](https://www.amazon.com/BTS7960-H-bridge-Double-Current-Diagnostic/dp/B09W8VV6RH) - one per motor)
  - Arduino Mega or similar _base controller_ with [software](https://github.com/slgrobotics/Misc/tree/master/Arduino/Sketchbook/DraggerROS)

- Power
  - 12V LiFePO4 battery (20–50 Ah)
  - DC-DC buck converter (for 5V output)
  - Switches, diodes, capacitors (for circuit protection) 

- [Sensors](https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Sensors) - choose what's available:
  - IMU - [MPU9250](https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/MPU9250.md) or [BNO055](https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/BNO055%20IMU.md)
  - [LIDAR](https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/LD14.md) - LD14 for indoors, LD19P for outdoors

- Raspberry Pi 5 (8GB) running Ubuntu 24.04 Server, ROS2 Jazzy Base and [robot software](https://github.com/slgrobotics/articubot_one) - see [this guide](https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Ubuntu-RPi)

**Option:** Use a _RoboClaw_ controller instead of Arduino + H-Bridges along with the appropriate [ROS2 driver](https://github.com/wimblerobotics/ros2_roboclaw_driver)

**Notes & Safety Tips:**
- Larger robots can be dangerous when out of control. An independent _[remote kill switch](https://www.amazon.com/DieseRC-Universal-Wireless-Receiver-Transmitter/dp/B098WGK35L)_ is recommended.
- Indoors, robot width should accommodate standard door frames, with padding for safety.
- Round robots maneuver better in tight spaces. Add side rollers and soft padding to protect walls.
- Two casters are more stable than one; aim for 2/3 weight on the drive wheels.
- Consider portability if your robot is large - can it fit in your car?
- _Voltage spikes:_ Brushed motors can generate damaging voltages when stopping or being manually moved. Lead-acid batteries absorb these, but Lithium batteries with BMS may disconnect, exposing electronics to spikes.
Depending on your robot design, a [Shunt Regulator](https://www.pololu.com/category/249/shunt-regulators) or a large electrolytic capacitor (4700+ uF 50 V) is needed.
Some motor controllers include spike protection—check your model.

## Base Control - the Arduino way

There are two components in my robots that require additional explanation: the Arduino-based wheel controller and its corresponding ROS 2 driver.
This architecture is inspired by Articulated Robotics by Josh Newans, a mechatronics engineer from Newcastle, Australia.
You can find his work [here](https://articulatedrobotics.xyz/category/getting-ready-to-build-a-ros-robot).

ROS2 nodes control wheels rotation by publishing to the */diff_cont/cmd_vel* topic.
In return, wheel positions must be reported by the driver via the */joint_states* topic.

The ROS2 [driver](https://github.com/slgrobotics/diffdrive_arduino) operates under the Control Manager, and consists of _DiffDriveController_ and _JointStateBroadcaster_.

To control the wheels the ROS2 driver opens a serial connection to an Arduino Mega 2560 microcontroller running custom firmware.

The Arduino [firmware](https://github.com/slgrobotics/Misc/tree/master/Arduino/Sketchbook/DraggerROS) is responsible for:
- Generating PWM signals for H-Bridges (DraggerMotors.ino)
- Reading quadrature encoders (DraggerEncoders.ino)
- Calculating PID control values (DraggerCalculate.ino)
- Communicating with the ROS2 driver via serial (DraggerComm.ino)
- Integrating a "*local override*" joystick (DraggerJoystick.ino)
- Integrating a "[Parking Sensor](https://photos.app.goo.gl/WsqkA4XpYSLrVDX59)" (sonars) - code available [here](https://github.com/slgrobotics/Misc/tree/master/Arduino/Sketchbook/ParkingSensorI2C)

While you may need to adapt the Arduino software to match your specific robot base, the ROS 2 driver can be used as-is.

----------------

**Back to** [Docs Folder](https://github.com/slgrobotics/robots_bringup/tree/main/Docs)
