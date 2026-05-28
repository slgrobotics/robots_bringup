**Back to** [Wiki](https://github.com/slgrobotics/articubot_one/wiki)

## MockBots Initiative

The MockBots project was initiated by members of the HomeBrew Robotics [club](https://www.hbrobotics.org/)
to create instructions for building a better, modern version of the TurtleBot.
These *MockBots* can be built using various vacuum cleaner bases, such as iRobot Create, Roomba, or Neato, and can integrate different sensors.

The builders' experiences are detailed in a book available on [Amazon](https://www.amazon.com/MockBOT-Over-shoulder-instructions-personal/dp/B0GK1TS39N).

## MockBots and the *articubot_one* Codebase

While the book introduces ROS2 and walks you through the initial robot setup, builders must still do considerable extra work to make their code manageable.
This is where using an established codebase with an organized collection of *config*, *launch* and *description* files becomes valuable.

The *articubot_one* repository already features *Turtle* — a robot layout very similar to the MockBots design.

*Turtle* features include:
- An iRobot *Create 1* base driven by the ROS 2 *Autonomous Labs* driver
- Customizable *description* files
- A comprehensive collection of *configuration* files (URDF, Nav2 etc.)
- Structured *launch* files easily adaptable to different sensors and localizers (AMCL, SLAM Toolbox etcd.)
- Support for both Gazebo simulation and physical hardware deployment
- Support for ROS2 *Jazzy* and *Kilted* releases, and *Lyrical* as soon as its Nav2 and other packages are released.

## Why bother?

While running "raw" *TurtleBot 4* code on your hardware can be exciting initially, you may quickly find yourself overwhelmed by editing files directly in the `/opt` directory and losing track of your changes.

Furthermore, the standard *TurtleBot 4* code functions primarily as a sample and does not fully align with modern ROS 2 patterns and practices.
Transitioning to `ros2_control` provides a much more robust foundation for your project.

Having a fully functional simulation of your robot in Gazebo simplifies debugging and saves development time.

Turtle and especially Seggy feature custom Behavior Trees — a powerful tool you may want to explore further.

## How to start

First, clone the repository (the `dev` branch is preferred) and follow the setup [instructions](https://github.com/slgrobotics/robots_bringup/blob/main/Docs/ROS-Jazzy/README.md#build-articubot_one-robot-codebase).

Verify that you can run the *Turtle* in the Gazebo simulation, drive it using a [joystick](https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/Joystick.md),
and navigate through the *Test* or *Warehouse* worlds.

Next, explore the *Turtle*'s `config`, `launch`, and `description` files to understand their structure and how they relate to the root configuration files.

Once you are ready to customize the project, fork the repository and implement your changes:
* Copy the `/robots/turtle` [directory](https://github.com/slgrobotics/articubot_one/tree/dev/robots/turtle) and rename it (for example, `/robots/my_mockbot_one`).
* Modify the `config`, `launch`, and `description` files in your new directory to match your MockBot's specific hardware (*Tip:* look for sensors used in Seggy and other robots).
* Launch your custom robot in Gazebo to ensure it can be driven by a joystick and can navigate the Test world.
* Adapt the launch files to your physical hardware (sensors, etc.) and execute the code on your robot's Raspberry Pi.

**Tip:** consult this [guide](https://github.com/slgrobotics/robots_bringup/tree/main/Docs/MakeYourOwn) for more info.

When you are satisfied with your modifications, submit a Pull Request to the *articubot_one* repository. It will be merged following a friendly review.

TBD

----------------

**Back to** [Wiki](https://github.com/slgrobotics/articubot_one/wiki) or [Docs Folder](https://github.com/slgrobotics/robots_bringup/tree/main/Docs)
