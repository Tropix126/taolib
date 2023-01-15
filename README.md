<h1 align="center">taolib</h1>

<p align="center">
  <a href="#overview">Overview</a> |
  <a href="#development">Development</a> |
  <a href="#contributors">Contributors</a>
</p>

<p align="center">  <br/>
A library for complex autonomous robot movement in the VEX Robotics Competition (VRC).
  <br/>
</p>

---

# Overview

taolib is a library that combines many motion control algorithms such as [PID feedback control](https://renegaderobotics.org/pid-beginners-guide/), [Odometry](http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf), and [Pure Pursuit](https://colab.research.google.com/drive/1fS4KaYXb7F1YQjP9lu66p511gsIPZWFe?usp=sharing#scrollTo=mx9nirOrLlN3) into an easy-to-use library with as little setup as possible, allowing the creation of complex and accurate autonomous routines.

## Downloads

These will link you to the latest builds found in the [releases](https://github.com/Tropix126/taolib/releases/) tab of this repository.

| [VEXcode]()  | PROS (Coming (hopefully) soon!) |
| ------------- | ------------- |



## Codebase (headers)

```
.
└──include
    ├──taolib.h         // The entry point of the library, which includes all of the important files necessary for the library's usage.
    ├──drivetrain.h     // The base tao::Drivetrain class. This class abstracts various position tracking and motion control algorithms to provide various functions for controlling a physical drivetrain. Given an IMU, left motor, right motor, and a model (called a "profile") for tuning certain physical aspects unique to each robot, the tao::Drivetrain class can perform various actions such as drive the robot straight, turn the robot in place, arc the robot to a point, or make the robot follow a path.
    ├──math.h           // Contains the tao::math namespace which stores various helper functions and variables used in mathematical operations. This includes utilities for working with angles, intersection finders for pure pursuit, and basic number operations.
    ├──pid.h            // Contains the tao::PIDController class, which is a form of closed-loop feedback control for keeping a system at a stable desired target. Given a target and error (the difference between the current and desired number) as the input for the controller, the controller will output what is required to reach the target input. In practice, there are two PID controllers used in tao::Drivetrain for maintaining a target angular and linear velocity given a position as the input.
    ├──threading.h      // Contains various helpers for creating and working with multithreaded applications on vexcode. These helpers assist in various tasks that would normally be difficult to achieve with the vexcode provided vex::thread class, such as starting a thread from a non-static class member, or passing arguments to threads.
    └──vector2.h        // A class that represents a 2-dimensional vector stored as cartesian coordinates. Used internally to simplify the math used in odometry.
```

---

# Development

---

# Contributors

For information on contributing to this project, please see [CONTRIBUTING.md](/CONTRIBUTING.md).

<a href="https://github.com/tropix126/taolib/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=tropix126/taolib" />
</a>
