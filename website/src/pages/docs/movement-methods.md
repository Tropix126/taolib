---
title: Getting Started
layout: ~/layouts/DocsLayout.astro
---

# Movement Methods
taolib's `Drivetrain` class provides several methods for controlling the movement of your robot's drivetrain. In this tutorial, we'll take a look at the different methods available, and provide examples of how to use them in your autonomous routines.

## Driving Straight

The `Drivetrain::drive()` method is used to move the drivetrain directly forwards or backwards by a given distance. The first parameter of this method is the distance that you wish to travel. A negative number denotes that the drivetrain should drive backwards.

![Driving straight demo](https://i.imgur.com/htLZvB4.png)

```cpp
chassis.drive(10); // Drive 10 inches forward
chassis.drive(-10); // Drive 10 inches backwards
```

You can also specify that the function is *non-blocking* by passing in `false` as the second parameter, meaning that you will be able to execute commands while the drivetrain is moving.

```cpp
chassis.drive(20, false); // Drive 20 inches forward. Do not block the main thread.
intake.spin(vex::forward, 100, vex::percent); // Start spinning an "intake" motor forwards as we drive.
waitUntil(chassis.is_settled()); // Wait until the movement is settled before doing more things.
```

## Turning to an angle


The `Drivetrain::turn_to()` method is used to rotate the robot to a specific heading in place. The first parameter of this method is the absolute angle in degrees to rotate the drivetrain to.

![Turning to angle demo](https://i.imgur.com/y2DoMeq.png)

```cpp
chassis.turn_to(0); // Rotate to 0 degrees
chassis.turn_to(-60); // Rotate to -60 degrees
```

As with the drive method, you can also specify that the function is non-blocking by passing in false as the second parameter.

```cpp
chassis.turn_to(180, false); // Rotate to 180 degrees. Do not block the main thread.
intake.spin(vex::forward, 100, vex::percent); // Start spinning an "intake" motor forwards as we turn.
waitUntil(chassis.is_settled()); // Wait until the movement is settled before doing more things.
```

## Turning to a point

The `Drivetrain::turn_to()` method can also be used to rotate the robot to face towards a specific cartesian coordinate. Rather than passing in an angle, pass in a `tao::Vector2` instance.

![Turning to point demo](https://i.imgur.com/SOKilKl.png)

```cpp
tao::Vector2 point(24, 24);
chassis.turn_to(point); // Rotate to face towards the point (24,24)
```

This is especially useful for cases where precision aiming is important, as the drivetrain will always face the absolute point regardless of where it is positioned on the field.

## Moving to a point

The `Drivetrain::move_to()` method is used to move the drivetrain to a set of target coordinates. The first parameter is a 2D Vector representing the absolute target coordinates to move to.

![Moving to point demo](https://i.imgur.com/Qr1XP7D.png)

```cpp
chassis.move_to(tao::Vector2(24, 24)); // Move to the point (24,24)
```

## Moving along a path

Sometimes moving point-to-point doesn't cut it. Maybe you're losing too much time stopping in-between points. The `Drivetrain::move_path()` method is used to move the drivetrain along a set of waypoints using pure pursuit. This means that given a list of points, the drivetrain will smoothly follow a path without stopping in the middle to settle.

![Moving along path demo](https://i.imgur.com/jfHyD7L.png)

> Tuning lookahead distance (covered in [Drivetrain Setup]()) is an important optimization for ensuring your robot's drivetrain maintains a correct balance of speed and accuracy when following a path.

The first parameter is a C++ vector of 2D Vectors representing waypoints forming a path.

```cpp
std::vector<Vector2> path = {Vector2(0, 0), Vector2(10, 10), Vector2(20, 0)};
chassis.move_path(path);
```

> Unlike other movement functions, the move_path() method does not have a non-blocking option. However, you can perform other actions in parallel with this method by starting a new [thread](https://www.vexforum.com/t/how-to-use-vex-threads/100901). This limitation might be changed in future releases of taolib.
