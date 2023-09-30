---
title: Drivetrain Setup
category: Library
layout: ~/layouts/DocsLayout.astro
page: 2
---

# Drivetrain Setup

In this tutorial, we'll create and configure a `DifferentialDrivetrain`` object, tune the parameters of the drivetrain, and perform some basic movement.

# Setting Up Devices

First you'll need to provide your drivetrain's devices (motors, sensors, etc). taolib supports a few different device configurations:
* At minimum, 2 parallel motors controlling wheels of the same size.
  * You can have as many motors as you wish on each side of the drivetrain (4, 6, 8, etc...)
* You can optionally include a [VEX inertial sensor (IMU)](https://www.vexrobotics.com/276-4855.html) for more accurate heading telemetry and turns.
* You can optionally use two parallel [3-Wire Optical Shaft Encoders](https://kb.vex.com/hc/en-us/articles/360039512851-Using-the-V5-3-Wire-Optical-Shaft-Encoder) attached to [tracking wheels](https://gm0.org/en/latest/docs/common-mechanisms/dead-wheels.html) to prevent wheel slipping from messing up tracking.

> If you're on VEXcode Pro, you can add the required devices through the Robot Configuration button at the top right of the application window.

First, create two [motor groups](https://kb.vex.com/hc/en-us/articles/360058592211-Configuring-a-Motor-Group-in-VEXcode-V5) for your left and right side motors.

> It's possible that one of these motor groups will need to be reversed to properly control the drivetrain. taolib does not handle this for you, so it is important that you check if one side of your drivetrain has its motors flipped upside-down. If it does, then set the appropriate motor group side to run in reverse.

If you wish to use an inertial sensor or external encoders, add those as devices with the appropriate ports as well.

# Creating a DifferentialDrivetrain Object

After following the [getting started guide](/docs) and setting up taolib, you're ready to create a `DifferentialDrivetrain`` instace. This class is used for controlling all movement provided by taolib.

## Setup with 2 `vex::motor_group`s
```cpp
auto chassis = tao::DifferentialDrivetrain(left_drive, right_drive, IMU, config);
```

This device configuration will initialize the `tao::DifferentialDrivetrain` instance with two parallel motor groups and nothing else (the bare minimum configuration).

## Setup with an IMU
```cpp
auto chassis = tao::DifferentialDrivetrain(left_drive, right_drive, imu, config);
```

Similar to the previous configuration, we have two motor groups, but have additionally added in a V5 Inertial Sensor for accurate turns.
> If you use an intertial sensor, you'll need to calibrate it before moving the robot using `DifferentialDrivetraincalibrate_imu`. **This is an important step if you want to use a gyro for turning.** Example code:
> ```cpp
> auto chassis = tao::DifferentialDrivetrain(left_drive, right_drive, IMU, profile);
>
> void pre_auton() {
> 	chassis.calibrate_imu();
> }
> ...
> void autonomous() {
>	chassis.start_tracking();
> }
> ```

## Setup with tracking wheels

The following device configurations can be used if you wish to use two parallel tracking wheels on your drivetrain. These configurations accept two `vex::encoder` instances along with your motor groups and optionally an IMU:

### With IMU
```cpp
auto chassis = tao::DifferentialDrivetrain(left_drive, right_drive, left_encoder, right_encoder, IMU, profile);
```

### Without IMU
```cpp
auto chassis = tao::DifferentialDrivetrain(left_drive, right_drive, left_encoder, right_encoder, profile);
```

# Configuring the DifferentialDrivetrain

We aren't done yet. `DifferentialDrivetrain` objects need to be configured specifically for the physical aspects of a robot. Differences in size, weight, speed, and wheels typically means that no robot will behave exactly the same as another. This tuning process is extremely important, and an improperly tuned drivetrain will produce sub-optimal or unexpected movements.

You might recall that in the last section, we passed in a `config` parameter to `tao::DifferentialDrivetrain`. Lets fill that in with an example config for our drivetrain:

```cpp
auto chassis = tao::DifferentialDrivetrain(left_drive, right_drive, {
	.drive_gains = { 0, 0, 0 },
	.turn_gains = { 0, 0, 0 },
	.drive_tolerance = 0.0,
	.turn_tolerance = 0.0,
	.lookahead_distance = 0.0,
	.track_width = 0.0,
	.wheel_diameter = 0.0,
	.gearing = (1.0 / 1.0),
});
```

All these values are set to 0, though, and need to be tuned for your specific drivetrain. Let's go over each of them individually.

## Gearing

The `gearing` parameter of `tao::DifferentialDrivetrainProfile` describes the external gear ratio used by your drivetrain. The parameter accepts a fraction of `(DRIVE_TEETH / DRIVEN_TEETH)`. Let's say you have a drivetrain where your motors are attached to an 84 tooth gear that drives a 60 tooth gear attached to your wheels. Your gearing `(84.0 / 60.0)`.

> Note: It's important that decimals (e.g. `84.0` rather than `84`) are used when passing in a gear ratio fraction to avoid unexpected behavior when dividing integers.

If you have no gear ratio at all (your wheels are directly powered by motors), then leave this property as `1.0` to indicate you have a 1:1 direct drive.

## Wheel Diameter

This value describes the diameter of your drivetrain's driven wheels. To find this number, measure the distance from the center of the left wheels to the center of the right wheels. The wheels currently sold by VEX come in sizes of `2.75`, `3.25`, `4.0` inch diameters, however some of the older wheels (such as the discontinued versions of the 4-inch Omni-wheels) had a slightly larger diameter than advertised.

> IMPORTANT: The units you use for measurement here (and in `.track_width` below) will determine the units used for everything else (including odometry and movement functions).

## Track Width

This value describes the distance between the center of your left wheels to the center of your right wheels. Track width is used for finding the drivetrain's absolute heading using encoders if you don't have an IMU or if the IMU was unplugged.

![Track Width Visualization](https://kb.vex.com/hc/article_attachments/360085801912/track_width.jpg)

*You must measure this with the same units you used for your `wheel_diameter`*.

## Tolerances

The two tolerance values (`drive_tolerance`, `turn_tolerance`) are used to determine when the drivetrain has "settled" at its target location. These values represent the minimum acceptable *range of error* that is required for the drivetrain to be considered at its target. `drive_tolerance` is the minimum distance error for driving straight and `turn_tolerance` is the minimum degrees of rotational error for turning.

> Movement errors must be within |tolerance| for at least 50ms for the drivetrain to be considered settled.

Tolerances that are too low will result in the drivetrain either never fully settling or taking longer than expected to settle. Tolerances that are too high will result in blocking movement commands to unblock before the drivetrain has reached an acceptable target.

## PID Gains

PID Gains are a set of three numbers (called `kP`, `kI`, and `kD`) that are used to tune how fast and how smoothly the drivetrain will move to reach its target. If you aren't familiar with what a PID controller is, you can read an introduction to that [here](pid-controllers).

taolib utilizes two PID controllers to move the drivetrain. One outputs linear velocity (for driving straight) and the other outputs angular velocity (for turning). The gains of each PID controller must be tuned until both driving and turning actions come to a smooth and controlled stop in a reasonable amount of time.

To turn `turn_gains` you'll want to setup a basic autonomous routine that turns the robot a certain amount.

```cpp
void autonomous() {
	// chassis.calibrate_imu(); // Uncomment if you use an inertial sensor!
	chassis.setup_tracking();
	chassis.turn_to(0);
}
```

From there, the general process for tuning is as follows:
1. Increase `kP` (the first number AKA the proportional constant) until the robot reaches the target (for example, 180 degrees) in a reasonable amount of time and oscillates around it (bounces back/forth around the target). The oscillations should not be large enough to increase over time. Rather, you want to find a `kP` value that causes minor overshoot of the target and has oscillations that decrease and eventually reach the target.
2. From there, increase `kD` (the third number AKA the derivative constant) until the oscillations stop. A `kD` value that is too high will cause unpredictable movements, so increase it in small increments each time you test the movement.
3. If the robot *consistently undershoots the target every time*, you *might* want to increase `kI` (the second number AKA the integral constant) a very slight amount.
	> Keep in mind that undershoots are often caused by `kP` being too low, and not the need for an integral term. For almost all cases, `kI` should stay at or near 0, because the integral term is susceptable to [integral windup](https://en.wikipedia.org/wiki/Integral_windup), which can cause unpredictable movements.

A similar process will be used for tuning the drivetrain's `drive_gains`. We can run a test movement that drives forward 24 inches (1 VEX field tile) to tune linear movement:

```cpp
void autonomous() {
	// chassis.calibrate_imu(); // Uncomment if you use an inertial sensor!
	chassis.setup_tracking();
	chassis.drive(24);
}
```

From there, follow the same steps above to tune `drive_gains`.

> If you're having trouble figuring out how to tune PID gains or understanding this step, then the following additional resources might helpful:
> - https://georgegillard.com/documents/2-introduction-to-pid-controllers
> - https://www.youtube.com/watch?v=JEpWlTl95Tw
> - https://renegaderobotics.org/pid-beginners-guide/

## Lookahead Distance

> This is a value used when following curves with [Pure Pursuit](path-following). If you don't plan to use the `move_path` method, you can safely ignore this property and move on.

Lookahead distance is a value used by the [Pure Pursuit Algorithm]() for following a path smoothly. Pure pursuit works by finding the intersection between a circle centered around the drivetrain's current position (with the radius being the lookahead distance) and a line created by the provided path.

![Path following visualization with lookahead circle](https://i.imgur.com/u0KF0Xj.png)

The intersection point between a circle and the path is called the *lookahead point* and is what the robot will actually attempt to move to. You can imagine this algorithm as mimicking how a person drives on a road. The center of the road is the path and the lookahead distance is however far the person sees on the road. The person will steer towards the lookahead point until they reach their destination (the end of the path).

To tune lookahead distance, make the drivetrain follow a path and increase the distance until the path is followed with acceptable accuracy in a reasonable amount of time. A larger lookahead distance will typically yield lower accuracy but faster movement, while a smaller lookahead distance will have higher accuracy but slower movement.

## Example Configuration

Here's an example of what a drivetrain profile *might* look like once properly tuned for movement.

```cpp
// Create a drivetrain with two motor groups and an inertial sensor
auto chassis = tao::DifferentialDrivetrain(left_drive, right_drive, IMU, {

	// PID Gains (no integral term, so technically a PD controller)
	.drive_gains = { 4.24, 0, 0.06 },
	.turn_gains = { 0.82, 0, 0.0875 },

	// Driving movements must be accurate to 0.7 inches to settle.
	.drive_tolerance = 0.7,

	// Turning movements must be accurate to 1.4 degrees to settle.
	.turn_tolerance = 1.4,

	// Lookahead distance of 8.5 inches for curve following
	.lookahead_distance = 8.5,

	// Track width is 13.75 inches
	.track_width = 13.75,

	// 3.25 inch wheel radius (tuned for VEX's new 4-inch omni wheels)
	.wheel_radius = 3.25,

	// 36:60 external gear ratio
	.gearing = (36.0 / 60.0),
});
```