---
title: Basic Drivetrain Movement
layout: ~/layouts/DocsLayout.astro
---

# Drivetrain Setup

In this tutorial, we'll cover the basics of creating a Drivetrain object, tuning the parameters of the drivetrain to create a profile, and moving the robot around using taolib.

# Setting Up Devices

First you'll need to configure your drivetrain's devices (motors, sensors, etc). taolib supports a few different device configurations:
* At minimum, 2 parallel motors controlling wheels of the same size.
  * You can have as many motors as you wish on each side of the drivetrain (4, 6, 8, etc...)
* You can optionally include a [VEX inertial sensor (IMU)]() for considerably more accurate turns.
* You can optionally use two parallel [3-Wire Optical Shaft Encoders](https://kb.vex.com/hc/en-us/articles/360039512851-Using-the-V5-3-Wire-Optical-Shaft-Encoder) attached to [tracking wheels](https://gm0.org/en/latest/docs/common-mechanisms/dead-wheels.html) to prevent wheel slipping from messing up tracking.

> If you're on VEXcode Pro, you can add the required devices through the Robot Configuration button at the top right of the application window.

First, create two [motor groups](https://kb.vex.com/hc/en-us/articles/360058592211-Configuring-a-Motor-Group-in-VEXcode-V5) for your left and right side motors.

> It is possible that one of these motor groups will need to be reversed to properly control the drivetrain. taolib does not handle this for you, so it is important that you check if one side of your drivetrain has its motors flipped upside-down. If it does, then set the appropriate motor group side to run in reverse.

If you wish to use an inertial sensor or external encoders, add those as devices with the appropriate ports as well.

# Creating a Drivetrain Object

After following the [getting started guide]() and setting up taolib, you're ready to create a `tao::Drivetrain` instace. This class is used for controlling all movement provided by taolib.

## Setup with 2 `vex::motor_group`s
```cpp
auto chassis = tao::Drivetrain(left_drive, right_drive, IMU, profile);
```

This device configuration will initialize the `tao::Drivetrain` instance with two parallel motor groups and nothing else (the bare minimum configuration).

## Setup with an IMU
```cpp
auto chassis = tao::Drivetrain(left_drive, right_drive, IMU, profile);
```

Similar to the previous configuration, we have two motor groups, but have additionally added in a V5 Inertial Sensor for accurate turns.
> You'll need to calibrate the Inertial Sensor at the start of pre-autonomous. taolib doesn't handle this for you and **this is an important step if you want to use a gyro for turning.** Example code:
> ```cpp
> auto chassis = tao::Drivetrain(left_drive, right_drive, IMU, profile);
>
> bool IMU_calibrated = false;
> void pre_auton() {
>   // Perform calibration and ensure that it's properly done before setting IMU_calibrated to true.
>  	IMU.calibrate();
>  	waitUntil(!IMU.isCalibrating());
>  	IMU_calibrated = true;
> }
> ...
> // We need to make sure that the robot can't be moved while the IMU is calibrated, so wait until it is in both auton and driver control.
> void autonomous() {
>  	waitUntil(IMU_calibrated);
> }
> ...
> void usercontrol() {
>  	waitUntil(IMU_calibrated);
> }
> ```

## Setup with tracking wheels

The following device configurations can be used if you wish to use two parallel tracking wheels on your drivetrain. These configurations accept two `vex::encoder` instances along with your motor groups and optionally an IMU:

### With IMU
```cpp
auto chassis = tao::Drivetrain(left_drive, right_drive, left_encoder, right_encoder, IMU, profile);
```

### Without IMU
```cpp
auto chassis = tao::Drivetrain(left_drive, right_drive, left_encoder, right_encoder, profile);
```

# Creating a Drivetrain Profile

We aren't done yet. The next (and arguably most important step) is to tune our drivetrain. We do this by creating a *profile*, which will describe to taolib certain attributes (such as measurements and tolerances) that will be used by the drivetrain to calculate correct movements.

You might recall that in the last section, we passed in a `profile` object to `tao::Drivetrain`. Lets fill that in with an example profile for our drivetrain:

```cpp
auto chassis = tao::Drivetrain(left_drive, right_drive, {
	.drive_gains = { 0, 0, 0 },
	.turn_gains = { 0, 0, 0 },
	.drive_tolerance = 0.0,
	.turn_tolerance = 0.0,
	.lookahead_distance = 0.0,
	.track_width = 0.0,
	.wheel_radius = 0.0,
	.external_gear_ratio = (double)(1 / 1),
});
```

All these values are set to 0, though, and need to be tuned for your specific drivetrain configuration. Let's go over each of them individually (backwards, since the first 4 values are the hardest to tune).

## External Gear Ratio

The `external_gear_ratio` parameter of `tao::DrivetrainProfile` describes the gear ratio used by your drivetrain. The parameter accepts a fraction of `(double)(DRIVE_TEETH / DRIVEN_TEETH)`. Let's say you have a drivetrain where your motors are on an 86 tooth gear engaged with a 60 tooth gear attached to your wheels. Your gear ratio would be `.external_gear_ratio = (double)(84 / 60)`.

If you have no gear ratio at all (your wheels are directly powered by motors), then leave this property as `1` to say you have a 1:1 direct drive.

## Wheel Radius

This value describes the radius of your drivetrain's driven wheels. Measure the distance between the center of your wheels to the outside edge, and set that measurement as your wheel radius.

> IMPORTANT: The units you use for measurement here (and in `.track_width` below) will determine the units used for everything else (including odometry and movement functions).

## Track Width

This value describes the distance between the center of your left wheels to the center of your right wheels. Track width is used for finding the drivetrain's absolute heading using encoders if you don't have an IMU or if the IMU was unplugged.

![Track Width Visualization](https://kb.vex.com/hc/article_attachments/360085801912/track_width.jpg)

*You must measure this with the same units you used for your wheel radius*.

## Tolerances

The two tolerance values (`drive_tolerance`, `turn_tolerance`) are used to determine when the drivetrain has "settled" at its target location. These values represent the minimum acceptable *range of error* that is required for the drivetrain to be considered at its target. `drive_tolerance` is the minimum distance error for driving straight and `turn_tolerance` is the minimum degrees of rotational error for turning.

> Movement errors must be within |tolerance| for at least 50ms for the drivetrain to be considered settled.

Tolerances that are too low will result in the drivetrain either never fully settling or taking longer than expected to settle. Tolerances that are too high will result in blocking movement commands to unblock before the drivetrain has reached an acceptable target.

## PID Gains

PID Gains are a set of three numbers (called `kP`, `kI`, and `kD`) that are used to tune how fast and how smoothly the drivetrain will move to reach its target. If you aren't familiar with what a PID controller is, you can read an introduction to that [here]().

taolib utilizes two PID controllers to move the drivetrain. One outputs linear velocity (for driving straight) and the other outputs angular velocity (for turning). The gains of each PID controller must be tuned until both driving and turning actions come to a smooth and controlled stop in a reasonable amount of time.

To turn `turn_gains` you'll want to setup a basic autonomous routine that turns the robot a certain amount.

```cpp
void autonomous() {
	chassis.setup_tracking(tao::Vector2(0, 0), 90);
	chassis.turn_to(180);
}
```

From there, the general process for tuning is as follows:
1. Increase `kP` (the first number AKA the proportional constant) until the robot reaches the target (for example, 180 degrees) in a reasonable amount of time and oscillates around it (bounces back/forth around the target). The oscillations should not be large enough to increase over time. Rather, you want to find a `kP` value that causes minor overshoot of the target and has oscillations that decrease and eventually reach the target.
2. From there, increase `kD` (the third number AKA the derivative constant) until the oscillations stop. A `kD` value that is too high will cause unpredictable movements, so increase it in small increments each time you test the movement.
3. If the robot *consistently undershoots the target every time*, you *might* want to increase `kI` (the second number AKA the integral constant) a very slight amount.
	> Keep in mind that undershoots are often caused by `kP` being too low, and not the need for an integral term. For almost all cases, `kI` should stay at or near 0, because the integral term is susceptable to [integral windup](), which can cause unpredictable movement.

A similar process will be used for tuning the drivetrain's `drive_gains`. We can run a test movement that drives forward 24 inches (1 VEX field tile) to tune linear movement:

```cpp
void autonomous() {
	chassis.setup_tracking(tao::Vector2(0, 0), 90);
	chassis.drive(24);
}
```

From there, follow the steps above to tune `drive_gains`.

> If you're having trouble figuring out how to tune PID gains or understanding this step, then the following additional resources might helpful:
> - https://georgegillard.com/documents/2-introduction-to-pid-controllers
> - https://www.youtube.com/watch?v=JEpWlTl95Tw
> - https://renegaderobotics.org/pid-beginners-guide/

## Lookahead Distance

> This is a value used when following curves with [Pure Pursuit](). If you don't plan to use the `move_path` method, you can safely ignore this property and move on.

Lookahead distance is a value used by the [Pure Pursuit Algorithm]() for following a path smoothly. Pure pursuit works by finding the intersection between a circle centered around the drivetrain's current position (with the radius being the lookahead distance) and a line created by the provided path.

![Path following visualization with lookahead circle](https://i.imgur.com/u0KF0Xj.png)

The intersection point between a circle and the path is called the *lookahead point* and is what the robot will actually attempt to move to. You can imagine this algorithm as mimicking how a person drives on a road. The center of the road is the path and the lookahead distance is however far the person sees on the road. The person will steer towards the lookahead point until they reach their destination (the end of the path).

To tune lookahead distance, make the drivetrain follow a path and increase the distance until the path is followed with acceptable accuracy in a reasonable amount of time. A larger lookahead distance will typically yield lower accuracy but faster movement, while a smaller lookahead distance will have higher accuracy but slower movement.

## Example Profile

Here's an example of what a drivetrain profile *might* look like once properly tuned for movement.

```cpp
// Create a drivetrain with two motor groups and an inertial sensor
auto chassis = tao::Drivetrain(left_drive, right_drive, IMU, {

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

	// 2.02 inch wheel radius (tuned for VEX's new 4-inch omni wheels)
	.wheel_radius = 2.02,

	// 84:60 external gear ratio
	.external_gear_ratio = ((double)84 / 60),
});
```