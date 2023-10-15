---
title: Advanced Movement
category: Library
layout: ~/layouts/DocsLayout.astro
page: 4
---

# Advanced Movement

## Asynchronous (Non-blocking) Movements

By default, all movement functions will wait until the movement is completed, or the drivetrain is in it's *settled* state before continuing the program:

```cpp
// Drive 20 inches forward
drivetrain.drive(20);

// After we've driven 20 inches, run a motor called "intake"
intake.spin(vex::forward);
```

But most methods also support asynchronous movement executions, allowing you to do things *during* a movement. By passing in `false` as a second argument to most of these functions, they will not wait until the drivetrain is settled.

In this example, we start a `drive` movement, wait 0.5 seconds after starting, then run the intake while driving forwards:

```cpp
// Start a drive movement for 20 inches
drivetrain.drive(20, false);

// Wait 0.5 seconds
vex::wait(0.5, vex::seconds);

// Run an intake
intake.spin(vex::forward);
```

## Settled State

When a drivetrain reaches its target, it is considered `settled`. Settle behavior is determined by the `drive_tolerance` and `turn_tolerance` parameters passed through the config we made in the [Drivetrain Setup Tutorial](drivetrain-setup). Once a drivetrain is in tolerance for 50 milliseconds, it has settled and the program can continue.

taolib provides a few functions for handling settle state in unique ways:

- `drivetrain.wait_until_settled()` will block the main thread until the drivetrain is settled. This is useful for asynchronous movements where you want to perform an action during the movement, then wait until the movement is done.
- `drivetrain.set_drive_tolerance()` and `drivetrain.set_turn_tolerance()` will control the tolerance values used to determine if the drivetrain is eligible to settle. This will effectively change the "margin of error" that a movement can be in. If the margin of error is too little, the drivetrain may never settle and your program can freeze.

In the previous example, we ran our `intake` motor half a second after driving forward, but we never wait for the movement to complete after starting the movement. We can do this with `wait_until_settled`:

```cpp
// Start a drive movement for 20 inches
drivetrain.drive(20, false);

// Wait 0.5 seconds
vex::wait(0.5, vex::seconds);

// Run an intake
intake.spin(vex::forward);

// Wait until we've reached the target distance (20 inches)
drivetrain.wait_until_settled();

// Now that we've settled, we can do other things with the drivetrain
drivetrain.turn_to(50);
```

## Running Multiple Movements at a Time

By using non-blocking movements, we can create arc-like movements by driving and turning at the same time:

```cpp
drivetrain.drive(10, false);
drivetrain.turn_to(80, false);
drivetrain.wait_until_settled();
```

This starts two movements at the same time - a `drive` for 10 inches and a `turn_to` to 80 degrees. The drivetrain will perform these at the same time and wait until both movements are settled.

> This trick only works with `drive` and `turn_to`! For example, if we were to interrupt a `move_to` with a `turn_to`, it would cancel the first movement and only execute the turn.

## Time-based Movements

Sometimes accuracy doesn't matter as much as the time it takes to execute the movement. In this case, we want to

## Adjusting Speed
