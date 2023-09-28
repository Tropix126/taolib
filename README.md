<h1 align="center">taolib</h1>

<p align="center">
  <a href="#overview">Overview</a> |
  <a href="#development">Development</a> |
  <a href="#contributors">Contributors</a>
</p>

<p align="center">
<br/>
A library for creating autonomous routines on VEX V5 robots.
</p>

---

# Overview

taolib is a library that combines many motion control algorithms such as [PID feedback control](https://renegaderobotics.org/pid-beginners-guide/), [Odometry](http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf), and [Pure Pursuit](https://colab.research.google.com/drive/1fS4KaYXb7F1YQjP9lu66p511gsIPZWFe?usp=sharing#scrollTo=mx9nirOrLlN3) into an easy-to-use library with as little setup as possible, allowing the creation of complex and accurate autonomous routines.

## Downloads

These will link you to the latest builds found in the [releases](https://github.com/Tropix126/taolib/releases/) tab of this repository.

| [VEXcode](#)  | PROS (planned) |
| ------------- | ------------- |



## Codebase (headers)

```
.
└──taolib
    ├──taolib.h         // Entry point of the library.
    ├──drivetrain.h     // Contains the Drivetrain class, which abstracts over position tracking, motion control, etc... to move a physical drivetrain.
    ├──utility.h        // Basic utilities for math operations.
    ├──pid.h            // Basic closed-loop PID Controller.
    ├──threading.h      // Various helpers for dealing with vexcode's threading limitations.
    ├──logger.h         // Logger class for logging data to various places.
    └──vector2.h        // 2D Vector abstraction.
```

---

# Development

---

# Contributors

For information on contributing to this project, please see [CONTRIBUTING.md](/CONTRIBUTING.md).

<a href="https://github.com/tropix126/taolib/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=tropix126/taolib" />
</a>
