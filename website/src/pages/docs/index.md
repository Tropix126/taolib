---
title: Getting Started
layout: ~/layouts/DocsLayout.astro
---

# What is taolib?
taolib is an easy-to-use C++ library for creating accurate autonomous routines on VEX V5 robots. It's compatible with VEXcode (although [PROS](https://pros.cs.purdue.edu/) support is planned). The library implements PID controllers for motion control, position tracking, and path following using Pure Pursuit.

# Setting Things Up

## Prerequisites
* Basic knowledge of the filesystem.
* Web browser of your choice (well, you're here, aren't you?)
* [VEXcode Pro V5]() or the [VEX Visual Studio Code extension]().

## Downloading the library
1. Head to the releases page on [GitHub](https://github.com/tropix126/taolib/releases/latest)
2. Download the latest version of the library as a zip file
3. Extract the contents of the zip file to a directory of your choice

## Installing the library
1. Copy the `src/taolib` directory from the extracted folder to your project's `src` folder.
2. Copy the `include/taolib` directory from the extracted folder to your project's `include` folder.
3. That's it! You should now be able to include the library in your project's `main.cpp` file:
   ```cpp
   #include "taolib/taolib.h"
   ```

# Next Steps
* If you're new to writing autonomous routines, check out some of our more beginner-friendly tutorials. We recommend starting with the tutorial on [Basic Drivetrain Movement]().
* If you've already done this sort of thing before, you can take a look at the [API documentation](), which provides some helpful information on what's available to you in the library.
* Want to learn more about how the library works? There are [writeups on that too](), including code examples.