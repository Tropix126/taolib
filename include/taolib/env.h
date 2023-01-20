/**
 * @file src/taolib/env.h
 * @author Tropical
 *
 * Defines platform-speccific methods and classes for
 * simpler cross-compatiblity between VEXcode and PROS.
 */

#pragma once

// Determines the current enviornment
#define TAO_ENV_VEXCODE
// #define TAO_ENV_PROS

// Include the required environment libraries
#ifdef TAO_ENV_VEXCODE
#include "v5_cpp.h"
#elif defined(TAO_ENV_PROS)
#include "api.h"
#endif

#include <memory>

namespace tao::env {

// Useful timer stuff
void sleep_for(uint32_t time);
uint64_t system_time_high_resolution();

// make_unique polyfill
template<class T, class... Args>
std::unique_ptr<T> make_unique(Args&&... args) {
	return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

// Platform-specific classes
#ifdef TAO_ENV_VEXCODE
	using Thread = vex::thread;
	using MotorGroup = vex::motor_group;
	using IMU = vex::inertial;
	using Encoder = vex::encoder;
#elif defined(TAO_ENV_PROS)
	using Thread = pros::Task;
	using MotorGroup = pros::MotorGroup;
	using IMU = pros::IMU;
	using Encoder = pros::ADIEncoder;
#endif

}