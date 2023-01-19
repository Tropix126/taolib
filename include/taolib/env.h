#pragma once

#define TAO_ENV_VEXCODE
// #define TAOLIB_ENV_PROS

#ifdef TAO_ENV_VEXCODE
#include "v5_cpp.h"
#elif defined(TAO_ENV_PROS)
#include "pros/rtos.hpp"
#include "pros/motors.hpp"
#include "pros/imu.hpp"
#include "pros/adi.hpp"
#endif

namespace tao::env {
	// Platform-specific timer stuff
	void sleep_for(uint32_t time);
	int64_t system_time_high_resolution();

	// Platform-specific aliases
#ifdef TAO_ENV_VEXCODE
	using Thread = vex::thread;
	using Mutex = vex::mutex;
	using MotorGroup = vex::motor_group;
	using IMU = vex::inertial;
	using Encoder = vex::encoder;
#elif defined(TAO_ENV_PROS)
	using Thread = pros::Task;
	using Mutex = pros::Mutex;
	using MotorGroup = pros::Motor_Group;
	using IMU = pros::IMU;
	using Encoder = pros::ADIEncoder;
#endif
}