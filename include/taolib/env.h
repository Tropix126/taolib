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
	void sleep_for(uint32_t time);
	int64_t system_time_high_resolution();

#ifdef TAO_ENV_VEXCODE
	typedef vex::thread PlatformThread;
	typedef vex::mutex PlatformMutex;
#elif defined(TAO_ENV_PROS)
	typedef pros::Task PlatformThread;
	typedef pros::Mutex PlatformMutex;
#endif
}