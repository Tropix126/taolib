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
#include <cstdint>
#include <functional>

namespace tao {
namespace env {

// make_unique polyfill
template<class T, class... Args>
std::unique_ptr<T> make_unique(Args&&... args) {
	return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

// Platform-specific aliases
#ifdef TAO_ENV_VEXCODE
	using Thread = vex::thread;
	using Mutex = vex::mutex;
	using MotorGroup = vex::motor_group;
	using IMU = vex::inertial;
	using Encoder = vex::encoder;

	constexpr auto sleep_for = static_cast<void(*)(uint32_t)>(vex::this_thread::sleep_for);
	constexpr auto high_resolution_clock = vex::timer::systemHighResolution;

	auto imu_is_installed = std::mem_fn(&vex::inertial::installed);
	auto imu_get_heading = std::mem_fn(&vex::inertial::heading);
	auto imu_is_calibrating = std::mem_fn(&vex::inertial::isCalibrating);
	auto imu_calibrate = std::mem_fn(&vex::inertial::calibrate);
	auto imu_reset_heading = std::mem_fn(&vex::inertial::resetHeading);

	auto motor_group_reset_rotation = std::mem_fn(&vex::motor_group::resetPosition);

	int32_t encoder_get_rotation(vex::encoder& encoder);
	auto encoder_reset_rotation = std::mem_fn(&vex::encoder::resetRotation);
#elif defined(TAO_ENV_PROS)
	using Thread = pros::Task;
	using Mutex = pros::Mutex;
	using MotorGroup = pros::MotorGroup;
	using IMU = pros::IMU;
	using Encoder = pros::ADIEncoder;

	constexpr auto sleep_for = pros::delay;
	constexpr auto high_resolution_clock = pros::micros;

	auto imu_is_installed = std::mem_fn(&vex::inertial::installed);
	auto imu_get_heading = std::mem_fn(&pros::Imu::get_heading);
	auto imu_is_calibrating = std::mem_fn(&pros::Imu::is_calibrating);
	auto imu_calibrate = std::mem_fn(&pros::Imu::reset);
	auto imu_reset_heading = std::mem_fn(&pros::Imu::tare_heading);

	auto motor_group_reset_rotation = std::mem_fn(&vex::motor_group::resetPosition);
	
	auto encoder_get_rotation = std::mem_fn(&vex::encoder::resetRotation);
	auto encoder_reset_rotation = std::mem_fn(&vex::encoder::resetRotation);
#endif

double motor_group_get_rotation(Encoder& encoder);
void motor_group_set_voltage(MotorGroup& group, double voltage);

}
}