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
#elif defined(TAO_ENV_PROS)
	using Thread = pros::Task;
	using Mutex = pros::Mutex;
	using MotorGroup = pros::MotorGroup;
	using IMU = pros::IMU;
	using Encoder = pros::adi::Encoder;

	constexpr auto sleep_for = pros::delay;
	constexpr auto high_resolution_clock = pros::micros;
#endif

bool imu_is_installed(IMU& imu);
bool imu_is_calibrating(IMU& imu);
double imu_get_heading(IMU& imu);
void imu_calibrate(IMU& imu);
void imu_reset_heading(IMU& imu);

double motor_group_get_rotation(MotorGroup& encoder);
void motor_group_set_voltage(MotorGroup& group, double voltage);
void motor_group_reset_rotation(MotorGroup& motor_group);

int32_t encoder_get_rotation(Encoder& encoder);
void encoder_reset_rotation(Encoder& encoder);

class Timer {
public:
	Timer();
	~Timer();

	int64_t elapsed() const;
	void reset();

private:
	int64_t timestamp;
};

} // namespace env
} // namespace tao