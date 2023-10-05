/**
 * @file src/taolib/env.cpp
 * @author Tropical
 *
 * Defines platform-speccific methods and classes for
 * simpler cross-compatiblity between VEXcode and PROS.
 */
#include "taolib/env.h"
#include <cstdint>

namespace tao {
namespace env {

#ifdef TAO_ENV_VEXCODE

bool imu_is_installed(vex::inertial& imu) { return imu.installed(); }
bool imu_is_calibrating(vex::inertial& imu) { return imu.isCalibrating(); }
double imu_get_heading(vex::inertial& imu) { return imu.heading(vex::degrees); }
void imu_calibrate(vex::inertial& imu) { return imu.calibrate(); }
void imu_reset_heading(vex::inertial& imu) { return imu.resetHeading(); }

void motor_group_set_voltage(vex::motor_group& group, double voltage) {
	return group.spin(vex::forward, voltage, vex::percent);
}
double motor_group_get_rotation(vex::motor_group& group) {
	return group.position(vex::degrees);
}
void motor_group_reset_rotation(vex::motor_group& group) {
	return group.resetPosition();
}

int32_t encoder_get_rotation(vex::encoder& encoder) {
	return encoder.rotation(vex::degrees);
}
void encoder_reset_rotation(vex::encoder& encoder) {
	return encoder.resetRotation();
}

#elif defined(TAO_ENV_PROS)

void motor_group_set_voltage(MotorGroup& group, double voltage) {
	return group.move((voltage / 12.0) * 127.0);
}

#endif

}
}