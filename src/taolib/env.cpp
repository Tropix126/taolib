/**
 * @file src/taolib/env.cpp
 * @author Tropical
 *
 * Defines platform-speccific methods and classes for
 * simpler cross-compatiblity between VEXcode and PROS.
 */

#include "taolib/env.h"
#include "taolib/math.h"

#include <cstdint>

namespace tao {
namespace env {

#ifdef TAO_ENV_VEXCODE

bool imu_is_installed(vex::inertial& imu) { return imu.installed(); }
bool imu_is_calibrating(vex::inertial& imu) { return imu.isCalibrating(); }
double imu_get_heading(vex::inertial& imu) { return imu.heading(vex::degrees); }
void imu_calibrate(vex::inertial& imu) { imu.calibrate(); }
void imu_reset_heading(vex::inertial& imu) { imu.resetHeading(); }

void motor_group_set_voltage(vex::motor_group& group, double voltage) {
	group.spin(vex::forward, voltage, vex::percent);
}
double motor_group_get_rotation(vex::motor_group& group) {
	return group.position(vex::degrees);
}
void motor_group_reset_rotation(vex::motor_group& group) {
	group.resetPosition();
}

int32_t encoder_get_rotation(vex::encoder& encoder) {
	return encoder.rotation(vex::degrees);
}
void encoder_reset_rotation(vex::encoder& encoder) {
	encoder.resetRotation();
}

#elif defined(TAO_ENV_PROS)

bool imu_is_installed(pros::v5::Imu& imu) { return imu.is_installed(); }
bool imu_is_calibrating(pros::v5::Imu& imu) { return imu.is_calibrating(); }
double imu_get_heading(pros::v5::Imu& imu) { return imu.get_heading(); }
void imu_calibrate(pros::v5::Imu& imu) { imu.reset(); }
void imu_reset_heading(pros::v5::Imu& imu) { imu.tare_heading(); }

void motor_group_set_voltage(pros::v5::MotorGroup& group, double voltage) {
	group.move((voltage / 12.0) * 127.0);
}
double motor_group_get_rotation(pros::v5::MotorGroup& group) {
	return math::vector_average(group.get_position_all());
}
void motor_group_reset_rotation(pros::v5::MotorGroup& group) {
	group.tare_position_all();
}

int32_t encoder_get_rotation(pros::adi::Encoder& encoder) {
	return encoder.get_value();
}
void encoder_reset_rotation(pros::adi::Encoder& encoder) {
	encoder.reset();
}

#endif

} // namespace env
} // namespace tao