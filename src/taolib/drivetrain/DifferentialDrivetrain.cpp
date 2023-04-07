#pragma once

#include <cmath>
#include <vector>
#include <iostream>
#include <utility>

#include "v5_cpp.h"

#include "taolib/drivetrain/DifferentialDrivetrain.h"
#include "taolib/utility/math.h"

namespace tao {

DifferentialDrivetrain::DifferentialDrivetrain(vex::motor_group& left_motors, vex::motor_group& right_motors, Odometry& odometry, const Config& config, const Logger& logger)
: left_motors_(left_motors), right_motors_(right_motors), odometry_(odometry), drive_controller_(config.drive_controller), turn_controller_(config.turn_controller),
  drive_tolerance_(config.drive_tolerance), turn_tolerance_(config.turn_tolerance), lookahead_distance_(config.lookahead_distance), logger_(&logger) {
}

DifferentialDrivetrain::DifferentialDrivetrain(vex::motor_group& left_motors, vex::motor_group& right_motors, Odometry& odometry, const Config& config)
: left_motors_(left_motors), right_motors_(right_motors), odometry_(odometry), drive_controller_(config.drive_controller), turn_controller_(config.turn_controller),
  drive_tolerance_(config.drive_tolerance), turn_tolerance_(config.turn_tolerance), lookahead_distance_(config.lookahead_distance) {
}

DifferentialDrivetrain::~DifferentialDrivetrain() {}

Odometry& DifferentialDrivetrain::get_odometry() const { return odometry_; }
const Logger& DifferentialDrivetrain::get_logger() const { return *logger_; }

double DifferentialDrivetrain::get_drive_error() const { return drive_error_; }
double DifferentialDrivetrain::get_turn_error() const { return turn_error_; }

double DifferentialDrivetrain::get_max_drive_power() const { return max_drive_power_; }
double DifferentialDrivetrain::get_max_turn_power() const { return max_turn_power_; }
void DifferentialDrivetrain::set_max_drive_power(double power) { max_drive_power_ = power; }
void DifferentialDrivetrain::set_max_turn_power(double power) { max_turn_power_ = power; }

MotionController& DifferentialDrivetrain::get_drive_controller() const { return drive_controller_; }
MotionController& DifferentialDrivetrain::get_turn_controller() const { return turn_controller_; }

double DifferentialDrivetrain::get_drive_tolerance() const { return drive_tolerance_; }
double DifferentialDrivetrain::get_turn_tolerance() const { return turn_tolerance_; }
void DifferentialDrivetrain::set_drive_tolerance(double tolerance) { drive_tolerance_ = tolerance; }
void DifferentialDrivetrain::set_turn_tolerance(double tolerance) { turn_tolerance_ = tolerance; }

double DifferentialDrivetrain::get_lookahead_distance() const { return lookahead_distance_; }
void DifferentialDrivetrain::set_lookahead_distance(double distance) { lookahead_distance_ = distance; }

void DifferentialDrivetrain::enable() { enabled_ = true; }
void DifferentialDrivetrain::disable() { enabled_ = false; }
bool DifferentialDrivetrain::is_enabled() const { return enabled_; }
bool DifferentialDrivetrain::is_settled() const { return settled_; }

int DifferentialDrivetrain::daemon_(void* args) {
    uint64_t previous_time = vex::timer::systemHighResolution();
    int32_t settle_counter = 0;

    while (daemon_thread_active_) {
        // Measure the current time in microseconds and calculate how long the last cycle took to complete in milliseconds.
		uint64_t current_time = vex::timer::systemHighResolution();
		double delta_time = (double)(current_time - previous_time) * 0.000001;

        odometry_.update();
        // logger_->debug("[DifferentialDrivetrain] Got odometry update: (" + odometry_.get_position().get_x() + ", " + odometry_.get_position().get_y() + ")");

        Vector2 position = odometry_.get_position();
        double heading = odometry_.get_heading();
        double forward_travel = odometry_.get_forward_travel();

        // Recalculate error for each PID controller.
		// - If in absolute mode, the error is determined by the robot's distance from a point (the target is an absolute Vector2).
		// - If in relative mode, the error is determined by a target encoder distance and heading (The target is heading and distance).
		if (movement_type_ == MovementTypes::Absolute) {
			Vector2 local_target = target_position_ - position;

			turn_error_ = math::normalize_degrees(heading - math::degrees(local_target.get_angle()));
			drive_error_ = local_target.get_magnitude();

			// Update relative targets for when we settle.
			target_distance_ = forward_travel;
			target_heading_ = heading;
			
			drive_error_ *= std::cos(math::radians(turn_error_));
		} else if (movement_type_ == MovementTypes::Relative) {
			turn_error_ = math::normalize_degrees(heading - target_heading_);
			drive_error_ = target_distance_ - forward_travel;
		}

		// Get output of linear and angular motion controlles and clamp between max drive/turn power.
		double drive_power = math::clamp(
			drive_controller_.update(drive_error_, delta_time),
			-max_drive_power_,
			max_drive_power_
		);
		double turn_power = math::clamp(
			turn_controller_.update(turn_error_, delta_time),
			-max_turn_power_,
			max_turn_power_
		);

		// Scale drive power by turn error.
		// This does two things:
		// 1. "Flattens" out the driven arc so that the chassis doesn't overshoot and land at an odd angle.
		// 2. Moves the chassis backwards if the point is behind it
		if (movement_type_ == MovementTypes::Absolute) {
			drive_power *= std::cos(math::radians(turn_error_));
		}

		// Find left and right motor voltages
		std::pair<double, double> motor_voltages = math::normalize_speeds(
			12 * (drive_power + turn_power) / 100,
			12 * (drive_power - turn_power) / 100,
			12
		);

		// Spin motors
		left_motors_.spin(vex::forward, motor_voltages.first, vex::volt);
		right_motors_.spin(vex::forward, motor_voltages.second, vex::volt);

        // Integrated motor encoders only report at 100hz (once every 10ms).
        // Additionally ADI access on CPU0 is soft-capped at 100hz.
		vex::this_thread::sleep_for(10);

		// Measure how long this cycle took to complete for calculating delta_time
		previous_time = current_time;
    }

    return 0;
}

void DifferentialDrivetrain::drive(double distance, bool blocking) {
    // Dummy implementation
}

void DifferentialDrivetrain::turn_to(double heading, bool blocking) {
    // Dummy implementation
}

void DifferentialDrivetrain::turn_to(const Vector2& point, bool blocking) {
    // Dummy implementation
}

void DifferentialDrivetrain::move_to(const Vector2& position, bool blocking) {
    // Dummy implementation
}

void DifferentialDrivetrain::follow_path(const std::vector<Vector2>& path) {
    // Dummy implementation
}

void DifferentialDrivetrain::hold_position(bool blocking) {
    // Dummy implementation
}

void DifferentialDrivetrain::wait_until_settled() {
	// Dummy implementation
}

}