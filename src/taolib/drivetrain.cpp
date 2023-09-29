/**
 * @file src/taolib/drivetrain.cpp
 * @author Tropical
 *
 * Abstracts various position tracking and motion control algorithms to
 * provide functions for controlling a physical drivetrain.
 * Given appropriate devices, and a model (called a "config") for
 * tuning physical aspects unique to each robot, the
 * tao::Drivetrain class can perform autonomous movement.
 */

#include <cmath>
#include <vector>
#include <iostream>
#include <cstdint>
#include <utility>

#include "v5_cpp.h"

#include "taolib/drivetrain.h"
#include "taolib/pid.h"
#include "taolib/math.h"
#include "taolib/vector2.h"
#include "taolib/threading.h"

namespace tao {

Drivetrain::Drivetrain(vex::motor_group& left_motors,
					   vex::motor_group& right_motors,
					   vex::inertial& imu,
					   Config config,
					   Logger logger)
	: left_motors(left_motors),
	  right_motors(right_motors),
	  imu(&imu),
	  drive_tolerance(config.drive_tolerance),
	  turn_tolerance(config.turn_tolerance),
	  lookahead_distance(config.lookahead_distance),
	  track_width(config.track_width),
	  wheel_diameter(config.wheel_diameter),
	  gearing(config.gearing),
	  logger(logger) {
	drive_controller.set_gains(config.drive_gains);
	turn_controller.set_gains(config.turn_gains);
}

Drivetrain::Drivetrain(vex::motor_group& left_motors,
					   vex::motor_group& right_motors,
					   Config config,
					   Logger logger)
	: left_motors(left_motors),
	  right_motors(right_motors),
	  imu(nullptr),
	  drive_tolerance(config.drive_tolerance),
	  turn_tolerance(config.turn_tolerance),
	  lookahead_distance(config.lookahead_distance),
	  track_width(config.track_width),
	  wheel_diameter(config.wheel_diameter),
	  gearing(config.gearing),
	  logger(logger) {
	drive_controller.set_gains(config.drive_gains);
	turn_controller.set_gains(config.turn_gains);
}

Drivetrain::Drivetrain(vex::motor_group& left_motors,
					   vex::motor_group& right_motors,
					   vex::encoder& left_encoder,
					   vex::encoder& right_encoder,
					   vex::inertial& imu,
					   Config config,
					   Logger logger)
	: left_motors(left_motors),
	  right_motors(right_motors),
	  left_encoder(&left_encoder),
	  right_encoder(&right_encoder),
	  imu(&imu),
	  drive_tolerance(config.drive_tolerance),
	  turn_tolerance(config.turn_tolerance),
	  lookahead_distance(config.lookahead_distance),
	  track_width(config.track_width),
	  wheel_diameter(config.wheel_diameter),
	  gearing(config.gearing),
	  logger(logger) {
	drive_controller.set_gains(config.drive_gains);
	turn_controller.set_gains(config.turn_gains);
}

Drivetrain::Drivetrain(vex::motor_group& left_motors,
					   vex::motor_group& right_motors,
					   vex::encoder& left_encoder,
					   vex::encoder& right_encoder,
					   Config config,
					   Logger logger)
	: left_motors(left_motors),
	  right_motors(right_motors),
	  left_encoder(&left_encoder),
	  right_encoder(&right_encoder),
	  imu(nullptr),
	  drive_tolerance(config.drive_tolerance),
	  turn_tolerance(config.turn_tolerance),
	  lookahead_distance(config.lookahead_distance),
	  track_width(config.track_width),
	  wheel_diameter(config.wheel_diameter),
	  gearing(config.gearing),
	  logger(logger) {
  drive_controller.set_gains(config.drive_gains);
  turn_controller.set_gains(config.turn_gains);
}

Drivetrain::~Drivetrain() {
	stop_tracking();
}

Vector2 Drivetrain::get_position() const { return global_position; }
PIDController::Gains Drivetrain::get_drive_gains() const { return drive_controller.get_gains(); }
PIDController::Gains Drivetrain::get_turn_gains() const { return turn_controller.get_gains(); }
double Drivetrain::get_drive_error() const { return drive_error; }
double Drivetrain::get_turn_error() const { return turn_error; }
double Drivetrain::get_max_drive_power() const { return max_drive_power; }
double Drivetrain::get_max_turn_power() const { return max_turn_power; }
double Drivetrain::get_drive_tolerance() const { return drive_tolerance; }
double Drivetrain::get_turn_tolerance() const { return turn_tolerance; }
double Drivetrain::get_track_width() const { return track_width; }
double Drivetrain::get_lookahead_distance() const { return lookahead_distance; }
double Drivetrain::get_gearing() const { return gearing; }
double Drivetrain::get_wheel_diameter() const { return wheel_diameter; }
Drivetrain::Config Drivetrain::get_config() const {
	return {
		drive_controller.get_gains(),
		turn_controller.get_gains(),
		drive_tolerance,
		turn_tolerance,
		track_width,
		lookahead_distance,
		wheel_diameter,
		gearing
	};
}

std::pair<double, double> Drivetrain::get_wheel_travel() const {
	double left_travel, right_travel;
	double wheel_circumference = wheel_diameter * math::PI;

	if (left_encoder != nullptr && right_encoder != nullptr) {
		left_travel = (left_encoder->position(vex::degrees) / 360.0) * wheel_circumference * gearing;
		right_travel = (right_encoder->position(vex::degrees) / 360.0) * wheel_circumference * gearing;
	} else {
		left_travel = (left_motors.position(vex::degrees) / 360.0) * wheel_circumference * gearing;
		right_travel = (right_motors.position(vex::degrees) / 360.0) * wheel_circumference * gearing;
	}

	return { left_travel, right_travel };
}

double Drivetrain::get_forward_travel() const {
	std::pair<double, double> wheel_travel = get_wheel_travel();
	double average = (wheel_travel.first + wheel_travel.second) / 2;

	return average;
}

double Drivetrain::get_heading() {
	// The imu has was passed in, but is not plugged in. Invalidate it's readings for the duration of the tracking routine.
	// IDEA: check for possible spikes in reported heading due to ESD, then invalidate the imu if detected.
	if (!imu_invalid && imu != nullptr && !imu->installed()) {
		imu_invalid = true;
		logger.error("IMU was unplugged. Switching to wheeled heading calculation (less accurate).");
	}

	if (imu != nullptr && !imu_invalid) {
		// Use the imu-reported imuscope heading if available.
		return std::fmod((360.0 - imu->heading()) + start_heading, 360.0);
	} else {
		// If the imu is not available, then find the heading based on only encoders.
		std::pair<double, double> wheel_travel = get_wheel_travel();

		// Unrestricted counterclockwise-facing heading in radians ((right - left) / trackwidth).
		double raw_heading = (wheel_travel.second - wheel_travel.first) / track_width;

		// Convert to degrees, restrict to 0 <= x < 360, add the user-provided heading offset.
		return std::fmod(math::to_degrees(raw_heading) + start_heading, 360.0);
	}
}
bool Drivetrain::is_settled() const { return settled; }

void Drivetrain::set_turn_tolerance(double error) { turn_tolerance = error; }
void Drivetrain::set_drive_tolerance(double error) { drive_tolerance = error; }
void Drivetrain::set_drive_gains(const PIDController::Gains& gains) { drive_controller.set_gains(gains); }
void Drivetrain::set_turn_gains(const PIDController::Gains& gains) { turn_controller.set_gains(gains); }
void Drivetrain::set_max_drive_power(double power) { max_drive_power = power; }
void Drivetrain::set_max_turn_power(double power) { max_turn_power = power; }
void Drivetrain::set_lookahead_distance(double distance) { lookahead_distance = distance; }
void Drivetrain::set_gearing(double ratio) { gearing = ratio; }
void Drivetrain::set_wheel_diameter(double diameter) { wheel_diameter = diameter; }

void Drivetrain::set_target(Vector2 position) {
	error_mode = ErrorModes::Absolute;
	target_position = position;
}
void Drivetrain::set_target(double distance, double heading) {
	error_mode = ErrorModes::Relative;
	target_distance = distance;
	target_heading = heading;
}

int Drivetrain::tracking() {
	logger.info("Tracking period started.");

	// Store the wheel travel from the last loop cycle.
	double previous_forward_travel = 0.0;

	// Counter representing the amount of cycles that each PID position has been within it's respective min error range.
	// For example, if the counter reaches 10 then the drivetrain has been within drive_tolerance and turn_tolerance for ~100ms.
	std::int32_t settle_counter = 0;

	// Integrated motor encoders only report at 100hz (once every 10ms).
	constexpr int32_t SAMPLE_RATE = 10;

	while (tracking_active) {
		// Measure the current absolute heading and calculate the change in heading from the last loop cycle
		double heading = get_heading();

		// Measure the forward travel by taking the average value of all encoders.
		double forward_travel = get_forward_travel();
		double delta_forward_travel = forward_travel - previous_forward_travel;
		previous_forward_travel = forward_travel;

		// Perform a basic odometry calculation that uses average wheel travel deltas
		// and the current heading to approximate change in position using a straight
		// line as the hypotenuse.
		//
		// This method is different from some more sophisticated odometry algorithms
		// (such as the one used by The Pilons), in that it estimates arc length as
		// a right triangle rather than forming an arc. The loss in accuracy of this
		// is generally negligible, because it's recalculated at a high sample rate (10ms).
		// For more information: https://rossum.sourceforge.net/papers/DiffSteer/
		global_position += Vector2(delta_forward_travel, 0.0).rotated(math::to_radians(heading));

		// Recalculate error for each PID controller.
		// - If in absolute mode, the error is determined by the robot's distance from a point (the target is an absolute Vector2).
		// - If in relative mode, the error is determined by a target encoder distance and heading (The target is heading and distance).
		if (error_mode == ErrorModes::Absolute) {
			Vector2 local_target = target_position - global_position;

			turn_error = math::normalize_degrees(heading - math::to_degrees(local_target.get_angle()));
			drive_error = local_target.get_magnitude();

			// If the turn error exceeds 90 degrees, then the point is behind the
			// robot, so it's more efficient to travel to the point backwards.
			if (std::abs(turn_error) >= 90.0) {
				turn_error = math::normalize_degrees(turn_error - 180.0);
				drive_error *= -1.0;
			}
		} else if (error_mode == ErrorModes::Relative) {
			turn_error = math::normalize_degrees(heading - target_heading);
			drive_error = target_distance - forward_travel;
		}

		// Get output of PID controllers and cap to max power
		double dt = SAMPLE_RATE / 1000.0;
		double drive_power = math::clamp(drive_controller.update(drive_error, dt), -max_drive_power, max_drive_power);
		double turn_power = math::clamp(turn_controller.update(turn_error, dt), -max_turn_power, max_turn_power);

		// Scale drive power by the cosine of turn_error if moving to a point.
		// This biases turn power over drive power at the start of the movement, which makes the
		// arc shapes less dramatic when moving to a point.
		if (error_mode == ErrorModes::Absolute) {
			drive_power *= std::cos(math::to_radians(turn_error));
		}

		// Convert drive and turn power to left and right motor voltages
		std::pair<double, double> normalized_voltages = math::normalize_speeds(
			12 * (drive_power + turn_power) / 100,
			12 * (drive_power - turn_power) / 100,
			12
		);

		// Spin motors at the output voltage.
		left_motors.spin(vex::forward, normalized_voltages.first, vex::volt);
		right_motors.spin(vex::forward, normalized_voltages.second, vex::volt);

		// Check if the errors of both loops are under their tolerances.
		// If they are, increment the settle_counter. If they aren't, reset the counter.
		if ((std::abs(drive_error) <= drive_tolerance) && ((std::abs(turn_error) <= turn_tolerance) || error_mode == ErrorModes::Absolute)) {
			settle_counter++;
		} else {
			settle_counter = 0;
		}

		// Once the settle_counter reaches 5 (~50ms of wait time), the drivetrain is now considered "settled", and
		// blocking movement functions will now complete.
		if (settle_counter >= 5 && !settled) {
			logger.debug("Drivetrain has settled. Drive error: %f, Turn error: %f", drive_error, turn_error);

			if (error_mode == ErrorModes::Absolute) {
				hold_position();
			}

			settled = true;
			settle_counter = 0;
		}

		vex::this_thread::sleep_for(SAMPLE_RATE);
	}

	// Stop the motors before the thread joins to prevent them from running at whatever the last voltage command was.
	left_motors.stop();
	right_motors.stop();

	logger.info("Tracking period stopped.");

	return 0;
}

int Drivetrain::logging() {
	// Print the current global position of the robot every second.
	while (logging_active) {
		logger.info("Position: (%f, %f) Heading: %f°\n", global_position.get_x(), global_position.get_y(), get_heading());

		vex::this_thread::sleep_for(1000);
	}

	return 0;
}

void Drivetrain::calibrate_imu() {
	if (imu != nullptr) {
		if (!imu->installed()) {
			logger.error("IMU not plugged in. Skipping calibration.");
			return;
		}
		
		// Prevent a possible race condition that can occur if the imu isn't detected as plugged in yet.
		vex::wait(0.25, vex::seconds);
		imu->calibrate();
		vex::wait(0.1, vex::seconds);
		while (imu->isCalibrating()) { vex::wait(0.1, vex::seconds); }
		vex::wait(0.25, vex::seconds);

		imu_calibrated = true;
	}
}

void Drivetrain::start_tracking(Vector2 origin, double heading) {
	// Reset motor encoders.
	left_motors.resetPosition();
	right_motors.resetPosition();

	// Reset imu heading.
	if (imu != nullptr) {
		while (imu->isCalibrating()) { vex::wait(0.1, vex::seconds); }
		if (!imu_calibrated) {
			logger.warning("IMU has not been calibrated! Heading may report inaccurate as a result. Call drivetrain.calibrate_imu() before the tracking period.");
		}
		imu->resetHeading();
	}

	start_heading = heading;
	global_position = origin;

	// Start threads
	if (!tracking_active) {
		tracking_active = true;
		tracking_thread = threading::make_member_thread(this, &Drivetrain::tracking);
	}
	if (!logging_active) {
		logging_active = true;
		logging_thread = threading::make_member_thread(this, &Drivetrain::logging);
	}

	set_target(0.0, start_heading);
}

void Drivetrain::stop_tracking() {
	tracking_active = false;
	logging_active = false;
	
	if (tracking_thread.joinable()) {
		tracking_thread.join();
	}

	if (logging_thread.joinable()) {
		logging_thread.join();
	}
}

void Drivetrain::drive(double distance, bool blocking) {
	logger.debug("Driving for %f", distance);

	settled = false;

	// Set the PID target distance to our desired distance plus our current wheel travel.
	set_target(get_forward_travel() + distance, target_heading);

	while (!settled && blocking) { vex::wait(10, vex::msec); }
}

void Drivetrain::turn_to(double heading, bool blocking) {
	logger.debug("Turning to %f°", heading);
	settled = false;

	set_target(target_distance, heading);
	
	while (!settled && blocking) { vex::wait(10, vex::msec); }
}

void Drivetrain::turn_to(Vector2 point, bool blocking) {
	Vector2 local_target = point - global_position;

	logger.debug("Turning to (%f, %f). Calculated angle: %f°", point.get_x(), point.get_y());

	set_target(target_distance, local_target.get_angle());

	while (!settled && blocking) { vex::wait(10, vex::msec); }
}

void Drivetrain::move_to(Vector2 position, bool blocking) {
	logger.debug("Moving to (%f, %f). Distance: %f", position.get_x(), position.get_y(), global_position.distance(position));
	settled = false;

	set_target(position);

	while (!settled && blocking) { vex::wait(10, vex::msec); }
}

void Drivetrain::follow_path(std::vector<Vector2> path) {
	logger.debug("Following path.");
	settled = false;

	// Add current position to the start of the path so that intersections can be found.
	path.insert(path.begin(), global_position);

	// Loop through all waypoints in the provided path.
	for (int i = 0; i < (path.size() - 1); i++) {
		Vector2 start = path[i]; // The current waypoint
		Vector2 end = path[i + 1]; // The next waypoint

		while (global_position.distance(end) > lookahead_distance) {
			// Find the point(s) of intersection between a circle centered around our global position with the radius of our
			// lookahead distance and a line segment formed between our starting/ending points.
			std::vector<Vector2> intersections = math::line_circle_intersections(global_position, lookahead_distance, start, end);

			Vector2 target_intersection;

			// Choose the best intersection to go to, ensuring that we don't go backwards along the path.
			if (intersections.size() == 2) {
				// There are two intersections. Find the one closest to the end of the line segment.
				if (intersections[0].distance(end) < intersections[1].distance(end)) {
					target_intersection = intersections[0];
				} else {
					target_intersection = intersections[1];
				}
			} else if (intersections.size() == 1) {
				// There is one intersection. Go to that intersection.
				target_intersection = intersections[0];
			}

			// Move to the target intersection
			if (intersections.size() > 0) {
				set_target(target_intersection);
			}

			vex::wait(10, vex::msec);
		}
	}

	while (!settled) { vex::wait(10, vex::msec); }
}

void Drivetrain::hold_position() {
	set_target(get_forward_travel(), get_heading());
}

} // namespace tao