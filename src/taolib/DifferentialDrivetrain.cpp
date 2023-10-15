/**
 * @file src/taolib/drivetrain.cpp
 * @author Tropical
 *
 * Abstracts various position tracking and motion control algorithms to
 * provide functions for controlling a physical drivetrain.
 * Given appropriate devices, and a model (called a "config") for
 * tuning physical aspects unique to each robot, the
 * tao::DifferentialDrivetrain class can perform autonomous movement.
 */

#include <cmath>
#include <vector>
#include <iostream>
#include <cstdint>
#include <utility>

#include "taolib/env.h"

#include "taolib/DifferentialDrivetrain.h"
#include "taolib/PIDController.h"
#include "taolib/Vector2.h"
#include "taolib/math.h"
#include "taolib/threading.h"

namespace tao {

// Constructors/Destructors

DifferentialDrivetrain::DifferentialDrivetrain(env::MotorGroup& left_motors,
					   env::MotorGroup& right_motors,
					   env::IMU& imu,
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

DifferentialDrivetrain::DifferentialDrivetrain(env::MotorGroup& left_motors,
					   env::MotorGroup& right_motors,
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

DifferentialDrivetrain::DifferentialDrivetrain(env::MotorGroup& left_motors,
					   env::MotorGroup& right_motors,
					   env::Encoder& left_encoder,
					   env::Encoder& right_encoder,
					   env::IMU& imu,
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

DifferentialDrivetrain::DifferentialDrivetrain(env::MotorGroup& left_motors,
					   env::MotorGroup& right_motors,
					   env::Encoder& left_encoder,
					   env::Encoder& right_encoder,
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

DifferentialDrivetrain::~DifferentialDrivetrain() { stop_tracking(); }

// Getters

Vector2 DifferentialDrivetrain::get_position() {
	mutex.lock();
	Vector2 position = this->position;
	mutex.unlock();
	return position;
}
PIDController::Gains DifferentialDrivetrain::get_drive_gains() const { return drive_controller.get_gains(); }
PIDController::Gains DifferentialDrivetrain::get_turn_gains() const { return turn_controller.get_gains(); }
double DifferentialDrivetrain::get_drive_error() {
	mutex.lock();
	double drive_error = this->drive_error;
	mutex.unlock();
	return drive_error;
}
double DifferentialDrivetrain::get_turn_error() {
	mutex.lock();
	double turn_error = this->turn_error;
	mutex.unlock();
	return turn_error;
}
double DifferentialDrivetrain::get_max_drive_power() const { return max_drive_power; }
double DifferentialDrivetrain::get_max_turn_power() const { return max_turn_power; }
double DifferentialDrivetrain::get_drive_tolerance() const { return drive_tolerance; }
double DifferentialDrivetrain::get_turn_tolerance() const { return turn_tolerance; }
double DifferentialDrivetrain::get_track_width() const { return track_width; }
double DifferentialDrivetrain::get_lookahead_distance() const { return lookahead_distance; }
double DifferentialDrivetrain::get_gearing() const { return gearing; }
double DifferentialDrivetrain::get_wheel_diameter() const { return wheel_diameter; }
DifferentialDrivetrain::Config DifferentialDrivetrain::get_config() const {
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

std::pair<double, double> DifferentialDrivetrain::get_wheel_travel() const {
	double left_rotation, right_rotation;
	double wheel_circumference = wheel_diameter * math::PI;

	if (left_encoder != nullptr && right_encoder != nullptr) {
		left_rotation = env::encoder_get_rotation(*left_encoder);
		right_rotation = env::encoder_get_rotation(*right_encoder);
	} else {
		left_rotation = env::motor_group_get_rotation(left_motors);
		right_rotation = env::motor_group_get_rotation(right_motors);
	}

	return {
		(left_rotation / 360.0) * wheel_circumference * gearing,
		(right_rotation / 360.0) * wheel_circumference * gearing
	};
}

double DifferentialDrivetrain::get_forward_travel() const {
	std::pair<double, double> wheel_travel = get_wheel_travel();
	double average = (wheel_travel.first + wheel_travel.second) / 2;

	return average;
}

double DifferentialDrivetrain::get_heading() {
	// The imu has was passed in, but is not plugged in. Invalidate it's readings for the duration of the tracking routine.
	// IDEA: check for possible spikes in reported heading due to ESD, then invalidate the imu if detected.
	if (!imu_invalid && imu != nullptr && !env::imu_is_installed(*imu)) {
		imu_invalid = true;
		logger.error("IMU was unplugged. Switching to wheeled heading calculation (less accurate).");
	}

	if (imu != nullptr && !imu_invalid) {
		// Use the imu-reported imuscope heading if available.
		return std::fmod((360.0 - env::imu_get_heading(*imu)) + start_heading, 360.0);
	} else {
		// If the imu is not available, then find the heading based on only encoders.
		std::pair<double, double> wheel_travel = get_wheel_travel();

		// Unrestricted counterclockwise-facing heading in radians ((right - left) / trackwidth).
		double raw_heading = (wheel_travel.second - wheel_travel.first) / track_width;

		// Convert to degrees, restrict to 0 <= x < 360, add the user-provided heading offset.
		return std::fmod(math::to_degrees(raw_heading) + start_heading, 360.0);
	}
}
bool DifferentialDrivetrain::is_settled() {
	mutex.lock();
	bool _settled = settled;
	mutex.unlock();
	return _settled;
}

// Setters

void DifferentialDrivetrain::set_turn_tolerance(double error) {
	mutex.lock();
	turn_tolerance = error;
	mutex.unlock();
}
void DifferentialDrivetrain::set_drive_tolerance(double error) {
	mutex.lock();
	drive_tolerance = error;
	mutex.unlock();
}
void DifferentialDrivetrain::set_drive_gains(const PIDController::Gains& gains) {
	mutex.lock();
	drive_controller.set_gains(gains);
	mutex.unlock();
}
void DifferentialDrivetrain::set_turn_gains(const PIDController::Gains& gains) {
	mutex.lock();
	turn_controller.set_gains(gains);
	mutex.unlock();
}
void DifferentialDrivetrain::set_max_drive_power(double power) {
	mutex.lock();
	max_drive_power = power;
	mutex.unlock();
}
void DifferentialDrivetrain::set_max_turn_power(double power) {
	mutex.lock();
	max_turn_power = power;
	mutex.unlock();
}
void DifferentialDrivetrain::set_lookahead_distance(double distance) {
	mutex.lock();
	lookahead_distance = distance;
	mutex.unlock();
}
void DifferentialDrivetrain::set_gearing(double ratio) {
	mutex.lock();
	gearing = ratio;
	mutex.unlock();
}
void DifferentialDrivetrain::set_wheel_diameter(double diameter) {
	mutex.lock();
	wheel_diameter = diameter;
	mutex.unlock();
}

void DifferentialDrivetrain::set_target(Vector2 position) {
	target_type = TargetType::Point;
	target_position = position;
}
void DifferentialDrivetrain::set_target(double distance, double heading) {
	target_type = TargetType::DistanceAndHeading;
	target_distance = distance;
	target_heading = heading;
}

// Threading

int DifferentialDrivetrain::tracking() {
	logger.info("Tracking period started.");

	double previous_forward_travel = 0.0;
	double previous_heading = 0.0;

	// Counter representing the amount of cycles that each PID position has been within it's respective min error range.
	// For example, if the counter reaches 10 then the drivetrain has been within drive_tolerance and turn_tolerance for ~100ms.
	std::int32_t settle_counter = 0;

	// Integrated motor encoders only report at 100hz (once every 10ms).
	constexpr int32_t SAMPLE_RATE = 10;

	while (tracking_active) {
		mutex.lock();

		// Measure the current absolute heading and calculate the change in heading from the last loop sample
		double heading = get_heading();
		double delta_heading = heading - previous_heading;
		previous_heading = heading;

		// Measure the forward travel by taking the average value of all encoders.
		double forward_travel = get_forward_travel();
		double delta_forward_travel = forward_travel - previous_forward_travel;
		previous_forward_travel = forward_travel;

		// assume no sideways travel for now.
		constexpr double delta_sideways_travel = 0.0;

		// Find the average between the current and previous headings
		// This is useful to know when performing odometry calculations, since
		// it provides a more accurate estimation of the robot's heading
		// "during the movement". Simply rotating by the current heading after
		// the movement would build up error faster.
		double average_heading = previous_heading + delta_heading / 2.0;

		// Estimate change in global position
		if (delta_heading == 0.0) {
			// Fallback estimation to avoid divide-by-zero errors
			position += Vector2(delta_forward_travel, delta_sideways_travel).rotated(math::to_radians(average_heading));
		} else {
			// Using chord length formula
			position += Vector2(
				2.0 * (delta_forward_travel / math::to_radians(delta_heading)) * std::sin(math::to_radians(delta_heading / 2)),
				0.0 // 2.0 * (delta_sideways_travel / math::to_radians(delta_heading)) * std::sin(math::to_radians(delta_heading / 2))
			).rotated(math::to_radians(average_heading));
		}

		// Recalculate error for each PID controller.
		// - If in absolute mode, the error is determined by the robot's distance from a point (the target is an absolute Vector2).
		// - If in relative mode, the error is determined by a target encoder distance and heading (The target is heading and distance).
		if (target_type == TargetType::Point) {
			Vector2 local_target = target_position - position;

			turn_error = math::normalize_degrees(heading - math::to_degrees(local_target.get_angle()));
			drive_error = local_target.get_magnitude();

			// If the turn error exceeds 90 degrees, then the point is behind the
			// robot, so it's more efficient to travel to the point backwards.
			if (std::abs(turn_error) >= 90.0) {
				turn_error = math::normalize_degrees(turn_error - 180.0);
				drive_error *= -1.0;
			}
		} else if (target_type == TargetType::DistanceAndHeading) {
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
		if (target_type == TargetType::Point) {
			drive_power *= std::cos(math::to_radians(turn_error));
		}

		// Convert drive and turn power to left and right motor voltages
		std::pair<double, double> normalized_voltages = math::normalize_speeds(
			12.0 * (drive_power + turn_power) / 100,
			12.0 * (drive_power - turn_power) / 100,
			12.0
		);

		// Spin motors at the output voltage.
		env::motor_group_set_voltage(left_motors, normalized_voltages.first);
		env::motor_group_set_voltage(right_motors, normalized_voltages.second);

		// Check if the errors of both loops are under their tolerances.
		// If they are, increment the settle_counter. If they aren't, reset the counter.
		if ((std::abs(drive_error) <= drive_tolerance) && ((std::abs(turn_error) <= turn_tolerance) || target_type == TargetType::Point)) {
			settle_counter++;
		} else {
			settle_counter = 0;
		}

		// Once the settle_counter reaches 5 (~50ms of wait time), the drivetrain is now considered "settled", and
		// blocking movement functions will now complete.
		if (settle_counter >= 5 && !settled) {
			logger.debug("DifferentialDrivetrain has settled. Drive error: %f, Turn error: %f", drive_error, turn_error);

			if (target_type == TargetType::Point) {
				set_target(forward_travel, heading);
			}

			settled = true;
			settle_counter = 0;
		}

		mutex.unlock();
		env::sleep_for(SAMPLE_RATE);
	}

	// Stop the motors before the thread joins to prevent them from running at whatever the last voltage command was.
	env::motor_group_set_voltage(left_motors, 0.0);
	env::motor_group_set_voltage(right_motors, 0.0);

	logger.info("Tracking period stopped.");

	return 0;
}

int DifferentialDrivetrain::logging() {
	int64_t time = 0;

	// Log data periodically
	while (logging_active) {
		mutex.lock();
		Vector2 position_ = position;
		mutex.unlock();
		
		// logger.telemetry("{type:\"TELEMETRY_UPDATE\",data:{position: {x: %f,y: %f},heading: %f,pid: {drive: {kP: %f,kI: %f,kD: %f,},turn: {kP: %f,kI: %f,kD: %f,}}}}");

		// This is a gross way to do this, but I sure as hell don't feel like
		// starting more threads right now.
		if (time % 1000 == 0) {
			logger.info("Position: (%f, %f) Heading: %f\u00B0", position_.get_x(), position_.get_y(), get_heading());
		}

		env::sleep_for(10);
		time += 10;
	}

	return 0;
}

// Lifecycle

void DifferentialDrivetrain::calibrate_imu() {
	if (imu != nullptr) {
		if (!env::imu_is_installed(*imu)) {
			logger.error("IMU not plugged in. Skipping calibration.");
			return;
		}
		
		// Prevent a possible race condition that can occur if the imu isn't detected as plugged in yet.
		env::sleep_for(250);
		env::imu_calibrate(*imu);
		env::sleep_for(100);
		while (env::imu_is_calibrating(*imu)) { env::sleep_for(10); }
		env::sleep_for(250);

		imu_calibrated = true;
	}
}

void DifferentialDrivetrain::start_tracking(Vector2 position, double heading) {
	// Reset sensors
	reset_tracking(position, heading);

	// Start threads
	if (!tracking_active) {
		tracking_active = true;
		tracking_thread = env::make_unique<env::Thread>(threading::make_member_thread(this, &DifferentialDrivetrain::tracking));
	}
	
	if (!logging_active) {
		logging_active = true;
		logging_thread = env::make_unique<env::Thread>(threading::make_member_thread(this, &DifferentialDrivetrain::logging));
	}
}

void DifferentialDrivetrain::reset_tracking(Vector2 position, double heading) {
	env::motor_group_reset_rotation(left_motors);
	env::motor_group_reset_rotation(right_motors);

	if (left_encoder != nullptr) { env::encoder_reset_rotation(*left_encoder); }
	if (right_encoder != nullptr) { env::encoder_reset_rotation(*right_encoder); }

	if (imu != nullptr) {
		while (env::imu_is_calibrating(*imu)) { env::sleep_for(100); }
		if (!imu_calibrated) {
			logger.warning("IMU has not been calibrated! Heading may report inaccurate as a result. Call drivetrain.calibrate_imu() before the tracking period.");
		}
		env::imu_reset_heading(*imu);
	}

	start_heading = heading;
	this->position = position;

	set_target(0.0, start_heading);
}

void DifferentialDrivetrain::stop_tracking() {
	mutex.lock();

	tracking_active = false;
	logging_active = false;
	
	mutex.unlock();

	if (tracking_thread != nullptr) {
		tracking_thread->join();
	}

	if (logging_thread != nullptr) {
		logging_thread->join();
	}
}

void DifferentialDrivetrain::wait_until_settled() {
	// Spinlock until settled
	while (!settled) { env::sleep_for(10); }
}

// Movement

void DifferentialDrivetrain::drive(double distance, bool blocking) {
	mutex.lock();
	settled = false;
	set_target(get_forward_travel() + distance, target_heading);
	mutex.unlock();

	logger.debug("Driving for %f", distance);
	if (blocking) wait_until_settled();
}

void DifferentialDrivetrain::turn_to(double heading, bool blocking) {
	mutex.lock();
	settled = false;
	set_target(target_distance, heading);
	mutex.unlock();

	logger.debug("Turning to %f\u00B0", heading);
	if (blocking) wait_until_settled();
}

void DifferentialDrivetrain::turn_to(Vector2 point, bool blocking) {
	mutex.lock();
	settled = false;
	Vector2 local_target = point - position;
	set_target(target_distance, local_target.get_angle());
	mutex.unlock();

	logger.debug("Turning to (%f, %f). Calculated angle: %f\u00B0", point.get_x(), point.get_y());
	if (blocking) wait_until_settled();
}

void DifferentialDrivetrain::move_to(Vector2 point, bool blocking) {
	mutex.lock();
	settled = false;
	set_target(point);
	Vector2 position = this->position;
	mutex.unlock();

	logger.debug("Moving to (%f, %f). Distance: %f", point.get_x(), point.get_y(), point.distance(position));
	if (blocking) wait_until_settled();
}

void DifferentialDrivetrain::follow_path(std::vector<Vector2> path) {
	logger.debug("Following path.");
	
	mutex.lock();
	settled = false;

	// Add current position to the start of the path so that intersections can be found.
	path.insert(path.begin(), position);
	mutex.unlock();

	// Loop through all waypoints in the provided path.
	for (int i = 0; i < (path.size() - 1); i++) {
		Vector2 start = path[i]; // The current waypoint
		Vector2 end = path[i + 1]; // The next waypoint

		while (position.distance(end) > lookahead_distance) {
			mutex.lock();
			// Find the point(s) of intersection between a circle centered around our global position with the radius of our
			// lookahead distance and a line segment formed between our starting/ending points.
			std::vector<Vector2> intersections = math::line_circle_intersections(position, lookahead_distance, start, end);

			// Choose the best intersection to go to, ensuring that we don't go backwards along the path.
			Vector2 target_intersection;
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

			if (intersections.size() > 0) {
				set_target(target_intersection);
			}

			mutex.unlock();
			
			env::sleep_for(10);
		}
	}

	wait_until_settled();
}

void DifferentialDrivetrain::hold_position() {
	mutex.lock();
	set_target(get_forward_travel(), get_heading());
	mutex.unlock();

	logger.debug("Holding position. Forward Travel: %f, Heading: %f", get_forward_travel(), get_heading());
}

} // namespace tao