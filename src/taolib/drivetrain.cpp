/**
 * @file src/taolib/drivetrain.cpp
 * @author Tropical
 *
 * Abstracts various position tracking and motion control algorithms to
 * provide various functions for controlling a physical drivetrain.
 * Given appropriate devices, and a model (called a "profile") for
 * tuning certain physical aspects unique to each robot, the
 * tao::Drivetrain class can perform various autonomous actions.
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
					   vex::inertial& IMU,
					   DrivetrainProfile profile)
	: left_motors(left_motors),
	  right_motors(right_motors),
	  IMU(&IMU),
	  drive_tolerance(profile.drive_tolerance),
	  turn_tolerance(profile.turn_tolerance),
	  lookahead_distance(profile.lookahead_distance),
	  track_width(profile.track_width),
	  wheel_circumference(profile.wheel_radius * 2 * math::PI),
	  external_gear_ratio(profile.external_gear_ratio) {
	drive_controller.set_gains(profile.drive_gains);
	turn_controller.set_gains(profile.turn_gains);
}

Drivetrain::Drivetrain(vex::motor_group& left_motors,
					   vex::motor_group& right_motors,
					   DrivetrainProfile profile)
	: left_motors(left_motors),
	  right_motors(right_motors),
	  IMU(nullptr),
	  drive_tolerance(profile.drive_tolerance),
	  turn_tolerance(profile.turn_tolerance),
	  lookahead_distance(profile.lookahead_distance),
	  track_width(profile.track_width),
	  wheel_circumference(profile.wheel_radius * 2 * math::PI),
	  external_gear_ratio(profile.external_gear_ratio) {
	drive_controller.set_gains(profile.drive_gains);
	turn_controller.set_gains(profile.turn_gains);
}

Drivetrain::Drivetrain(vex::motor_group& left_motors,
					   vex::motor_group& right_motors,
					   vex::encoder& left_encoder,
					   vex::encoder& right_encoder,
					   vex::inertial& IMU,
					   DrivetrainProfile profile)
	: left_motors(left_motors),
	  right_motors(right_motors),
	  left_encoder(&left_encoder),
	  right_encoder(&right_encoder),
	  IMU(&IMU),
	  drive_tolerance(profile.drive_tolerance),
	  turn_tolerance(profile.turn_tolerance),
	  lookahead_distance(profile.lookahead_distance),
	  track_width(profile.track_width),
	  wheel_circumference(profile.wheel_radius * 2 * math::PI),
	  external_gear_ratio(profile.external_gear_ratio) {
	drive_controller.set_gains(profile.drive_gains);
	turn_controller.set_gains(profile.turn_gains);
}

Drivetrain::Drivetrain(vex::motor_group& left_motors,
					   vex::motor_group& right_motors,
					   vex::encoder& left_encoder,
					   vex::encoder& right_encoder,
					   DrivetrainProfile profile)
	: left_motors(left_motors),
	  right_motors(right_motors),
	  left_encoder(&left_encoder),
	  right_encoder(&right_encoder),
	  IMU(nullptr),
	  drive_tolerance(profile.drive_tolerance),
	  turn_tolerance(profile.turn_tolerance),
	  lookahead_distance(profile.lookahead_distance),
	  track_width(profile.track_width),
	  wheel_circumference(profile.wheel_radius * 2 * math::PI),
	  external_gear_ratio(profile.external_gear_ratio) {
  drive_controller.set_gains(profile.drive_gains);
  turn_controller.set_gains(profile.turn_gains);
}

Drivetrain::~Drivetrain() {
	stop_tracking();
}

Vector2 Drivetrain::get_position() const { return global_position; }
PIDGains Drivetrain::get_drive_gains() const { return drive_controller.get_gains(); }
PIDGains Drivetrain::get_turn_gains() const { return turn_controller.get_gains(); }
double Drivetrain::get_drive_error() const { return drive_error; }
double Drivetrain::get_turn_error() const { return turn_error; }
double Drivetrain::get_max_drive_velocity() const { return max_drive_velocity; }
double Drivetrain::get_max_turn_velocity() const { return max_turn_velocity; }
double Drivetrain::get_drive_tolerance() const { return drive_tolerance; }
double Drivetrain::get_turn_tolerance() const { return turn_tolerance; }
double Drivetrain::get_track_width() const { return track_width; }
double Drivetrain::get_lookahead_distance() const { return lookahead_distance; }
double Drivetrain::get_external_gear_ratio() const { return external_gear_ratio; }
DrivetrainProfile Drivetrain::get_profile() const {
	return {
		drive_controller.get_gains(),
		turn_controller.get_gains(),
		drive_tolerance,
		turn_tolerance,
		track_width,
		lookahead_distance,
		wheel_circumference / 2 / math::PI,
		external_gear_ratio
	};
}
std::pair<double, double> Drivetrain::get_wheel_travel() const {
	double left_travel, right_travel;

	if (left_encoder != nullptr && right_encoder != nullptr) {
		left_travel = left_encoder->position(vex::rev) * wheel_circumference;
		right_travel = right_encoder->position(vex::rev) * wheel_circumference;
	} else {
		left_travel = left_motors.position(vex::rev) * external_gear_ratio * wheel_circumference;
		right_travel = right_motors.position(vex::rev) * external_gear_ratio * wheel_circumference;
	}

	return { left_travel, right_travel };
}
double Drivetrain::get_heading() {
	// The IMU has was passed in, but is not plugged in. Invalidate it's readings for the duration of the tracking routine.
	// IDEA: check for possible spikes in reported heading due to ESD, then invalidate the IMU if detected.
	if (!IMU_invalid && IMU != nullptr && !IMU->installed()) {
		IMU_invalid = true;
	}

	if (IMU != nullptr && !IMU_invalid) {
		// Use the IMU-reported gyroscope heading if available.
		return std::fmod((360 - IMU->heading()) + start_heading, 360);
	} else {
		// If the IMU is not available, then find the heading based on only encoders.
		std::pair<double, double> wheel_travel = get_wheel_travel();

		// Unrestricted counterclockwise-facing heading in radians ((right - left) / trackwidth).
		double raw_heading = (wheel_travel.second - wheel_travel.first) / track_width;

		// Convert to degrees, restrict to 0 <= x < 360, add the user-provided heading offset.
		return std::fmod(math::radians_to_degrees(raw_heading) + start_heading, 360);
	}
}
bool Drivetrain::is_settled() const { return settled; }

void Drivetrain::set_turn_tolerance(double error) { turn_tolerance = error; }
void Drivetrain::set_drive_tolerance(double error) { drive_tolerance = error; }
void Drivetrain::set_drive_gains(const PIDGains& gains) { drive_controller.set_gains(gains); }
void Drivetrain::set_turn_gains(const PIDGains& gains) { turn_controller.set_gains(gains); }
void Drivetrain::set_max_drive_velocity(double velocity) { max_drive_velocity = velocity; }
void Drivetrain::set_max_turn_velocity(double velocity) { max_turn_velocity = velocity; }
void Drivetrain::set_lookahead_distance(double distance) { lookahead_distance = distance; }
void Drivetrain::set_external_gear_ratio(double ratio) { external_gear_ratio = ratio; }

void Drivetrain::set_target_position(Vector2 position) {
	initial_heading = get_heading();

	error_mode = ErrorModes::Absolute;
	target_position = position;
}
void Drivetrain::set_target_distance(double distance) {
	error_mode = ErrorModes::Relative;
	target_distance = distance;
}
void Drivetrain::set_target_heading(double heading) {
	error_mode = ErrorModes::Relative;
	target_heading = heading;
}

int Drivetrain::daemon() {
	// Stores the previous time in microseconds that the last loop cycle started at.
	std::uint64_t previous_time = vex::timer::systemHighResolution();

	// Stores the average encoder revolutions from the last loop cycle.
	std::pair<double, double> previous_wheel_travel = {0, 0};

	double previous_sideways_travel = 0;

	double previous_heading;

	// Counter representing the amount of cycles that each PID position has been within it's respective min error range.
	// For example, if the counter reaches 10 then the drivetrain has been within drive_tolerance and turn_tolerance for ~100ms.
	std::int32_t settle_counter = 0;

	while (daemon_active) {
		// Measure the current time in microseconds and calculate how long the last cycle took to complete in milliseconds.
		std::uint64_t current_time = vex::timer::systemHighResolution();
		double delta_time = (double)(current_time - previous_time) * 0.000001;

		// Measure the current absolute heading and calculate the change in heading from the last loop cycle
		double heading = get_heading();
		double delta_heading = previous_heading - heading;

		previous_heading = heading;

		// Measure the linear left and rigt distance distance the wheels have traveled using encoders.
		std::pair<double, double> wheel_travel = get_wheel_travel();
		std::pair<double, double> delta_travel = {
			wheel_travel.first - previous_wheel_travel.first,
			wheel_travel.second - previous_wheel_travel.second
		};
		previous_wheel_travel = wheel_travel;

		// Find the average distance traveled by all wheels
		double forward_travel = (wheel_travel.first + wheel_travel.second) / 2;
		double delta_forward_travel = (delta_travel.first + delta_travel.second) / 2;

		// Perform a basic odometry calculation that uses average wheel travel deltas
		// and the current heading to approximate change in position using a straight
		// line as the hypotenuse.
		//
		// This method is different from some more sophisticated odometry algorithms
		// (such as the one used by The Pilons), in that it estimates arc length as
		// a right triangle rather than forming an arc. The loss in accuracy of this
		// is generally negligible, because it's recalculated every 10ms.
		// For more information: https://rossum.sourceforge.net/papers/DiffSteer/
		if (sideways_encoder == nullptr) {
			// Use two parallel encoder inputs if heading has not changed or if sideways tracker is unavailable.
			// Treat change in position as a vector (using wheel travel as the y-axis), then rotate that vector by the robot's heading to 
			// find a position delta.
			//
			// This doesn't account for any lateral translation of the robot (for example, the robot sliding sideways on omni-wheels), because
			// we have no way to track side-to-side motion without the presence of a ideways encoder.
			global_position += Vector2(0, delta_forward_travel).rotated(math::degrees_to_radians(heading));
		} else {
			// Find lateral shift from sideways encoder if available.
			constexpr double SIDEWAYS_TRACKER_OFFSET = 0;

			// Find the rotation of the sideways encoder. Convert that rotation to a distance of wheel travel.
			double sideways_travel = sideways_encoder->position(vex::rev) * wheel_circumference;
			double delta_sideways_travel = sideways_travel - previous_sideways_travel;
			previous_sideways_travel = sideways_travel;

			// Approximated lateral shift along the x-axis canceling out change of rotation in place.
			double sideways_offset = delta_sideways_travel - (math::degrees_to_radians(delta_heading) * SIDEWAYS_TRACKER_OFFSET);

			global_position += Vector2(sideways_offset, delta_forward_travel).rotated(math::degrees_to_radians(heading));
		}

		// Recalculate error for each PID controller.
		// - If in absolute mode, the error is determined by the robot's distance from a point (the target is an absolute Vector2).
		// - If in relative mode, the error is determined by a target encoder distance and heading (The target is heading and distance).
		if (error_mode == ErrorModes::Absolute) {
			Vector2 local_target = target_position - global_position;

			turn_error = math::normalize_degrees(heading - math::radians_to_degrees(local_target.get_angle()));
			drive_error = local_target.get_magnitude();

			// Update relative targets for when we settle.
			target_distance = forward_travel;
			target_heading = heading;

			// Reverse the direction that the drivetrain travels to the point if turn error exceeds 90.
			if (std::abs(turn_error) >= 90) {
				turn_error = math::normalize_degrees(turn_error - 180);
				drive_error *= -1;
			}
		} else if (error_mode == ErrorModes::Relative) {
			turn_error = math::normalize_degrees(heading - target_heading);
			drive_error = target_distance - forward_travel;
		}

		// Get output of PID controllers (uncapped velocity percentages)
		double drive_output = drive_controller.update(drive_error, delta_time);
		double turn_output = turn_controller.update(turn_error, delta_time);

		// Calculate the linear/angular final velocity of the motors by clamping the values at the provided max velocities
		double drive_velocity = math::clamp(drive_output, -max_drive_velocity, max_drive_velocity);
		double turn_velocity = math::clamp(turn_output, -max_turn_velocity, max_turn_velocity);

		// If we are driving to a point, scale linear velocity how complete the turn is. This is done by
		// finding the heading of the robot when the move first started, and comparing that to how far the
		// robot is from facing from the point.
		// As the turn gets closer to facing the point (error decreases), the scaling factor will approach 1,
		// running the drivetrain at the full capped output of the PID controller.
		if (error_mode == ErrorModes::Absolute) {
			Vector2 local_target = target_position - global_position;

			// TODO: test out consine scaling
			double turn_scale = 1 - fabs(turn_error / (math::normalize_degrees(initial_heading - math::radians_to_degrees(local_target.get_angle()))));

			drive_velocity *= turn_scale;
		}

		// Calculate the (uncapped) left and motor right voltages.
		double left_voltage = 12 * (drive_velocity + turn_velocity) / 100;
		double right_voltage = 12 * (drive_velocity - turn_velocity) / 100;

		// Normalize the voltages to be within +-12v while preserving the ratio of left to right speed.
		std::pair<double, double> normalized_voltages = math::normalize_speeds(left_voltage, right_voltage, 12);

		// Spin motors at the output voltage.
		left_motors.spin(vex::forward, normalized_voltages.first, vex::volt);
		right_motors.spin(vex::forward, normalized_voltages.second, vex::volt);

		// Check if the errors of both loops are under their tolerances.
		// If they are, increment the settle_counter. If they aren't, then reset the counter.
		if (std::abs(drive_error) <= drive_tolerance && (std::abs(turn_error) <= turn_tolerance || error_mode == ErrorModes::Absolute)) {
			settle_counter++;
		} else {
			settle_counter = 0;
		}

		// Once the settle_counter reaches 10 (~100ms of wait time), the drivetrain is now considered "settled", and
		// blocking movement functions will now complete.
		if (settle_counter >= 5 && !settled) {
			settled = true;

			if (error_mode == ErrorModes::Absolute)  {
				error_mode = ErrorModes::Relative;
				initial_heading = heading;
			}
		}

		// Integrated motor encoders only report at 100hz (once every 10ms).
		vex::this_thread::sleep_for(10);

		// Measure how long this cycle took to complete for calculating delta_time
		previous_time = current_time;
	}

	return 0;
}

int Drivetrain::logging() {
	// Print the current global position of the robot every second.
	while (logging_active) {
		std::cout << "Global Position: " << "(" << global_position.get_x() << ", " << global_position.get_y() << ")" << " Heading: " << get_heading() << std::endl;

		vex::this_thread::sleep_for(1000);
	}

	return 0;
}

void Drivetrain::setup_tracking(Vector2 start_vector, double start_heading, bool enable_logging) {
	// Reset with desired starting data
	reset_tracking(start_vector, start_heading);

	// Start daemon if it isn't already running
	if (!daemon_active) {
		daemon_active = true;
		daemon_thread = threading::make_member_thread(this, &Drivetrain::daemon);
	}

	// Start logging if enable_logging was specified and it isn't already running
	if (enable_logging && !logging_active) {
		logging_active = true;
		logging_thread = threading::make_member_thread(this, &Drivetrain::logging);
	}
}

void Drivetrain::reset_tracking(Vector2 start_vector, double start_heading) {
	// Reset motor encoders.
	left_motors.resetPosition();
	right_motors.resetPosition();

	// Reset gyro heading.
	if (IMU != nullptr) {
		IMU->resetHeading();
	}
	this->start_heading = start_heading;
	set_target_heading(start_heading);

	// Set global position to supplied starting vector.
	global_position = start_vector;
}

void Drivetrain::stop_tracking() {
	daemon_active = false;
	logging_active = false;
	
	if (daemon_thread.joinable()) {
		daemon_thread.join();
		
		left_motors.stop();
		right_motors.stop();
	}

	if (logging_thread.joinable()) {
		logging_thread.join();
	}
}

void Drivetrain::drive(double distance, bool blocking) {
	settled = false;

	std::pair<double, double> wheel_travel = get_wheel_travel();
	double average_wheel_travel = (wheel_travel.first + wheel_travel.second) / 2;

	// Set the PID target distance to our desired distance plus our current wheel travel.
	set_target_distance(average_wheel_travel + distance);

	while (!settled && blocking) { vex::wait(10, vex::msec); }
}

void Drivetrain::turn_to(double heading, bool blocking) {
	settled = false;

	// Set the PID target heading.
	set_target_heading(heading);
	
	while (!settled && blocking) { vex::wait(10, vex::msec); }
}

void Drivetrain::turn_to(Vector2 point, bool blocking) {
	Vector2 local_target = point - global_position;

	turn_to(math::radians_to_degrees(local_target.get_angle()), blocking);
}

void Drivetrain::move_to(Vector2 position, bool blocking) {
	settled = false;

	set_target_position(position);

	while (!settled && blocking) { vex::wait(10, vex::msec); }
}

void Drivetrain::move_path(std::vector<Vector2> path) {
	settled = false;

	// Add current position to the start of the path so that intersections can be found.
	path.insert(path.begin(), global_position);

	// Loop through all waypoints in the provided path.
	for (int i = 0; i < path.size() - 1; i++) {
		Vector2 start = path[i]; // The current waypoint
		Vector2 end = path[i + 1]; // The next waypoint

		while (global_position.distance(end) > lookahead_distance) {
			// Find the point(s) of intersection between a circle centered around our global position with the radius of our
			// lookahead distance and a line segment formed between our starting/ending points.
			std::vector<Vector2> intersections = math::line_circle_intersections(global_position, lookahead_distance, start, end);

			Vector2 target_intersection;

			// Choose the best intersection to go to, ensuring that we don't go backwards.
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

			// Move to the target intersection using relative-mode PID
			if (intersections.size() > 0) {
				Vector2 local_target(target_intersection - global_position);

				std::pair<double, double> wheel_travel = get_wheel_travel();
				double average_wheel_travel = (wheel_travel.first + wheel_travel.second) / 2;

				set_target_distance(average_wheel_travel + local_target.get_magnitude());
				set_target_heading(math::radians_to_degrees(local_target.get_angle()));
			}

			vex::wait(10, vex::msec);
		}
	}

	while (!settled) { vex::wait(10, vex::msec); }
}

void Drivetrain::hold_position(bool blocking) {
	settled = false;

	// Find the current heading and wheel travel and set relative PID targets to those current values
	// This effectively tells the PID controllers to hold both the current heading and the current wheel travel
	std::pair<double, double> wheel_travel = get_wheel_travel();
	double average_wheel_travel = (wheel_travel.first + wheel_travel.second) / 2;

	set_target_distance(average_wheel_travel);
	set_target_heading(get_heading());

	while (!settled && blocking) { vex::wait(10, vex::msec); }
}

}