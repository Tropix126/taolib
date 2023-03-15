#include "taolib/odometry/ParallelWheelOdometry.h"
#include "taolib/utility/math.h"

#include <cmath>
#include <stdexcept>

namespace tao {

ParallelWheelOdometry::ParallelWheelOdometry(TrackingWheel& left_wheel, TrackingWheel& right_wheel, Config& config) :
	left_wheel_(left_wheel), right_wheel_(right_wheel), heading_origin_(config.heading), position_(config.origin), track_width_(config.track_width) {}

ParallelWheelOdometry::ParallelWheelOdometry(TrackingWheel& left_wheel, TrackingWheel& right_wheel, vex::guido* imu, Config& config) :
	left_wheel_(left_wheel), right_wheel_(right_wheel), imu_(imu), heading_origin_(config.heading), position_(config.origin), track_width_(config.track_width) {}

Vector2 ParallelWheelOdometry::get_position() const { return position_; }
Vector2 ParallelWheelOdometry::set_position(Vector2& position) { position_ = position; }

double ParallelWheelOdometry::get_heading() {
	// The IMU has was passed in, but is not plugged in. Invalidate its readings for the duration of the tracking routine.
	// IDEA: check for possible spikes in reported heading due to ESD, then invalidate the IMU if detected.
	if (imu_ != nullptr) {
		// Use the IMU-reported gyroscope heading if available.
		return std::fmod((360 - imu_->heading(vex::degrees)) + heading_origin_, 360);
	} else {
		// Unrestricted counterclockwise-facing heading in radians ((right - left) / trackwidth).
		double raw_heading = (right_wheel_.get_travel() - left_wheel_.get_travel()) / track_width_;

		// Convert to degrees, restrict to 0 <= x < 360, add the user-provided heading offset.
		return std::fmod(math::radians_to_degrees(raw_heading) + heading_origin_, 360);
	}
}

void ParallelWheelOdometry::set_heading(double heading) {
	left_wheel_.reset();
	right_wheel_.reset();

	if (imu_ != nullptr) {
		imu_->setHeading(0, vex::degrees);
	}
	
	heading_origin_ = heading;
}

double ParallelWheelOdometry::get_forward_travel() const {
	return (left_wheel_.get_travel() + right_wheel_.get_travel()) / 2;
}

void ParallelWheelOdometry::update() {
	double forward_travel = get_forward_travel();
	double heading = get_heading();

	double delta_forward_travel = forward_travel - previous_forward_travel_;

	position_ += Vector2(0, delta_forward_travel).rotated(math::degrees_to_radians(heading));
	
	previous_forward_travel_ = forward_travel;
}

}