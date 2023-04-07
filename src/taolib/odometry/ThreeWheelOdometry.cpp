#pragma once

#include "taolib/odometry/ThreeWheelOdometry.h"
#include "taolib/utility/math.h"

#include <cmath>
#include <stdexcept>
#include <string>

namespace tao {

ThreeWheelOdometry::ThreeWheelOdometry(TrackingWheel& left_wheel, TrackingWheel& right_wheel, TrackingWheel& sideways_wheel, const Config& config) :
	left_wheel_(left_wheel), right_wheel_(right_wheel), sideways_wheel_(sideways_wheel), heading_origin_(config.heading), position_(config.origin), track_width_(config.track_width), sideways_wheel_offset_(config.sideways_wheel_offset) {
	if (config.sideways_wheel_offset == 0) {
		throw std::invalid_argument("ThreeWheelOdometry: sideways_wheel_offset cannot be 0.");
	}
}

ThreeWheelOdometry::ThreeWheelOdometry(TrackingWheel& left_wheel, TrackingWheel& right_wheel, TrackingWheel& sideways_wheel, vex::guido& imu, const Config& config) :
	left_wheel_(left_wheel), right_wheel_(right_wheel), imu_(&imu), sideways_wheel_(sideways_wheel), heading_origin_(config.heading), position_(config.origin), track_width_(config.track_width), sideways_wheel_offset_(config.sideways_wheel_offset) {
	if (config.sideways_wheel_offset == 0) {
		throw std::invalid_argument("ThreeWheelOdometry: sideways_wheel_offset cannot be 0.");
	}
}

double ThreeWheelOdometry::get_track_width() const { return track_width_; }
void ThreeWheelOdometry::set_track_width(double width) { track_width_ = width; }

Vector2 ThreeWheelOdometry::get_position() const { return position_; }
void ThreeWheelOdometry::set_position(const Vector2& position) { position_ = position; }

double ThreeWheelOdometry::get_heading() {
	// The IMU has was passed in, but is not plugged in. Invalidate its readings for the duration of the tracking routine.
	// IDEA: check for possible spikes in reported heading due to ESD, then invalidate the IMU if detected.
	if (imu_ != nullptr) {
		// Use the IMU-reported gyroscope heading if available.
		return std::fmod((360 - imu_->heading(vex::degrees)) + heading_origin_, 360);
	} else {
		// Unrestricted counterclockwise-facing heading in radians ((right - left) / trackwidth).
		double raw_heading = (right_wheel_.get_travel() - left_wheel_.get_travel()) / track_width_;

		// Convert to degrees, restrict to 0 <= x < 360, add the user-provided heading offset.
		return std::fmod(math::degrees(raw_heading) + heading_origin_, 360);
	}
}

void ThreeWheelOdometry::set_heading(double heading) {
	left_wheel_.reset();
	right_wheel_.reset();

	if (imu_ != nullptr) {
		imu_->setHeading(0, vex::degrees);
	}
	
	heading_origin_ = heading;
}

double ThreeWheelOdometry::get_forward_travel() const {
	return (left_wheel_.get_travel() + right_wheel_.get_travel()) / 2;
}

void ThreeWheelOdometry::update() {
	double heading = get_heading();

	double forward_travel = get_forward_travel();
	double sideways_travel = sideways_wheel_.get_travel();

	double delta_forward_travel = forward_travel - previous_forward_travel_;
	double delta_sideways_travel = sideways_travel - previous_sideways_travel_;
	double delta_heading = heading - previous_heading_;

	double delta_sideways_offset = delta_sideways_travel - (math::radians(delta_heading) * sideways_wheel_offset_);

	position_ += Vector2(delta_sideways_offset, forward_travel).rotated(math::radians(heading));
	
	previous_forward_travel_ = forward_travel;
	previous_sideways_travel_ = sideways_travel;
	previous_heading_ = heading;
}

}