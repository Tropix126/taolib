/**
 * @file src/taolib/pid.cpp
 * @author Tropical
 *
 * Closed-loop PID Feedback Controller
 */

#pragma once

#include "taolib/controller/PIDController.h"
#include "taolib/controller/MotionController.h"

#include <iostream>
#include <stdio.h>

namespace tao {

PIDController::PIDController(Gains gains) : gains_(gains), previous_error_(0), integral_(0) {}
PIDController::PIDController() : gains_({ 0, 0, 0 }), previous_error_(0), integral_(0) {}

void PIDController::set_gains(const Gains gains) { gains_ = gains; }
PIDController::Gains PIDController::get_gains() const { return gains_; }

double PIDController::update(double error, double delta_time) {
	// Avoid sudden derivative spikes on the first loop iteration.
	if (previous_error_ == 0.0) {
		previous_error_ = error;
	}

	// Calculate the integral term
	integral_ += error * delta_time;

	// Calculate the derivative term
	double derivative = (error - previous_error_) / delta_time;

	// Calculate the PID output
	double output = (gains_.kP * error) + (gains_.kI * integral_) + (gains_.kD * derivative);

	// Update the previous error for the next iteration
	previous_error_ = error;

	return output;
}

}