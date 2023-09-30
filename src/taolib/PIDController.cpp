/**
 * @file src/taolib/pid.cpp
 * @author Tropical
 *
 * Closed-loop PID Feedback Controller
 */

#include "taolib/PIDController.h"
#include <iostream>
#include <stdio.h>

namespace tao {

PIDController::PIDController(Gains gains) : gains(gains), previous_error(0), integral(0) {}
PIDController::PIDController() : gains({ 0, 0, 0 }), previous_error(0), integral(0) {}

void PIDController::set_gains(const Gains& gains) { this->gains = gains; }
PIDController::Gains PIDController::get_gains() const { return gains; }

double PIDController::update(double error, double delta_time) {
	// Calculate the integral term
	integral += error;

	// Calculate the derivative term
	double derivative = (error - previous_error) / delta_time;

	// Calculate the PID output
	double output = (gains.kP * error) + (gains.kI * integral * delta_time) + (gains.kD * derivative);

	// Update the previous error for the next iteration
	previous_error = error;

	return output;
}

} // namespace tao