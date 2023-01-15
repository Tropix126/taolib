#include "taolib/pid.h"
#include <iostream>
#include <stdio.h>

namespace tao {

PIDController::PIDController(PIDGains gains) : gains(gains), previous_error(0), integral(0) {}
PIDController::PIDController() : gains({ 0, 0, 0 }), previous_error(0), integral(0) {}

void PIDController::set_gains(const PIDGains& gains) { this->gains = gains; }
PIDGains PIDController::get_gains() const { return gains; }

double PIDController::update(double error, double delta_time) {
	// Avoid sudden derivative spikes on the first loop iteration.
	if (previous_error == 0.0) {
		previous_error = error;
	}

	// Calculate the integral term
	integral += error * delta_time;

	// Calculate the derivative term
	double derivative = (error - previous_error) / delta_time;

	// Calculate the PID output
	double output = (gains.kP * error) + (gains.kI * integral) + (gains.kD * derivative);

	// Update the previous error for the next iteration
	previous_error = error;

	return output;
}


}