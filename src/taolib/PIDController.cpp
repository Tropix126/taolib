/**
 * @file src/taolib/pid.cpp
 * @author Tropical
 *
 * Closed-loop PID Feedback Controller
 */

#include "taolib/PIDController.h"
#include "taolib/math.h"
#include <iostream>
#include <stdio.h>
#include <cmath>

namespace tao {

PIDController::PIDController(Gains gains) : gains(gains), previous_error(0), integral(0) {}
PIDController::PIDController() : gains({ 0, 0, 0 }), previous_error(0), integral(0) {}

void PIDController::set_gains(const Gains& gains) { this->gains = gains; }
PIDController::Gains PIDController::get_gains() const { return gains; }

double PIDController::update(double error, double delta_time) {
	// Calculate the integral term if error is within i_threshold.
	if (std::abs(error) < gains.i_threshold) {
		integral += error * delta_time;
	}

	// Reset integral term once the sign of error changes to prevent windup (the robot has overshot).
	if (math::sign(error) != math::sign(previous_error)) {
		integral = 0;
	}

	// Calculate the derivative term
	double derivative = (error - previous_error) / delta_time;

	// Calculate the PID output
	double output = (gains.kP * error) + (gains.kI * integral) + (gains.kD * derivative);

	// Update the previous error for the next iteration
	previous_error = error;

	return output;
}

} // namespace tao