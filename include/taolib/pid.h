/**
 * @file src/taolib/pid.h
 * @author Tropical
 *
 * Closed-loop PID Feedback Controller
 */

#pragma once
#include <iostream>

namespace tao {

/**
 * A structure containing the gain constants for a PID feedback controller.
 * Each component of the PID controller will be multiplied by these gain constants to calculate the final output.
 * The controller must be tuned by adjusting constants until the final output is stable.
 */
typedef struct {
	/** The proportional gain constant. */
	double kP;

	/** The integral gain constant. */
	double kI;

	/** The derivative gain constant. */
	double kD;
} PIDGains;

// PID controller class
class PIDController {
public:
	// Constructor(s)
	PIDController();
	PIDController(PIDGains gains);

	// Update the PID output with the given error and time step
	double update(double error, double delta_time, bool j);

	// Update the controller to use new gains.
	PIDGains get_gains() const;
	void set_gains(const PIDGains& gains);

private:
	// PID gains
	PIDGains gains;

	// Previous error and integral term
	double previous_error, integral;
};

}