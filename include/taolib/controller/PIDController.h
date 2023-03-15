/**
 * @file src/taolib/pid.h
 * @author Tropical
 *
 * Closed-loop PID Feedback Controller
 */

#pragma once

#include <iostream>

#include "taolib/controller/MotionController.h"

namespace tao {

// PID controller class
class PIDController: public MotionController {
public:
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
	} Gains;

	// Constructor(s)
	PIDController();
	PIDController(Gains gains);

	// Update the PID output with the given error and time step
	double update(double error, double delta_time) override;

	// Update the controller to use new gains.
	Gains get_gains() const;
	void set_gains(const Gains& gains);

private:
	// PID gains
	Gains gains;

	// Previous error and integral term
	double previous_error, integral;
};

}
