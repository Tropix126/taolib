/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Tropical                                                  */
/*    Created:      1/15/2023, 12:11:31 PM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "v5_cpp.h"

#include "taolib/taolib.h"

// A global instance of vex::brain used for printing to the V5 brain screen
vex::brain Brain;

// define your global instances of motors and other devices here
vex::motor front_left_drive(vex::PORT1, vex::ratio18_1, true);
vex::motor back_left_drive(vex::PORT2, vex::ratio18_1, true);
vex::motor front_right_drive(vex::PORT3, vex::ratio18_1, false);
vex::motor back_right_drive(vex::PORT4, vex::ratio18_1, false);

vex::motor_group left_drive(front_left_drive, back_left_drive);
vex::motor_group right_drive(front_right_drive, back_right_drive);

vex::inertial IMU(vex::PORT9);

vex::encoder left_encoder(Brain.ThreeWirePort.A);
vex::encoder right_encoder(Brain.ThreeWirePort.C);

tao::Drivetrain drivetrain(
	left_drive, right_drive,
	{
		.drive_gains = { 4.24, 0, 0.06 },
		.turn_gains = { 0.82, 0.003, 0.0875 },
		.drive_tolerance = 0.7,
		.turn_tolerance = 1.4,
		.lookahead_distance = 8.5,
		.track_width = 13.75,
		.wheel_radius = 2.0202411586464617389578904181119,
		.external_gear_ratio = ((double)84/60),
	}
);

int main() {
	drivetrain.setup_tracking(tao::Vector2(0, 0), 90);
	drivetrain.move_to(tao::Vector2(25, 25));

	while (true) {

		// Allow other tasks to run
		vex::this_thread::sleep_for(10);
	}
}