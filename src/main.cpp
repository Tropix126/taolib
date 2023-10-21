#include "taolib/taolib.h"
#include <iostream>

vex::motor front_left_drive(vex::PORT1, vex::ratio18_1, false);
vex::motor back_right_drive(vex::PORT10, vex::ratio18_1, true);

vex::motor_group left_drive(front_left_drive);
vex::motor_group right_drive(back_right_drive);

vex::inertial imu(vex::PORT9);

tao::DifferentialDrivetrain drivetrain(left_drive, right_drive, imu, {
	.drive_gains = { 3.24, 0.05, 0.125, 0 },
	.turn_gains = { 2.75, 0, 0.32, 0 },
	.drive_tolerance = 1.0,
	.turn_tolerance = 3.0,
	.lookahead_distance = 12.5,
	.track_width = 11.6,
	.wheel_diameter = 4,
	.gearing = (1.0 / 1.0)
}, tao::Logger(std::cout, tao::Logger::Level::DEBUG));

int main() {	
	using tao::Vector2;

	// drivetrain.calibrate_imu();
	drivetrain.start_tracking();

	// drivetrain.move_to(Vector2(24, 24));

	drivetrain.stop_tracking();

	while (true) {
		tao::env::sleep_for(10);
	}
}