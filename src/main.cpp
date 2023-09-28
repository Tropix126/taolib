#include "taolib/taolib.h"
#include "v5_cpp.h"
#include <iostream>

vex::motor front_left_drive(vex::PORT1, vex::ratio18_1, true);
vex::motor back_left_drive(vex::PORT2, vex::ratio18_1, true);
vex::motor front_right_drive(vex::PORT3, vex::ratio18_1, false);
vex::motor back_right_drive(vex::PORT4, vex::ratio18_1, false);

vex::motor_group left_drive(front_left_drive, back_left_drive);
vex::motor_group right_drive(front_right_drive, back_right_drive);

vex::inertial imu(vex::PORT9);

tao::Drivetrain drivetrain(left_drive, right_drive, imu, {
	.drive_gains = { 6.24, 0.05, 0.125 },
	.turn_gains = { 3.75, 0, 0.32 },
	.drive_tolerance = 1.0,
	.turn_tolerance = 3.0,
	.lookahead_distance = 12.5,
	.track_width = 13.75,
	.wheel_diameter = 3.25,
	.gearing = (36.0 / 60.0)
});

int main() {
	drivetrain.calibrate_imu();
	drivetrain.begin_tracking(tao::Vector2(0, 0), 90);
	
	drivetrain.drive(48);
	
	drivetrain.end_tracking();

	while (true) {
		vex::this_thread::sleep_for(10);
	}
}