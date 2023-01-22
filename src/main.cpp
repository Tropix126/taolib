#include "taolib/taolib.h"
#include "v5_cpp.h"

vex::motor_group left_drive(
	vex::motor(vex::PORT1, vex::ratio18_1, true),
	vex::motor(vex::PORT2, vex::ratio18_1, true)
);
vex::motor_group right_drive(
	vex::motor(vex::PORT3, vex::ratio18_1, false),
	vex::motor(vex::PORT4, vex::ratio18_1, false)
);
vex::inertial IMU(vex::PORT9);

tao::Drivetrain drivetrain(left_drive, right_drive, IMU, {
	.drive_gains = { 4.24, 0, 0.06 },
	.turn_gains = { 0.82, 0.003, 0.0875 },
	.drive_tolerance = 0.7,
	.turn_tolerance = 1.4,
	.lookahead_distance = 8.5,
	.track_width = 13.75,
	.wheel_radius = 2.0202411586464617389578904181119,
	.external_gear_ratio = ((double)84/60),
});

int main() {
	IMU.calibrate();
	while (IMU.isCalibrating()) {
		vex::wait(25, vex::msec);
	}

	drivetrain.setup_tracking(tao::Vector2(0, 0), 90);
	drivetrain.move_to(tao::Vector2(24, 24));

	while (true) {
		vex::this_thread::sleep_for(10);
	}
}