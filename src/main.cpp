#include <iostream>
#include <memory>

#include "taolib/taolib.h"

#include "v5_cpp.h"

// auto chassis = tao::DifferentialDrivetrain(left_motors, right_motors, odometry, {
// 	.drive_controller = tao::PIDController({ 0.3, 0.0, 1 }),
// 	.turn_controller = tao::PIDController({ 0.3, 0.1, 1 }),
// 	.drive_tolerance = 0.3,
// 	.turn_tolerance = 0.5,
// 	.lookahead_distance = 3
// });

int main() {
	tao::Vector2(0, 0);
	// vex::motor example(vex::PORT10);

	// vex::motor_group left_motors(example);
	// vex::motor_group right_motors(example);

	// tao::TrackingWheel left_tracker(left_motors, 3.25, (double)(48/60));
	// tao::TrackingWheel right_tracker(right_motors, 3.25, (double)(48/60));
	// tao::TrackingWheel sideways_tracker(example, 2.74);

	// auto odometry = tao::ParallelWheelOdometry(left_tracker, right_tracker, {
	// 	.origin = tao::Vector2(0, 0),
	// 	.heading = 90,
	// 	.track_width = 13.25
	// });
	// tao::Vector2 pos(0, 0);
	// odometry.set_position(pos);
	// IMU.calibrate();
	// while (IMU.isCalibrating()) { vex::wait(25, vex::msec); }

	// drivetrain.setup_tracking(tao::Vector2(0, 0), 90);
	// drivetrain.drive(48);

	// chassis->enable();
	// chassis->turn_to(90);
	// chassis->move_to(Vector2(2, 8));
	// chassis->follow_path(tao::CatmullRomSpline(Vector2(2, 2), Vector2(0, 0)).generate());
	// chassis->disable();
	// std::cout << "settled" << std::endl;

	// while (true) {
	// 	vex::this_thread::sleep_for(10);
	// }
}