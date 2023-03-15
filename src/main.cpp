#include <iostream>
#include <memory>

#include "taolib/taolib.h"

#include "v5_cpp.h"

tao::TrackingWheel left_tracker(left_motors, 3.25, (double)(48/60));
tao::TrackingWheel right_tracker(right_motors, 3.25, (double)(48/60));
tao::TrackingWheel sideways_tracker(sideways_encoder, 2.74);

std::make_shared<tao::ThreeWheelOdometry> odometry(left_tracker, right_tracker, sideways_tracker, imu {
	.origin = tao::Vector2(0, 0),
	.heading = 90,
	.track_width = 13.25,
	.sideways_wheel_offset = 2.0
}, tao::Logger::default());

std::make_shared<tao::DifferentialDrivetrain> chassis(left_motors, right_motors, odometry {
	.drive_controller = tao::PIDController({ .kP = 0.3, .kI = 0.0, .kD = 1 }),
	.turn_controller = tao::PIDController({ .kP = 0.3, kI = 0.0, .kD = 1 }),
	.drive_settler = tao::SettleCondition({
		.tolerance = 0.3,
		.tolerance_duration = 0.8
		.timeout_duration = 3
	}),
	.turn_settler = tao::SettleCondition({
		.tolerance = 0.3,
		.tolerance_duration = 0.3,
		.timeout_duration = 8
	}),
	.lookahead_distance = 3
}, tao::Logger::default());

chassis->enable();
chassis->turn_to(90);
chassis->move_to(Vector2(2, 8));
chassis->follow_path(tao::CatmullRomSpline(Vector2(2, 2), Vector2(0, 0)).generate());
chassis->disable();

int main() {
	IMU.calibrate();
	while (IMU.isCalibrating()) { vex::wait(25, vex::msec); }

	drivetrain.setup_tracking(tao::Vector2(0, 0), 90);
	drivetrain.drive(48);
	std::cout << "settled" << std::endl;

	while (true) {
		vex::this_thread::sleep_for(10);
	}
}