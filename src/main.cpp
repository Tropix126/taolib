/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Tropical                                                  */
/*    Created:      1/15/2023, 12:11:31 PM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

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
	left_drive, right_drive, IMU,
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
	using namespace tao;
	IMU.calibrate();
	while (IMU.isCalibrating()) {
		vex::wait(25, vex::msec);
	}
	drivetrain.setup_tracking(tao::Vector2(0, 0), 90);
	drivetrain.move_path({Vector2(30.69125, 26.64666666666667), Vector2(34.08632083333334, 28.3001875), Vector2(37.514700000000005, 30.025083333333335), Vector2(40.9763875, 31.821354166666666), Vector2(44.471383333333335, 33.68900000000001), Vector2(47.9996875, 35.62802083333333), Vector2(51.5613, 37.638416666666664), Vector2(55.156220833333336, 39.7201875), Vector2(58.78445000000001, 41.873333333333335), Vector2(62.44598750000001, 44.09785416666667), Vector2(66.14083333333333, 46.39375), Vector2(69.46809791666666, 48.67537083333333), Vector2(72.02689166666667, 50.857066666666675), Vector2(73.81721458333332, 52.9388375), Vector2(74.83906666666667, 54.920683333333336), Vector2(75.09244791666667, 56.80260416666667), Vector2(74.57735833333332, 58.5846), Vector2(73.29379791666666, 60.266670833333336), Vector2(71.24176666666666, 61.84881666666667), Vector2(68.42126458333334, 63.3310375), Vector2(64.83229166666666, 64.71333333333334), Vector2(61.31588333333334, 66.06945833333334), Vector2(58.71307500000001, 67.47316666666667), Vector2(57.02386666666666, 68.92445833333333), Vector2(56.24825833333334, 70.42333333333333), Vector2(56.38625, 71.96979166666667), Vector2(57.43784166666667, 73.56383333333335), Vector2(59.40303333333333, 75.20545833333334), Vector2(62.281825000000005, 76.89466666666668), Vector2(66.07421666666667, 78.63145833333333), Vector2(70.78020833333333, 80.41583333333334), Vector2(75.66939583333335, 82.295375), Vector2(80.01137500000002, 84.31766666666668), Vector2(83.80614583333333, 86.48270833333333), Vector2(87.05370833333335, 88.7905), Vector2(89.7540625, 91.24104166666666), Vector2(91.90720833333333, 93.83433333333333), Vector2(93.51314583333334, 96.570375), Vector2(94.57187500000002, 99.44916666666668), Vector2(95.08339583333333, 102.47070833333335), Vector2(95.04770833333333, 105.635), Vector2(94.60042500000002, 108.61609583333335), Vector2(93.87715833333334, 111.08805000000002), Vector2(92.87790833333332, 113.05086250000001), Vector2(91.60267499999999, 114.50453333333334), Vector2(90.05145833333333, 115.4490625), Vector2(88.22425833333332, 115.88445000000002), Vector2(86.121075, 115.81069583333333), Vector2(83.74190833333334, 115.22780000000002), Vector2(81.08675833333335, 114.1357625), Vector2(78.155625, 112.53458333333333), Vector2(75.12575625000001, 111.05236250000002), Vector2(72.1744, 110.31720000000003), Vector2(69.30155624999999, 110.32909583333334), Vector2(66.507225, 111.08805000000001), Vector2(63.79140625, 112.59406249999999), Vector2(61.15409999999999, 114.84713333333335), Vector2(58.59530624999999, 117.84726250000001), Vector2(56.115025, 121.59445000000001), Vector2(53.71325624999999, 126.08869583333333), Vector2(51.39, 131.33)});

	while (true) {

		// Allow other tasks to run
		vex::this_thread::sleep_for(10);
	}
}