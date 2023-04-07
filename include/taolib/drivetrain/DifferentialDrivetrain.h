#pragma once

#include <vector>
#include <memory>
#include <cmath>
#include <functional>
#include <iostream>

#include "taolib/controller/MotionController.h"
#include "taolib/odometry/Odometry.h"
#include "taolib/utility/Vector2.h"
#include "taolib/utility/Logger.h"

#include "v5_cpp.h"
#include <memory>

template<typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args) {
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

namespace tao {

struct Config {
	std::unique_ptr<MotionController> drive_controller;
	std::unique_ptr<MotionController> turn_controller;
	double drive_tolerance;
	double turn_tolerance;
	double lookahead_distance;

	Config(
		const PIDController& drive_controller_,
		const PIDController& turn_controller_,
		double drive_tolerance_,
		double turn_tolerance_,
		double lookahead_distance_
	) :
		drive_controller(make_unique<PIDController>(drive_controller_)),
		turn_controller(make_unique<PIDController>(turn_controller_)),
		drive_tolerance(drive_tolerance_),
		turn_tolerance(turn_tolerance_),
		lookahead_distance(lookahead_distance_)
	{}
};

class DifferentialDrivetrain {
public:
    DifferentialDrivetrain(vex::motor_group& left_motors, vex::motor_group& right_motors, Odometry& odometry, const Config& config, const Logger& logger);
    DifferentialDrivetrain(vex::motor_group& left_motors, vex::motor_group& right_motors, Odometry& odometry, const Config& config);

	~DifferentialDrivetrain();

	double get_drive_error() const;
	double get_turn_error() const;

	double get_max_drive_power() const;
	double get_max_turn_power() const;
	void set_max_drive_power(double power);
	void set_max_turn_power(double power);

	double get_drive_tolerance() const;
	double get_turn_tolerance() const;
	void set_drive_tolerance(double tolerance);
	void set_turn_tolerance(double tolerance);

	double get_lookahead_distance() const;
	void set_lookahead_distance(double distance);

	const Logger& get_logger() const;
	MotionController& get_drive_controller() const;
	MotionController& get_turn_controller() const;
	Odometry& get_odometry() const;

	void enable();
	void disable();
	bool is_enabled() const;
	bool is_settled() const;

	/**
	 * Moves the drivetrain directly forwards or backwards along the x-axis.
	 * @param distance The distance that the drivetrain will move relative to it's current position.
	 * @param blocking Determines if the function should block the current thread until settled (within drive_tolerance for 10 iterations).
	 */
	void drive(double distance, bool blocking = true);

	/**
	 * Turns the drivetrain to an absolute heading.
	 * @param heading The angle in degrees to rotate the drivetrain to.
	 * @param blocking Determines if the function should block the current thread until settled (within turn_tolerance for 10 iterations).
	 */
	void turn_to(double heading, bool blocking = true);

	/**
	 * Turns the drivetrain face towards the direction of a cartesian coordinate.
	 * @param point A 2D vector representing the desired coordinates to face towards.
	 * @param blocking Determines if the function should block the current thread until settled (within turn_tolerance for 10 iterations).
	 */
	void turn_to(const Vector2& point, bool blocking = true);

	/**
	 * Moves the drivetrain to a set of target coordinates.
	 * @param position A 2D vector representing the absolute target coordinates to move to.
	 * @param direction Determines the direction that the robot will face to move to the target. The auto direction will always take the most efficient turn.
	 * @param blocking Determines if the function should block the current thread until settled (within drive tolerance for 10 iterations).
	*/
	void move_to(const Vector2& position, bool blocking = true);
	
	/**
	 * Moves the drivetrain along a set of waypoints using pure pursuit.
	 * @param path A vector of 2D vectors representing waypoints forming a path..
	*/
	void follow_path(const std::vector<Vector2>& path);

	/**
	 * Stops and holds the drivetrain at its current position and heading.
	 * @param blocking Determines if the function should block the current thread until settled (within tolerance for 10 iterations).
	 */
	void hold_position(bool blocking = true);

	/**
	 * Blocks the current thread until the drivetrain is settled.
	 */
	void wait_until_settled();

private:
	enum class MovementTypes {
		Relative,
		Absolute
	};

	vex::motor_group &left_motors_, &right_motors_;
	const Logger* logger_ = nullptr;
	MotionController &drive_controller_, &turn_controller_;
	Odometry& odometry_;

	MovementTypes movement_type_;

	double lookahead_distance_;
	double target_distance_, target_heading_, target_position_;
	double max_drive_power_, max_turn_power_;
	double drive_error_, turn_error_;
	double previous_drive_error_, previous_turn_error_;
	double drive_tolerance_, turn_tolerance_;

	bool settled_;
	bool enabled_;

	int daemon_(void* args);
	vex::thread daemon_thread_;
	bool daemon_thread_active_;
};

}