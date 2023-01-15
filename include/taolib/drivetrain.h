#pragma once

#include <cmath>
#include <vector>
#include <ratio>

#include "vex.h"

#include "vector2.h"
#include "pid.h"
#include "threading.h"

namespace tao {

/**
 * A structure describing values specific to the drivetrain's physical state. These values must be specifically tuned for each drivetrain.
 */
typedef struct {
	/** The  */
	PIDGains drive_gains;
	PIDGains turn_gains;
	double drive_tolerance;
	double turn_tolerance;
	double lookahead_distance;
	double track_width;
	double wheel_radius;
	double external_gear_ratio;
} DrivetrainProfile;

/**
 * A class representing a drivetrain with autonomous closed-loop PID control and odometry capabilities.
 */
class Drivetrain {
public:
	// Constructors

	/**
	 * Constructs a new Drivetrain object using two motor groups and an IMU.
	 * @param left_motors A reference to a vex::motor_group object representing the left side of the drivetrain.
	 * @param right_motors A reference to a vex::motor_group object representing the right side of the drivetrain.
	 * @param IMU A reference to a vex::inertial object for tracking the drivetrain's orientation through a gyro.
	 * @param profile A tao::DrivetrainProfile structure describing values related to the drivetrain for tuning.
	 */
	Drivetrain(
		vex::motor_group& left_motors,
		vex::motor_group& right_motors,
		vex::inertial& IMU,
		DrivetrainProfile profile
	);

	/**
	 * Constructs a new Drivetrain object using two motor groups.
	 * @param left_motors A reference to a vex::motor_group object representing the left side of the drivetrain.
	 * @param right_motors A reference to a vex::motor_group object representing the right side of the drivetrain.
	 * @param profile A tao::DrivetrainProfile structure describing values related to the drivetrain for tuning.
	 */
	Drivetrain(
		vex::motor_group& left_motors,
		vex::motor_group& right_motors,
		DrivetrainProfile profile
	);

	/**
	 * Constructs a new Drivetrain object using two motor groups, an IMU, and two tracking encoders.
	 * @param left_motors A reference to a vex::motor_group object representing the left side of the drivetrain.
	 * @param right_motors A reference to a vex::motor_group object representing the right side of the drivetrain.
	 * @param left_encoder A reference to a vex::encoder object representing the left tracking encoder.
	 * @param right_encoder A reference to a vex::encoder object representing the right tracking encoder.
	 * @param IMU A reference to a vex::inertial object for tracking the drivetrain's orientation through a gyro.
	 * @param profile A tao::DrivetrainProfile structure describing values related to the drivetrain for tuning.
	 */
	Drivetrain(
		vex::motor_group& left_motors,
		vex::motor_group& right_motors,
		vex::encoder& left_encoder,
		vex::encoder& right_encoder,
		vex::inertial& IMU,
		DrivetrainProfile profile
	);

	/**
	 * Constructs a new Drivetrain object using two motor groups and two tracking encoders.
	 * @param left_motors A reference to a vex::motor_group object representing the left side of the drivetrain.
	 * @param right_motors A reference to a vex::motor_group object representing the right side of the drivetrain.
	 * @param left_encoder A reference to a vex::encoder object representing the left tracking encoder.
	 * @param right_encoder A reference to a vex::encoder object representing the right tracking encoder.
	 * @param profile A tao::DrivetrainProfile structure describing values related to the drivetrain for tuning.
	 */
	Drivetrain(
		vex::motor_group& left_motors,
		vex::motor_group& right_motors,
		vex::encoder& left_encoder,
		vex::encoder& right_encoder,
		DrivetrainProfile profile
	);

	~Drivetrain();



	// Getters

	/**
	 * Gets the current global position of the drivetrain as a Vector2 object.
	 * @return The current global position of the drivetrain.
	 */
	Vector2 get_position() const;

	/**
	 * Gets the current counter-clockwise heading of the drivetrain in degrees.
	 * 
	 * If the IMU is not configured or installed, the heading will be calculated based on
	 * integrated encoders using only the drivetrain's track width measurements.
	 * @return The current heading of the drivetrain in degrees.
	 */
	double get_heading() const;

	/**
	 * Gets the average linear distance from the starting position traveled by each wheel.
	 * @return The average linear distance from the starting position traveled by each wheel.
	 */
	double get_drive_distance() const;

	/**
	 * Gets the current gain constants of the drive PID controller.
	 * @return The current gains as a PIDGains struct.
	 */
	PIDGains get_drive_gains() const;

	/**
	 * Gets the current gains of the turn PID controller.
	 * @return The current gains as a PIDGains struct.
	 */
	PIDGains get_turn_gains() const;

	/**
	 * Gets the current error of the drive PID controller.
	 * @return The current drive error (distance between the desired position and the current position).
	 */
	double get_drive_error() const;

	/**
	 * Gets the current error of the turn PID controller.
	 * @return The current turn error (distance in degrees between the desired position and the current position).
	 */
	double get_turn_error() const;

	/**
	 * Gets the minimum acceptable error threshold for the drive PID controller to consider its movements settled.
	 * @return The current minimum drive error.
	 */ 
	double get_drive_tolerance() const;

	/**
	 * Gets the minimum acceptable error threshold in degrees for the drive PID controller to consider its movements settled.
	 * @return The current minimum turn error.
	 */ 
	double get_turn_tolerance() const;

	/**
	 * Gets the lookahead distance used by the drivetrain's pure pursuit controller.
	 * @return The current lookahead radius
	 */
	double get_lookahead_distance() const;

	/**
	 * Gets the track width of the drivetrain.
	 * @return The distance between the left and right drivetrain wheels.
	 */
	double get_track_width() const;

	/**
	 * Gets the current external gear ratio of the drivetrain.
	 * @return The current external gear ratio as a quotient.
	 */
	double get_external_gear_ratio() const;

	/**
	 * Gets the current maximum velocity cap for forwards/backwards driving.
	 * @return The maximum forward/backward driving velocity.
	 */
	double get_max_drive_velocity() const;

	/**
	 * Gets the current maximum velocity cap for turning.
	 * @return The maximum forward/backward turingn velocity.
	 */
	double get_max_turn_velocity() const;

	/**
	 * Generates a tao::DrivetrainProfile structure from the current drivetrain state.
	 * @return The current drivetrain profile.
	 */
	DrivetrainProfile get_profile() const;

	/**
	 * Indicates if the drivetrain is currently settled (within the threshold of both minimum error ranges).
	 * @return True if the drivetrain is settled, false otherwise.
	 */
	bool is_settled() const;



	// Setters

	/**
	 * Sets the minimum acceptable error threshold for the drive PID controller to consider its movements settled.
	 * @param error The new minimum error threhold.
	 */
	void set_drive_tolerance(double error);
	
	/**
	 * Sets the minimum acceptable error threshold in degrees for the turn PID controller to consider its movements settled.
	 * @param error The new minimum error threhold in degrees.
	 */
	void set_turn_tolerance(double error);

	/**
	 * Sets the lookahead distance used by the drivetrain's pure pursuit controller.
	 * @param distance The new lookahead radius
	 */
	void set_lookahead_distance(double distance);

	/**
	 * Sets the gain constants for the drive PID controller.
	 * @param gains A PIDGains structure containing the new proportional, integral and derivative gain constants.
	 */
	void set_drive_gains(const PIDGains& gains);
	
	/**
	 * Sets the gain constants for the turn PID controller.
	 * @param gains A PIDGains structure containing the new proportional, integral and derivative gain constants.
	 */
	void set_turn_gains(const PIDGains& gains);
	
	/**
	 * Gets the current gear external ratio of the drivetrain.
	 * This could be useful for robots with a variable gear ratio (PTO) system.
	 * 
	 * @param ratio The new external gear ratio as a quotient.
	 */
	void set_external_gear_ratio(double ratio);
	
	/**
	 * Sets the maximum velocity cap for forwards/backwards driving.
	 * @param velocity A percentage of the new maximum velocity cap.
	 */
	void set_max_drive_velocity(double velocity);

	/**
	 * Sets the maximum velocity cap for turning..
	 * @param velocity A percentage of the new maximum velocity cap.
	 */
	void set_max_turn_velocity(double velocity);



	// Lifecycle functions

	/**
	 * Starts the tracking and PID threads for autonomous movement.
	 * @param start_position A 2D vector representing the starting cartesian coordinates of the drivetrain.
	 * @param start_heading An angle in degrees representing the starting orientation of the drivetrain.
	 */
	void setup_tracking(Vector2 start_position = Vector2(0, 0), double start_heading = 0, bool enable_logging = true);

	/**
	 * Resets the drive encoders and sets a new starting position for the drivetrain.
	 * @param start_position A 2D vector representing the starting cartesian coordinates of the drivetrain.
	 * @param start_heading An angle in degrees representing the starting orientation of the drivetrain.
	 */
	void reset_tracking(Vector2 start_position = Vector2(0, 0), double start_heading = 0);

	/** Stops all threads used for tracking and movement, restoring manual control of the drivetrain. */
	void stop_tracking();

	

	// Movement functions

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
	void turn_to(Vector2 point, bool blocking = true);

	/**
	 * Moves the drivetrain to a set of target coordinates.
	 * @param position A 2D vector representing the absolute target coordinates to move to.
	 * @param direction Determines the direction that the robot will face to move to the target. The auto direction will always take the most efficient turn.
	 * @param blocking Determines if the function should block the current thread until settled (within tolerance for 10 iterations).
	*/
	void move_to(Vector2 position, bool blocking = true);
	
	/**
	 * Moves the drivetrain along a set of waypoints using pure pursuit.
	 * @param path A vector of 2D vectors representing waypoints forming a path..
	*/
	void move_path(std::vector<Vector2> path);

	/**
	 * Stops and holds the drivetrain at the current position and heading.
	 * @param blocking Determines if the function should block the current thread until settled (within tolerance for 10 iterations).
	 */
	void hold_position(bool blocking = true);

private:
	enum class ErrorModes {
		Relative,
		Absolute
	};

	vex::motor_group& left_motors;
	vex::motor_group& right_motors;

	vex::encoder* left_encoder;
	vex::encoder* right_encoder;

	vex::inertial* IMU;
	
	Vector2 global_position;

	Vector2 target_position;
	double target_distance;
	double target_heading;

	double initial_heading;

	ErrorModes error_mode;

	double lookahead_distance;
	double track_width;
	double wheel_circumference;
	double external_gear_ratio;

	double start_heading = 0;

	double max_drive_velocity = 100;
	double max_turn_velocity = 100;

	double turn_tolerance;
	double drive_tolerance;

	double turn_error;
	double drive_error;

	bool settled = false;

	PIDController drive_controller;
	PIDController turn_controller;

	void set_target_position(Vector2 position);
	void set_target_distance(double distance);
	void set_target_heading(double heading);

	int daemon();
	int logging();
	
	bool daemon_active = false;
	bool logging_active = false;

	vex::thread daemon_thread;
	vex::thread logging_thread;
};

}