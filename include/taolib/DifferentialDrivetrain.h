/**
 * @file src/taolib/drivetrain.cpp
 * @author Tropical
 *
 * Abstracts various position tracking and motion control algorithms to
 * provide various functions for controlling a physical drivetrain.
 * Given appropriate devices, and a model (called a "config") for
 * tuning certain physical aspects unique to each robot, the
 * tao::DifferentialDrivetrain class can perform various autonomous actions.
 */

#pragma once

#include <cmath>
#include <vector>
#include <ratio>
#include <memory>

#include "env.h"

#include "Vector2.h"
#include "PIDController.h"
#include "threading.h"
#include "Logger.h"

namespace tao {

/**
 * A class representing a nonholonomic drivetrain using position tracking and PID motion control.
 */
class DifferentialDrivetrain {
public:
	/**
	 * A structure describing values specific to the drivetrain's physical state.
	 * @attention These values are unique to each drivetrain and must be specifically tuned.
	 */
	typedef struct {
		/** The PID tuning constants used by the drive velocity PID controller. */
		PIDController::Gains drive_gains;

		/** The PID tuning constants used by the turn velocity PID controller. */
		PIDController::Gains turn_gains;

		/** The minimum acceptable error threshold (in arbitrary distance units) for the drive PID controller to consider its movement settled. */
		double drive_tolerance;

		/** The minimum acceptable error threshold (in degrees) for the turn PID controller to consider its movement settled. */
		double turn_tolerance;

		/** The radius that the robot will use to find lookahead points when following a curve using pure pursuit. */
		double lookahead_distance;

		/**
		 * The distance between the left and right drivetrain wheels.
		 * @note If an imu is unavailable (unplugged or not provided), this number will be used for calculating the drivetrain's absolute heading.
		 */
		double track_width;

		/**
		 * The radius of the drivetrain's wheels. This measurement will determine the units used for movement.
		 * @attention If using external encoders, this measurement should be the radius of the drivetrain's tracking wheels. Otherwise, these measurements should be the radius of the drivetrain's powered wheels.
		 */
		double wheel_diameter;

		/**
		 * The external gear ratio of the robot as a quotient (INPUT TEETH / OUTPUT TEETH).
		 */
		double gearing;
	} Config;

	// Constructors

	/**
	 * Constructs a new DifferentialDrivetrain object using two motor groups and an imu.
	 * @param left_motors A reference to a env::MotorGroup object representing the left side of the drivetrain.
	 * @param right_motors A reference to a env::MotorGroup object representing the right side of the drivetrain.
	 * @param imu A reference to a env::IMU object for tracking the drivetrain's orientation through a imu.
	 * @param config A DifferentialDrivetrain::Config structure describing values related to the drivetrain for tuning.
	 */
	DifferentialDrivetrain(
		env::MotorGroup& left_motors,
		env::MotorGroup& right_motors,
		env::IMU& imu,
		Config config,
		Logger logger = tao::Logger()
	);

	/**
	 * Constructs a new DifferentialDrivetrain object using two motor groups.
	 * @param left_motors A reference to a env::MotorGroup object representing the left side of the drivetrain.
	 * @param right_motors A reference to a env::MotorGroup object representing the right side of the drivetrain.
	 * @param config A DifferentialDrivetrain::Config structure describing values related to the drivetrain for tuning.
	 */
	DifferentialDrivetrain(
		env::MotorGroup& left_motors,
		env::MotorGroup& right_motors,
		Config config,
		Logger logger = tao::Logger()
	);

	/**
	 * Constructs a new DifferentialDrivetrain object using two motor groups, an imu, and two tracking encoders.
	 * @param left_motors A reference to a env::MotorGroup object representing the left side of the drivetrain.
	 * @param right_motors A reference to a env::MotorGroup object representing the right side of the drivetrain.
	 * @param left_encoder A reference to a env::Encoder object representing the left tracking encoder.
	 * @param right_encoder A reference to a env::Encoder object representing the right tracking encoder.
	 * @param imu A reference to a env::IMU object for tracking the drivetrain's orientation through a imu.
	 * @param config A DifferentialDrivetrain::Config structure describing values related to the drivetrain for tuning.
	 */
	DifferentialDrivetrain(
		env::MotorGroup& left_motors,
		env::MotorGroup& right_motors,
		env::Encoder& left_encoder,
		env::Encoder& right_encoder,
		env::IMU& imu,
		Config config,
		Logger logger = tao::Logger()
	);

	/**
	 * Constructs a new DifferentialDrivetrain object using two motor groups and two tracking encoders.
	 * @param left_motors A reference to a env::MotorGroup object representing the left side of the drivetrain.
	 * @param right_motors A reference to a env::MotorGroup object representing the right side of the drivetrain.
	 * @param left_encoder A reference to a env::Encoder object representing the left tracking encoder.
	 * @param right_encoder A reference to a env::Encoder object representing the right tracking encoder.
	 * @param config A DifferentialDrivetrain::Config structure describing values related to the drivetrain for tuning.
	 */
	DifferentialDrivetrain(
		env::MotorGroup& left_motors,
		env::MotorGroup& right_motors,
		env::Encoder& left_encoder,
		env::Encoder& right_encoder,
		Config config,
		Logger logger = tao::Logger()
	);

	~DifferentialDrivetrain();

	// Getters

	/**
	 * Gets the current global position of the drivetrain as a Vector2 object.
	 * @return The current global position of the drivetrain.
	 */
	Vector2 get_position();

	/**
	 * Gets the average wheel travel distance of each side of the drivetrain.
	 * @return A pair representing the total distance traveled by the left and right wheels of the drivetrain.
	 */
	std::pair<double, double> get_wheel_travel() const;

	double get_forward_travel() const;

	/**
	 * Gets the current counter-clockwise heading of the drivetrain in degrees.
	 * @note If the imu is not configured or installed, the heading will be calculated based on encoders only, using the drivetrain's track width measurements.
	 * @return The current heading of the drivetrain in degrees.
	 */
	double get_heading();

	/**
	 * Gets the current gain constants of the drive PID controller.
	 * @return The current gains as a PIDController::Gains struct.
	 */
	PIDController::Gains get_drive_gains() const;

	/**
	 * Gets the current gains of the turn PID controller.
	 * @return The current gains as a PIDController::Gains struct.
	 */
	PIDController::Gains get_turn_gains() const;

	/**
	 * Gets the current error of the drive PID controller.
	 * @return The current drive error (distance between the desired position and the current position).
	 */
	double get_drive_error();

	/**
	 * Gets the current error of the turn PID controller.
	 * @return The current turn error (distance in degrees between the desired position and the current position).
	 */
	double get_turn_error();

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
	double get_gearing() const;

	/**
	 * Gets the current maximum velocity cap for forwards/backwards driving.
	 * @return The maximum forward/backward driving velocity.
	 */
	double get_max_drive_power() const;

	/**
	 * Gets the current maximum velocity cap for turning.
	 * @return The maximum forward/backward turingn velocity.
	 */
	double get_max_turn_power() const;

	/**
	 * Generates a DifferentialDrivetrain::Config structure from the current drivetrain state.
	 * @return The current drivetrain config.
	 */
	Config get_config() const;

	/**
	 * Indicates if the drivetrain is currently settled (within the threshold of both minimum error ranges).
	 * @return True if the drivetrain is settled, false otherwise.
	 */
	bool is_settled();

	/**
	 * Gets the wheel diameter of the drivetrain
	 * @return The wheel diameter of the drivetrain.
	 */
	double get_wheel_diameter() const;

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
	 * @param gains A PIDController::Gains structure containing the new proportional, integral and derivative gain constants.
	 */
	void set_drive_gains(const PIDController::Gains& gains);
	
	/**
	 * Sets the gain constants for the turn PID controller.
	 * @param gains A PIDController::Gains structure containing the new proportional, integral and derivative gain constants.
	 */
	void set_turn_gains(const PIDController::Gains& gains);
	
	/**
	 * Gets the current gear external ratio of the drivetrain.
	 * @note This could be useful for robots with a variable gear ratio (PTO) system.
	 * 
	 * @param ratio The new external gear ratio as a quotient.
	 */
	void set_gearing(double ratio);
	
	/**
	 * Sets the maximum velocity cap for forwards/backwards driving.
	 * @param power A percentage of the new maximum velocity cap.
	 */
	void set_max_drive_power(double power);

	/**
	 * Sets the maximum velocity cap for turning..
	 * @param power A percentage of the new maximum velocity cap.
	 */
	void set_max_turn_power(double power);

	/**
	 * Gets the wheel diameter of the drivetrain
	 * @return The wheel diameter of the drivetrain.
	 */
	void set_wheel_diameter(double diameter);

	// Lifecycle functions

	/**
	 * Starts the tracking period for autonomous movement.
	 * @param position A 2D vector representing the starting cartesian coordinates of the drivetrain.
	 * @param heading An angle in degrees representing the starting orientation of the drivetrain.
	 */
	void start_tracking(Vector2 position = Vector2(0.0, 0.0), double heading = 90.0);

	/** Stops all threads used for tracking and movement, restoring manual control of the drivetrain. */
	void stop_tracking();

	/**
	 * Resets the tracking period's localization data (position/heading).
	 * @param position A 2D vector representing the starting cartesian coordinates of the drivetrain.
	 * @param heading An angle in degrees representing the starting orientation of the drivetrain.
	 */
	void reset_tracking(Vector2 position = Vector2(0.0, 0.0), double heading = 90.0);

	/**
	 * Calibrates the inertial sensor associated with this drivetrain, if available.
	 * Assuming an imu is available and installed, this function will block for up to
	 * 3600ms (3.6 seconds).
	 * 
	 * This function should be called while the robot is entirely stationary, ideally
	 * before the autonomous period starts. Physical disturbances or vibration during
	 * calibration may cause the sensor to drift. 
	 */
	void calibrate_imu();

	/**
	 * Blocks the current thread until the drivetrain is settled, or until a timeout is exceeded. 
	 * @param timeout The maximum amount of time to block the current thread in milliseconds, regardless of if the drivetrain settles or not.
	 */
	void wait_until_settled();

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
	 * Turns the drivetrain face towards the direction of a point.
	 * @param point A 2D vector representing the desired coordinates to face towards.
	 * @param blocking Determines if the function should block the current thread until settled (within turn_tolerance for 10 iterations).
	 */
	void turn_to(Vector2 point, bool blocking = true);

	/**
	 * Moves the drivetrain to a target point.
	 * @param point A 2D vector representing the absolute target coordinates to move to.
	 * @param blocking Determines if the function should block the current thread until settled (within drive tolerance for 10 iterations).
	*/
	void move_to(Vector2 point, bool blocking = true);
	
	/**
	 * Moves the drivetrain along a set of path waypoints.
	 * @param path A vector of 2D vectors representing waypoints forming a path..
	*/
	void follow_path(std::vector<Vector2> path);

	/**
	 * Stops and holds the drivetrain at its current position and heading.
	 */
	void hold_position();

private:
	enum class TargetType {
		DistanceAndHeading,
		Point
	};

	env::MotorGroup &left_motors, &right_motors;
	env::Encoder *left_encoder, *right_encoder;
	env::IMU* imu;
	
	Vector2 position;

	Vector2 target_position;
	double target_distance, target_heading;
	double start_heading;

	TargetType target_type;
	
	double max_drive_power = 100, max_turn_power = 100;
	double drive_tolerance, turn_tolerance;
	double drive_error, turn_error;

	double lookahead_distance;
	double track_width;
	double wheel_diameter;
	double gearing;

	bool settled = false;
	bool imu_calibrated = false;
	bool imu_invalid = false;

	PIDController drive_controller, turn_controller;
	Logger logger;

	void set_target(Vector2 position);
	void set_target(double distance, double heading);

	int tracking();
	int logging();
	
	bool tracking_active = false;
	bool logging_active = false;

	std::shared_ptr<env::Thread> tracking_thread, logging_thread;
	env::Mutex mutex;
};

} // namespace tao
