/**
 * @file src/taolib/math.h
 * @author Tropical
 *
 * Various helper functions and variables used in mathematical operations.
 */

#pragma once

#include "Vector2.h"
#include <vector>

namespace tao {
namespace math {

constexpr double PI = 3.14159265358979323846;

double vector_average(std::vector<double> const& v);

/**
 * Converts a number representing an angle in degrees and to an equivalent angle between -179 and 180 degrees.
 *
 * @param degrees The initial angle in degrees.
 * @return The equivalent normalized angle between -170 and 180 degrees.
 */
double normalize_degrees(double degrees);

/**
 * Given a left, right, and maximum speed, calculate a new set of speed values that preserve the
 * ratio between left and right if one of the two values is over the maximum threshold.
 * 
 * @param left_speed The uncapped left speed.
 * @param right_speed The uncapped right speed.
 * @param max_speed The maximum speed that can be reached by a motor.
 * 
 * @return A pair of speed values {left_speed, right_speed} that is correctly scaled down if one
 * of the two values exceeds max_speed.
*/
std::pair<double, double> normalize_speeds(double left_speed, double right_speed, double max_speed);

/**
 * Converts an angle in degrees to an angle in radians.
 *
 * @param degrees The angle in degrees.
 * @return The angle in radians.
 */
constexpr double to_radians(double degrees) {
	return degrees * (PI / 180.0);
}

/**
 * Converts an angle in radians to an angle in degrees.
 *
 * @param radians The angle in radians.
 * @return The angle in degrees.
 */
constexpr double to_degrees(double radians) {
	return radians * (180.0 / PI);
}

/**
 * Restricts a number between a minimum and maximum value.
 *
 * @param value The number to restrict.
 * @param min The minimum value that the input will be restricted to.
 * @param maximum The maximum value that the input will be restricted to.
 * 
 * @return The input value restricted between [min, max].
 */
template <typename T>
constexpr T clamp(T value, T min, T max) {
	return std::min(max, std::max(min, value));
}

/**
 * Determines whether an input number is positive or negative.
 *
 * @param value The input number to be checked.
 * 
 * @return -1 if the number is negative, 1 if it is positive.
 */
template <typename T>
constexpr T sign(T value) {
	return value < 0 ? -1 : 1;
}

/**
 * Finds the point(s) of intersection between a line segment and a circle.
 *
 * @param center The location of the circle's center point.
 * @param radius The radius of the circle.
 * @param point_1 The starting point of the line segment.
 * @param point_2 The endpoint of the line segment.
 * 
 * @return 0-2 Vector2 instances representing an intersection between the line segment and the circle.
 */
std::vector<Vector2> line_circle_intersections(Vector2 center, double radius, Vector2 point_1, Vector2 point_2);

} // namespace math
} // namespace tao