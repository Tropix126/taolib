#include "vex.h"
#include "vector2.h"
#include <vector>

namespace tao::math {

constexpr double PI = 3.14159265358979323846;

/**
 * Converts a number representing an angle in degrees and to an equivalent angle between -179 and 180 degrees.
 *
 * @param degrees The initial angle in degrees.
 * @return The equivalent normalized angle between -170 and 180 degrees.
 */
double normalize_degrees(double degrees);

/**
 * Converts an angle in degrees to an angle in radians.
 *
 * @param degrees The angle in degrees.
 * @return The angle in radians.
 */
constexpr double degrees_to_radians(double degrees) {
	return degrees * (PI / 180);
}

/**
 * Converts an angle in radians to an angle in degrees.
 *
 * @param radians The angle in radians.
 * @return The angle in degrees.
 */
constexpr double radians_to_degrees(double radians) {
	return radians * (180 / PI);
}

constexpr double clamp(double x, double min, double max) {
	return (x < min) ? min : (x > max) ? max : x;
}

constexpr double sign(double x) {
	return x < 0 ? -1 : 1;
}

std::vector<Vector2> line_circle_intersections(Vector2 center, Vector2 point1, Vector2 point2, double radius);


}