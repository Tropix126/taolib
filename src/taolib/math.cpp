/**
 * @file src/taolib/math.cpp
 * @author Tropical
 *
 * Various helper functions and variables used in mathematical operations.
 */


#include <cmath>
#include <vector>
#include <numeric>
#include <algorithm>

#include "taolib/math.h"
#include "taolib/vector2.h"

namespace tao {
namespace math {

double normalize_degrees(double degrees) {
	return std::remainder(degrees, 360.0);
}

std::pair<double, double> normalize_speeds(double left_speed, double right_speed, double max_speed) {
	double largest_speed = std::max(std::abs(left_speed), std::abs(right_speed)) / max_speed;

	if (largest_speed > 1.0) {
		left_speed /= largest_speed;
		right_speed /= largest_speed;
	}

	return { left_speed, right_speed };
}

std::vector<Vector2> line_circle_intersections(Vector2 center, double radius, Vector2 point_1, Vector2 point_2) {
	std::vector<Vector2> intersections = {};

	// Subtract the circle's center to offset the system to origin.
	Vector2 offset_1 = point_1 - center;
	Vector2 offset_2 = point_2 - center;

	double dx = (offset_2 - offset_1).get_x();
	double dy = (offset_2 - offset_1).get_y();
	double dr = offset_1.distance(offset_2);
	double D = offset_1.cross(offset_2);
	double discriminant = pow(radius, 2) * pow(dr, 2) - pow(D, 2);

	// If our discriminant is greater than or equal to 0, the line formed as a slope of
	// point_1 and point_2 intersects the circle at least once.
	if (discriminant >= 0) {
		// https://mathworld.wolfram.com/Circle-LineIntersection.html
		Vector2 solution_1 = Vector2(
			(D * dy + sign(dy) * dx * std::sqrt(discriminant)) / std::pow(dr, 2),
			(-D * dx + fabs(dy) * std::sqrt(discriminant)) / std::pow(dr, 2)
		) + center;
		Vector2 solution_2 = Vector2(
			(D * dy - sign(dy) * dx * std::sqrt(discriminant)) / std::pow(dr, 2),
			(-D * dx - fabs(dy) * std::sqrt(discriminant)) / std::pow(dr, 2)
		) + center;

		double min_x = std::min(point_1.get_x(), point_2.get_x());
		double max_x = std::max(point_1.get_x(), point_2.get_x());
		double min_y = std::min(point_1.get_y(), point_2.get_y());
		double max_y = std::max(point_1.get_y(), point_2.get_y());

		// Find the bounded intersections.
		// solution_1 and solution_2 are assumed to be true when the line formed as a slope between point_1 and point_2
		// extends infinitely, however we only want to consider intersections that are part of a line segment *between*
		// point_1 and point_2.

		// Solution 1 intersects the circle within the bounds of point_1 and point_2
		if ((solution_1.get_x() >= min_x && solution_1.get_x() <= max_x) && (solution_1.get_y() >= min_y && solution_1.get_y() <= max_y)) {
			intersections.push_back(solution_1);
		}

		// Solution 2 intersects the circle within the bounds of point_1 and point_2
		if ((solution_2.get_x() >= min_x && solution_2.get_x() <= max_x) && (solution_2.get_y() >= min_y && solution_2.get_y() <= max_y)) {
			intersections.push_back(solution_2);
		}
	}

	return intersections;
}

} // namespace math
} // namespace tao