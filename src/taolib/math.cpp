/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       math.cpp                                                  */
/*    Author:       Tropical                                                  */
/*    Created:      Mon Oct 25 2022                                           */
/*    Description:                                                            */
/*                                                                            */
/*----------------------------------------------------------------------------*/


#include <cmath>
#include <vector>
#include <numeric>
#include <algorithm>
#include <math.h>

#include "taolib/math.h"
#include "taolib/vector2.h"

namespace tao::math {

double normalize_degrees(double degrees) {
	degrees = fmod(degrees, 360);
	degrees = fmod((degrees + 360), 360);

	if (degrees > 180) {
		degrees -= 360;
	}

	return degrees > 180 ? degrees - 360 : degrees;
}

std::vector<Vector2> line_circle_intersections(Vector2 center, Vector2 point_1, Vector2 point_2, double radius) {
	std::vector<Vector2> intersections = {};

	// Subtract the circle's center to offset the system to origin.
	Vector2 offset_1 = point_1 - center;
	Vector2 offset_2 = point_2 - center;

	double dx = (offset_2 - offset_1).get_x();
	double dy = (offset_2 - offset_1).get_y();
	double dr = Vector2::distance(offset_1, offset_2);
	double D = Vector2::cross(offset_1, offset_2);
	double discriminant = pow(radius, 2) * pow(dr, 2) - pow(D, 2);

	// The line formed as a slope of point_1 and point_2 intersects the circle at least once.
	if (discriminant >= 0) {
		// https://mathworld.wolfram.com/Circle-LineIntersection.html
		double solution_x1 = (D * dy + sign(dy) * dx * sqrt(discriminant)) / pow(dr, 2);
		double solution_x2 = (D * dy - sign(dy) * dx * sqrt(discriminant)) / pow(dr, 2);
		double solution_y1 = (-D * dx + fabs(dy) * sqrt(discriminant)) / pow(dr, 2);
		double solution_y2 = (-D * dx - fabs(dy) * sqrt(discriminant)) / pow(dr, 2);

		// Add the center of the circle back to the solutions, offset the system back to its original position.
		Vector2 solution_1 = Vector2(solution_x1, solution_y1) + center;
		Vector2 solution_2 = Vector2(solution_x2, solution_y2) + center;

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

}