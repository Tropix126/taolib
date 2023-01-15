#include <cmath>

#include "taolib/vector2.h"

namespace tao {
	
Vector2::Vector2(double x, double y): x(x), y(y) {}
Vector2::Vector2(const Vector2 &v): x(v.x), y(v.y) {}
Vector2::Vector2(): x(0), y(0) {}

double Vector2::get_x() { return x; }
double Vector2::get_y() { return y; }

double Vector2::get_magnitude() {
	return sqrt(pow(x, 2) + pow(y, 2));
}

double Vector2::get_angle() {
	return atan2(y, x);
}

Vector2 Vector2::normalized() {
	return Vector2(x / get_magnitude(), y / get_magnitude());
}

Vector2 Vector2::rotated(double angle) {
	return Vector2(
		(x * cos(get_angle())) - (y * sin(get_angle())),
		(y * cos(get_angle())) + (x * sin(get_angle()))
	);
}

double Vector2::dot(const Vector2 &left, const Vector2 &right) {
	return (left.x * right.x) + (left.y * right.y);
}

double Vector2::cross(const Vector2 &v1, const Vector2 &v2) {
	return (v1.x * v2.y) - (v1.y * v2.x);
}

double Vector2::distance(const Vector2 &v1, const Vector2 &v2) {
	return sqrt(pow(v2.x - v1.x, 2) + pow(v2.y - v1.y, 2));
}

Vector2 operator+(const Vector2 &v1, const Vector2 &v2) {
	return Vector2(v1.x + v2.x, v1.y + v2.y);
}
Vector2 operator+(const Vector2 &v1, const double scalar) {
	return Vector2(v1.x + scalar, v1.y + scalar);
}

Vector2 operator-(const Vector2 &v1, const Vector2 &v2) {
	return Vector2(v1.x - v2.x, v1.y - v2.y);
}
Vector2 operator-(const Vector2 &v1, const double scalar) {
	return Vector2(v1.x - scalar, v1.y - scalar);
}

Vector2 operator*(const Vector2 &v1, const Vector2 &v2) {
	return Vector2(v1.x * v2.x, v1.y * v2.y);
}
Vector2 operator*(const Vector2 &v1, const double scalar) {
	return Vector2(v1.x * scalar, v1.y * scalar);
}

Vector2 operator/(const Vector2 &v1, const Vector2 &v2) {
	return Vector2(v1.x / v2.x, v1.y / v2.y);
}
Vector2 operator/(const Vector2 &v1, const double scalar) {
	return Vector2(v1.x / scalar, v1.y / scalar);
}

// Vector2 operator+=(const Vector2 &v1, const Vector2 &v2) {
// }
// Vector2 operator+=(const Vector2 &v1, const double scalar) {
// }

// Vector2 operator-=(const Vector2 &v1, const Vector2 &v2) {
// }
// Vector2 operator-=(const Vector2 &v1, const double scalar) {
// }

// Vector2 operator*=(const Vector2 &v1, const Vector2 &v2) {
// }
// Vector2 operator*=(const Vector2 &v1, const double scalar) {
// }

// Vector2 operator/=(const Vector2 &v1, const Vector2 &v2) {
// }
// Vector2 operator/=(const Vector2 &v1, const double scalar) {
// }

}