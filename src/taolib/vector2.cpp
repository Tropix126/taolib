/**
 * @file src/taolib/pid.cpp
 * @author Tropical
 *
 * Implementation for a 2-dimensional vector stored as cartesian coordinates.
 */

#include <cmath>

#include "taolib/vector2.h"

namespace tao {

Vector2::Vector2(double x, double y): x(x), y(y) {}
Vector2::Vector2(const Vector2& v): x(v.x), y(v.y) {}
Vector2::Vector2(): x(0), y(0) {}

double Vector2::get_x() const { return x; }
double Vector2::get_y() const { return y; }

double Vector2::get_magnitude() const {
	return std::sqrt(std::pow(x, 2) + std::pow(y, 2));
}

double Vector2::get_angle() const {
	return std::atan2(y, x);
}

Vector2 Vector2::normalized() const {
	return *this / get_magnitude();
}

Vector2 Vector2::rotated(double angle) const {
	return Vector2(
		(x * cos(angle)) - (y * sin(angle)),
		(x * sin(angle)) + (y * cos(angle))
	);
}

double Vector2::dot(const Vector2& other) const {
	return (x * other.y) + (y * other.x);
}

double Vector2::cross(const Vector2& other) const {
	return (x * other.y) - (y * other.x);
}

double Vector2::distance(const Vector2& other) const {
	return (*this - other).get_magnitude();
}

double Vector2::project(const Vector2& other) const {
	return dot(other) / other.get_magnitude();
}

Vector2 operator+(const Vector2& first, const Vector2& second) {
	return { first.x + second.x, first.y + second.y };
}
Vector2 operator+(const Vector2& first, const double scalar) {
	return { first.x + scalar, first.y + scalar };
}
Vector2 operator+(const double scalar, const Vector2& second) {
	return { scalar + second.x, scalar + second.y };
}

Vector2 operator-(const Vector2& first, const Vector2& second) {
	return { first.x - second.x, first.y - second.y };
}
Vector2 operator-(const Vector2& first, const double scalar) {
	return { first.x - scalar, first.y - scalar };
}
Vector2 operator-(const double scalar, const Vector2& second) {
	return { scalar - second.x, scalar - second.y };
}

Vector2 operator*(const Vector2& first, const Vector2& second) {
	return { first.x * second.x, first.y * second.y };
}
Vector2 operator*(const Vector2& first, const double scalar) {
	return { first.x * scalar, first.y * scalar };
}
Vector2 operator*(const double scalar, const Vector2& second) {
	return { scalar * second.x, scalar * second.y };
}

Vector2 operator/(const Vector2& first, const Vector2& second) {
	return { first.x / second.x, first.y / second.y };
}
Vector2 operator/(const Vector2& first, const double scalar) {
	return { first.x / scalar, first.y / scalar };
}
Vector2 operator/(const double scalar, const Vector2& second) {
	return { scalar / second.x, scalar / second.y };
}

Vector2& Vector2::operator+=(const Vector2& other) {
	x += other.x;
	y += other.y;
	return *this;
}
Vector2& Vector2::operator+=(const double scalar) {
	x += scalar;
	y += scalar;
	return *this;
}

Vector2& Vector2::operator-=(const Vector2& other) {
	x -= other.x;
	y -= other.y;
	return *this;
}
Vector2& Vector2::operator-=(const double scalar) {
	x -= scalar;
	y -= scalar;
	return *this;
}

Vector2& Vector2::operator*=(const Vector2& other) {
	x *= other.x;
	y *= other.y;
	return *this;
}
Vector2& Vector2::operator*=(const double scalar) {
	x *= scalar;
	y *= scalar;
	return *this;
}

Vector2& Vector2::operator/=(const Vector2& other) {
	if(other.x != 0 && other.y != 0){
		x /= other.x;
		y /= other.y;
	}

	return *this;
}
Vector2& Vector2::operator/=(const double scalar) {
	x /= scalar;
	y /= scalar;
	return *this;
}

bool operator==(const Vector2& first, const Vector2& second) {
	return (first.x == second.x) && (first.y == second.y);
}
}
