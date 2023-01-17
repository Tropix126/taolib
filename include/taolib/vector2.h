#pragma once

namespace tao {
/**
 * Class object to represent a vector within a 2 dimensional space
*/
class Vector2 {
public:
	/**
	 * Initializes the Vector2 class with preloaded x and y values
	 * @param x x value of the vector
	 * @param y y value of the vector
	*/
	Vector2(double x, double y);

	/**
	 * Initializes the Vector2 class with another vector
	 * @param v The other vector
	*/
	Vector2(const Vector2 &v);

	/**
	 * Initializes the Vector2 class with the default x and y values of 0.
	*/
	Vector2();

	/**
	 * Returns the x value of the vector
	*/
	double get_x() const;

	/**
	 * Returns the y value of the vector
	*/
	double get_y() const;

	/**
	 * Returns the magnitude of the vector (the vector's length, or distance from the origin)
	*/
	double get_magnitude() const;

	/**
	 * Returns the angle of the vector
	*/
	double get_angle() const;

	/**
	 * Normalize the vector (change the length of the vector to 1 while retaining the direction)
	 * @return Normalized version of the vector
	*/
	Vector2 normalized() const;

	/**
	 * Rotate the vector by a given angle in radians
	 * 
	 * @param angle The angle by which the vector should be rotated
	 * 
	 * @return Normalized version of the vector
	*/
	Vector2 rotated(double angle) const;

	/**
	 * Calculate the dot product of two vetors.
	 * @details The dot product is the product of the vector in the
	 * same length.
	 *
	 * @param other The other vector to perform the dot operation with.
	 *
	 * @return Vector result of the dot operation.
	 */
	double dot(const Vector2& other) const;
	
	/**
	 * Calculate the cross product of two vetors.
	 * 
	 * @param other The other vector to perform the cross operation with.
	 *
	 * @return Vector result of the cross operation.
	 */
	double cross(const Vector2& other) const;

	/**
	 * @brief Calculates the distance between two vectors.
	 *
	 * @param other The other vector to perform the distance calculation with.
	 *
	 * @return Vector result of the dot operation.
	 */
	double distance(const Vector2& other) const;

	double project(const Vector2& other) const;

	// Operators

	/**
	 * Scalar and vector addition
	 */
	friend Vector2 operator+(const Vector2& first, const Vector2& second);
	friend Vector2 operator+(const Vector2& first, double scalar);
	friend Vector2 operator+(double scalar, const Vector2& second);
	
	Vector2& operator+=(const Vector2& other);
	Vector2& operator+=(const double scalar);

	/**
	 * Scalar and vector subtraction
	 */
	friend Vector2 operator-(const Vector2& first, const Vector2& second);
	friend Vector2 operator-(const Vector2& first, double scalar);
	friend Vector2 operator-(double scalar, const Vector2& second);

	Vector2& operator-=(const Vector2& other);
	Vector2& operator-=(const double scalar);

	/**
	 * Scalar and vector multiplication
	 */
	friend Vector2 operator*(const Vector2& first, const Vector2& second);
	friend Vector2 operator*(const Vector2& first, double scalar);
	friend Vector2 operator*(const double scalar, const Vector2& second);

	Vector2& operator*=(const Vector2& other);
	Vector2& operator*=(const double scalar);

	/**
	 * Scalar and vector division
	 */
	friend Vector2 operator/(const Vector2& first, const Vector2& second);
	friend Vector2 operator/(const Vector2& first, double scalar);
	friend Vector2 operator/(double scalar, const Vector2& second);

	Vector2& operator/=(const Vector2& other);
	Vector2& operator/=(const double scalar);

	/**
	 * Vector comparison
	 */
	friend bool operator==(const Vector2& first, const Vector2& second);

private:
	/**
	 * The x value of the vector
	*/
	double x;

	/**
	 * The y value of the vector
	*/
	double y;
};
}