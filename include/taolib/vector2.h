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
	double get_x();

	/**
	 * Returns the y value of the vector
	*/
	double get_y();

	/**
	 * Returns the magnitude of the vector (the vector's length, or distance from the origin)
	*/
	double get_magnitude();

	/**
	 * Returns the angle of the vector
	*/
	double get_angle();

	/**
	 * Normalize the vector (change the length of the vector to 1 while retaining the direction)
	 * @return Normalized version of the vector
	*/
	Vector2 normalized();

	/**
	 * Rotate the vector by a given angle in radians
	 * 
	 * @param angle The angle by which the vector should be rotated
	 * 
	 * @return Normalized version of the vector
	*/
	Vector2 rotated(double angle);

	/**
	 * @brief Calculate the dot product of two vetors.
	 * @details The dot product is the product of the vector in the
	 * same length.
	 *
	 * @param left Left hand side of the dot operator.
	 * @param right Right hand side of the dot operator.
	 *
	 * @return Vector result of the dot operation.
	 */
	static double dot(const Vector2 &left, const Vector2 &right);
	
	/**
	 * Calculate the cross product of two vetors.
	 * 
	 * @param v1 The first vector.
	 * @param v2 The second vector.
	 *
	 * @return Vector result of the cross operation.
	 */
	static double cross(const Vector2 &v1, const Vector2 &v2);

	/**
	 * @brief Calculates the distance between two vectors.
	 *
	 * @param v1 The first vector.
	 * @param v2 The second vector.
	 *
	 * @return Vector result of the dot operation.
	 */
	static double distance(const Vector2 &v1, const Vector2 &v2);

	// Operators

	/**
	 * Scalar and vector addition
	*/
	friend Vector2 operator+(const Vector2 &v1, const Vector2 &v2);
	friend Vector2 operator+(const Vector2 &v1, double scalar);

	/**
	 * Scalar and vector subtraction
	*/
	friend Vector2 operator-(const Vector2 &v1, const Vector2 &v2);
	friend Vector2 operator-(const Vector2 &v1, double scalar);

	/**
	 * Scalar and vector multiplication
	*/
	friend Vector2 operator*(const Vector2 &v1, const Vector2 &v2);
	friend Vector2 operator*(const Vector2 &v1, double scalar);

	/**
	 * Scalar and vector division
	*/
	friend Vector2 operator/(const Vector2 &v1, const Vector2 &v2);
	friend Vector2 operator/(const Vector2 &v1, double scalar);

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