#ifndef QUATERNION_H_
#define QUATERNION_H_

#include <Eigen/Dense>

template<typename T>
class Quaternion
{
public:
	Quaternion();
	Quaternion(T w, T x, T y, T z);
	// Quaternion(const Eigen::Vector3f& normal);
	~Quaternion();


private:
	T w_;
	T x_;
	T y_;
	T z_;
};



















#endif