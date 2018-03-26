#ifndef QUATERNION_H_
#define QUATERNION_H_

#include <Eigen/Dense>

namespace utils{

	template<typename T>
	class Quaternion
	{
	public:
		Quaternion() : w_(1), x_(0), y_(0), z_(0)
		{};
		Quaternion(T w, T x, T y, T z) : w_(w), x_(x), y_(y), z_(z)
		{};
		// Quaternion(const Eigen::Vector3f& normal);
		~Quaternion(){};


	private:
		T w_;
		T x_;
		T y_;
		T z_;
	};

}//namespace utils



















#endif