#include "quaternion.h"


template<typename T>
Quaternion<T>::Quaternion():
w_(1),
x_(0),
y_(0),
z_(0)
{

}

template<typename T>
Quaternion<T>::Quaternion(T w, T x, T y, T z):
w_(w),
x_(x),
y_(y),
z_(z)
{

}

// Quaternion::Quaternion(const Eigen::Vector3f& normal) {
// 	Eigen::Vector3f a(1, 0, 0);
// 	Eigen::Vector3f b(0, 1, 0);

// 	Eigen::Vector3f t0 = normal.cross(a);

// 	if (t0.dot(t0) < 0.001f)
// 	    t0 = normal.cross(b);
// 	t0 = t0.normalized();

// 	Eigen::Vector3f t1 = normal.cross(t0);
// 	t1 = t1.normalized();

// 	Eigen::Matrix3f matrix;
// 	matrix.push_back(t0);
// 	matrix.push_back(t1);
// 	matrix.push_back(normal);
// 	w_ = sqrt(1.0 + matrix.at<float>(0,0) + matrix.at<float>(1,1) + matrix.at<float>(2,2)) / 2.0;
// 	//                FIXME: this breaks when w_ = 0;
// 	x_ = (matrix.at<float>(2,1) - matrix.at<float>(1,2)) / (w_ * 4);
// 	y_ = (matrix.at<float>(0,2) - matrix.at<float>(2,0)) / (w_ * 4);
// 	z_ = (matrix.at<float>(1,0) - matrix.at<float>(2,1)) / (w_ * 4);
// 	if(norm() > 0)
// 	    normalize();
// }

template<typename T>
Quaternion<T>::~Quaternion() {

}