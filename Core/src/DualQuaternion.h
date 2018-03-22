#ifndef DUAL_QUATERNION_H_
#define DUAL_QUATERNION_H_

#include <iostream>
#include "Utils/quaternion.h"

static float epsilon() {
	return 1e-6;
}

template<typename T>
class DualQuaternion
{
public:
	DualQuaternion();
	~DualQuaternion();


private:
	Quaternion<T> rotation_;
	Quaternion<T> translation_;
	
};












#endif