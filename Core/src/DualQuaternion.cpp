#include "DualQuaternion.h"

template<typename T>
DualQuaternion<T>::DualQuaternion() {
	rotation_ = Quaternion<float>();
	translation_ = Quaternion<float>();
}

template<typename T>
DualQuaternion<T>::~DualQuaternion() {

}