#ifndef DUAL_QUATERNION_H_
#define DUAL_QUATERNION_H_

#include <iostream>
#include "quaternion.h"

namespace utils {

	static float epsilon() {
		return 1e-6;
	}

	template<typename T>
	class DualQuaternion
	{
	public:
		DualQuaternion()
        {
            rotation_ = Quaternion<T>();
            translation_ = Quaternion<T>();
        };
        ~DualQuaternion(){};

        DualQuaternion(utils::Quaternion<T> translation, Quaternion<T> rotation)
        {
            rotation_ = rotation;
            translation_ = 0.5 * translation * rotation;
        }


	private:
		Quaternion<T> rotation_;
		Quaternion<T> translation_;
		
		T position_[3] = {};    /// default initialize vector to zeros.

        T rotAxis_[3] = {};     /// default initialize vector to zeros.
        T rotAngle_;


        // T getRoll()
        // {
        //     // TODO: test this!
        //     return atan2(2*((rotation_.w_ * rotation_.x_) + (rotation_.y_ * rotation_.z_)),
        //                  (1 - 2*((rotation_.x_*rotation_.x_) + (rotation_.y_*rotation_.y_))));
        // }

        // T getPitch()
        // {
        //     // TODO: test this!
        //     return asin(2*(rotation_.w_ * rotation_.y_ - rotation_.z_ * rotation_.x_));
        // }

        // T getYaw()
        // {
        //     // TODO: test this!
        //     return atan2(2*((rotation_.w_ * rotation_.z_) + (rotation_.x_ * rotation_.y_)),
        //                  (1 - 2*((rotation_.y_*rotation_.y_) + (rotation_.z_*rotation_.z_))));
        // }
	};

} //namespace utils












#endif