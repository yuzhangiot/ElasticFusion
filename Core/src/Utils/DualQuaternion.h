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

        DualQuaternion(T x, T y, T z, T roll, T pitch, T yaw)
        {
            // convert here.
            rotation_.w_ = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) +
                           sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
            rotation_.x_ = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) -
                           cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
            rotation_.y_ = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) +
                           sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2);
            rotation_.z_ = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) -
                           sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2);

            translation_ = 0.5 * Quaternion<T>(0, x, y, z) * rotation_;
        }

        /**
         * \brief store a rotation
         * \param angle is in radians
         */
        void encodeRotation(T angle, T x, T y, T z)
        {
            rotation_.encodeRotation(angle, x, y, z);
        }
        /**
         * \brief store a rotation
         * \param angle is in radians
         */
        void encodeRotation(T x, T y, T z)
        {
            rotation_.encodeRotation(sqrt(x*x+y*y+z*z), x, y, z);
        }

        void encodeTranslation(T x, T y, T z)
        {
            translation_ = 0.5 * Quaternion<T>(0, x, y, z) * rotation_;
        }

        /// handle accumulating error.
        void normalize()
        {
            T x, y, z;
            getTranslation(x, y, z);

            rotation_.normalize();

            encodeTranslation(x, y, z);
        }

        /**
         * \brief a reference-based method for acquiring the latest
         *        translation data.
         */
        void getTranslation(T &x, T &y, T &z) const
        {
            Quaternion<T> result = getTranslation();
            /// note: inverse of a quaternion is the same as the conjugate.
            x = result.x_;
            y = result.y_;
            z = result.z_;
        }

        /**
         * \brief a reference-based method for acquiring the latest
         *        translation data.
         */
        void getTranslation(Eigen::Vector3f& vec3f) const
        {
            getTranslation(vec3f[0], vec3f[1], vec3f[2]);
        }

        Quaternion<T> getTranslation() const
        {
            auto rot = rotation_;
            rot.normalize();
            return 2 * translation_ * rot.conjugate();
        }


        /**
         * \brief a reference-based method for acquiring the latest rotation data.
         */
        void getEuler(T &roll, T &pitch, T &yaw)
        {
            // FIXME: breaks for some value around PI.
            roll = getRoll();
            pitch = getPitch();
            yaw = getYaw();
        }

        Quaternion<T> getRotation() const
        {
            return rotation_;
        }

        DualQuaternion operator+(const DualQuaternion &other)
        {
            DualQuaternion result;
            result.rotation_ = rotation_ + other.rotation_;
            result.translation_ = translation_ + other.translation_;
            return result;
        }

        DualQuaternion operator-(const DualQuaternion &other)
        {
            DualQuaternion result;
            result.rotation_ = rotation_ - other.rotation_;
            result.translation_ = translation_ - other.translation_;
            return result;
        }

        DualQuaternion operator*(const DualQuaternion &other)
        {
            DualQuaternion<T> result;
            result.rotation_ = rotation_ * other.rotation_;
//                result.translation_ = (rotation_ * other.translation_) + (translation_ * other.rotation_);
            result.translation_ = translation_ + other.translation_;
            return result;
        }

        DualQuaternion operator/(const std::pair<T,T> divisor)
        {
            DualQuaternion<T> result;
            result.rotation_ = 1 / divisor.first * rotation_;
            result.translation_ = 1 / divisor.second * translation_;
            return result;
        }


	private:
		Quaternion<T> rotation_;
		Quaternion<T> translation_;
		
		T position_[3] = {};    /// default initialize vector to zeros.

        T rotAxis_[3] = {};     /// default initialize vector to zeros.
        T rotAngle_;


        T getRoll()
        {
            // TODO: test this!
            return atan2(2*((rotation_.w_ * rotation_.x_) + (rotation_.y_ * rotation_.z_)),
                         (1 - 2*((rotation_.x_*rotation_.x_) + (rotation_.y_*rotation_.y_))));
        }

        T getPitch()
        {
            // TODO: test this!
            return asin(2*(rotation_.w_ * rotation_.y_ - rotation_.z_ * rotation_.x_));
        }

        T getYaw()
        {
            // TODO: test this!
            return atan2(2*((rotation_.w_ * rotation_.z_) + (rotation_.x_ * rotation_.y_)),
                         (1 - 2*((rotation_.y_*rotation_.y_) + (rotation_.z_*rotation_.z_))));
        }
	};

} //namespace utils












#endif