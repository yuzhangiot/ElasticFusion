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

		T norm()
        {
            return sqrt((w_ * w_) + (x_ * x_) + (y_ * y_) + (z_ * z_));
        }
		void normalize()
        {
            // should never happen unless the Quaternion<T> wasn't initialized
            // correctly.
            assert( !((w_ == 0) && (x_ == 0) && (y_ == 0) && (z_ == 0)));
            T theNorm = norm();
            assert(theNorm > 0);
            (*this) = (1.0/theNorm) * (*this);
        }


		void encodeRotation(T theta, T x, T y, T z)
        {
            auto sin_half = sin(theta / 2);
            w_ = cos(theta / 2);
            x_ = x * sin_half;
            y_ = y * sin_half;
            z_ = z * sin_half;
            normalize();
        }

        void getRodrigues(T& x, T& y, T& z)
        {
            if(w_ == 1)
            {
                x = y = z = 0;
                return;
            }
            T half_theta = acos(w_);
            T k = sin(half_theta) * tan(half_theta);
            x = x_ / k;
            y = y_ / k;
            z = z_ / k;
        }

        Quaternion conjugate() const
        {
            return Quaternion<T>(w_, -x_, -y_, -z_);
        }

        void rotate(T& x, T& y, T& z)
        {
            Quaternion<T> q = (*this);
            Quaternion<T> qStar = (*this).conjugate();
            Quaternion<T> rotatedVal = q * Quaternion(0, x, y, z) * qStar;

            x = rotatedVal.x_;
            y = rotatedVal.y_;
            z = rotatedVal.z_;
        }

        void rotate(Eigen::Vector3f& v) const
        {
            auto rot= *this;
            rot.normalize();
            Eigen::Vector3f q_vec(rot.x_, rot.y_, rot.z_);
            v += (q_vec*2.f).cross( q_vec.cross(v) + v*rot.w_ );
        }

        Quaternion operator+(const Quaternion& other)
        {
            return Quaternion(  (w_ + other.w_),
                                (x_ + other.x_),
                                (y_ + other.y_),
                                (z_ + other.z_));
        }
        void operator+=(const Quaternion& other)
        {
            *this = *this + other;
        }

        Quaternion operator-(const Quaternion& other)
        {
            return Quaternion((w_ - other.w_),
                              (x_ - other.x_),
                              (y_ - other.y_),
                              (z_ - other.z_));
        }

        Quaternion operator-()
        {
            return Quaternion(-w_, -x_, -y_, -z_);
        }

        bool operator==(const Quaternion& other) const
        {
            return (w_ == other.w_) && (x_ == other.x_) && (y_ == other.y_) && (z_ == other.z_);
        }

        template <typename U> friend Quaternion operator*(const U scalar, const Quaternion& other)
        {
            return Quaternion<T>((scalar * other.w_),
                                 (scalar * other.x_),
                                 (scalar * other.y_),
                                 (scalar * other.z_));
        }

        template <typename U> friend Quaternion operator/(const Quaternion& q, const U scalar)
        {
            return (1 / scalar) * q;
        }

        /// Quaternion Product
        Quaternion operator*(const Quaternion& other)
        {
            return Quaternion(
                    ((w_*other.w_) - (x_*other.x_) - (y_*other.y_) - (z_*other.z_)),
                    ((w_*other.x_) + (x_*other.w_) + (y_*other.z_) - (z_*other.y_)),
                    ((w_*other.y_) - (x_*other.z_) + (y_*other.w_) + (z_*other.x_)),
                    ((w_*other.z_) + (x_*other.y_) - (y_*other.x_) + (z_*other.w_))
            );
        }

        T dotProduct(Quaternion other)
        {
            return 0.5 * ((conjugate() * other) + (*this) * other.conjugate()).w_;
        }

        template <typename U> friend std::ostream& operator << (std::ostream& os, const Quaternion<U>& q)
        {
            os << "(" << q.w_ << ", " << q.x_ << ", " <<  q.y_ << ", " << q.z_ << ")";
            return os;
        }

		T w_;
		T x_;
		T y_;
		T z_;
	};

}//namespace utils



















#endif