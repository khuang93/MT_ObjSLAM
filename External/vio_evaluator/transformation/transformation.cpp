#include "transformation.h"

namespace VGUGV
{
    namespace Common
    {
        Transformation::Transformation()
        : Base()
        {
        }

        Transformation::Transformation(double roll, double pitch, double yaw, const Eigen::Vector3d &t)
        {
            Eigen::Quaterniond R(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                                     * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
            so3_ = Sophus::SO3<double>(R);
            translation_ = t;
        }

        Eigen::Vector3d Transformation::getRollPitchYaw() const
        {
            double w = so3_.unit_quaternion().w();
            double x = so3_.unit_quaternion().x();
            double y = so3_.unit_quaternion().y();
            double z = so3_.unit_quaternion().z();

            double roll = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
            double pitch = asin(2.0 * (w * y - x * z));
            double yaw = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));

            Eigen::Vector3d rpy(roll, pitch, yaw);
            return rpy;
        }

        void Transformation::setRollPitchYaw(double roll, double pitch, double yaw)
        {
            double cr = cos(0.5 * roll);
            double sr = sin(0.5 * roll);
            double cp = cos(0.5 * pitch);
            double sp = sin(0.5 * pitch);
            double cy = cos(0.5 * yaw);
            double sy = sin(0.5 * yaw);

            so3_ = Sophus::SO3<double>(Eigen::Quaterniond(cr * cp * cy + sr * sp * sy,
                                                  sr * cp * cy - cr * sp * sy,
                                                  cr * sp * cy + sr * cp * sy,
                                                  cr * cp * sy - sr * sp * cy));
        }

        static Eigen::Vector3d rad2deg(Eigen::Vector3d& in_rpy){
            return in_rpy*180.0/M_PI;
        }

        Eigen::Vector3d Transformation::operator*(const Eigen::Vector3d &point) const
        {
            return Sophus::SE3d::operator*(point);
        }

        Transformation Transformation::operator*(const Eigen::Matrix4d &aT) const
        {
            Transformation T(*this);
            T *= Sophus::SE3d(aT);
            return T;
        }

        Transformation Transformation::operator*(const Eigen::Matrix<double, 6, 1> &tangent) const
        {
            Transformation T(*this);
            T *= SE3d::exp(tangent);
            return T;
        }
    }
}
