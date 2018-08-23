#ifndef _VGUGV_COMMON_POSE_
#define _VGUGV_COMMON_POSE_

#include <Eigen/Dense>
#include <memory>
#include <sophus/se3.hpp>
#include <math.h>
//#include "core/nonMemberFunctions/nonMemberFunctions.h"

namespace VGUGV
{
    namespace Common
    {
        Eigen::Vector3d rad2deg(Eigen::Vector3d in_rpy);
        class Transformation : public Sophus::SE3d
        {
          public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            using Base = Sophus::SE3d;
            using Base::SE3;
            using Base::operator*;
            using Base::operator*=;

            Transformation();
            Transformation(double roll, double pitch, double yaw, const Eigen::Vector3d &t);

            Eigen::Vector3d getRollPitchYaw() const;
            void setRollPitchYaw(double roll, double pitch, double yaw);

            Eigen::Vector3d operator*(const Eigen::Vector3d &point) const;
            Transformation operator*(const Eigen::Matrix4d &aT) const;
            Transformation operator*(const Eigen::Matrix<double, 6, 1> &tangent) const;



            friend std::ostream &operator<<(std::ostream &stream, Transformation obj)
            {
                Eigen::Vector3d rpy = obj.getRollPitchYaw();
//                stream << "rpy: |" << rad2deg<Eigen::Vector3d>(rpy).transpose() << "| ";
                stream << "rpy: |" << rad2deg(rpy).transpose() << "| ";
                stream << "xyz: |" << obj.translation().transpose() << "|";
                return stream;
            }
        };

    }
}

#endif
