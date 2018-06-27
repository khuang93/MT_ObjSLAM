//
// Created by peidong on 05.11.17.
//

#ifndef VGUGV_CAMERADISTORTIONRADTAN_H
#define VGUGV_CAMERADISTORTIONRADTAN_H

#include "cameraDistortionBase.h"
#include <vector>

namespace VGUGV
{
    namespace Common
    {
        class CameraDistortionRadTan : public CameraDistortionBase
        {
          public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

          public:
            CameraDistortionRadTan(double k1, double k2, double p1, double p2);

          public:
            std::vector<double> getDistortionParams() const;
            void distort(const Eigen::Vector2f &point, Eigen::Vector2f &dPoint) const;
            void distort(const Eigen::Vector2d &point, Eigen::Vector2d &dPoint) const;
            void undistort(const Eigen::Vector2f &point, Eigen::Vector2f &uPoint) const;
            void undistort(const Eigen::Vector2d &point, Eigen::Vector2d &uPoint) const;
            void distortionJacobian(const Vector2f &inputPoint, Matrix2x2f &jacobian) const;
            void distortionJacobian(const Vector2d &inputPoint, Matrix2x2d &jacobian) const;

          private:
            void distort(const Eigen::Vector2d &point, Eigen::Vector2d &dPoint, Eigen::Matrix2d &jacobian) const;

          private:
            double mk1, mk2, mp1, mp2;
        };
    }
}

#endif //VGUGV_CAMERADISTORTIONRADTAN_H
