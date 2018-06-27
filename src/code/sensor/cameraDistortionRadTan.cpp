//
// Created by peidong on 05.11.17.
//

#include "cameraDistortionRadTan.h"

namespace VGUGV{
    namespace Common{

        CameraDistortionRadTan::CameraDistortionRadTan(double k1, double k2, double p1, double p2)
        : mk1(k1), mk2(k2), mp1(p1), mp2(p2)
        {
            mDistortionType = CameraDistortionType::RAD_TAN;
        }

        std::vector<double> CameraDistortionRadTan::getDistortionParams() const
        {
            std::vector<double> params;
            params.push_back(mk1);
            params.push_back(mk2);
            params.push_back(mp1);
            params.push_back(mp2);
            return params;
        }

        void CameraDistortionRadTan::distort(const Eigen::Vector2f& point, Eigen::Vector2f& dPoint) const
        {
            Eigen::Vector2d point_(point(0), point(1)); Eigen::Vector2d dPoint_;
            distort(point_, dPoint_);
            dPoint(0) = dPoint_(0);
            dPoint(1) = dPoint_(1);
        }

        void CameraDistortionRadTan::distort(const Eigen::Vector2d& point, Eigen::Vector2d& dPoint) const
        {
            double mx2_u = point(0) * point(0);
            double my2_u = point(1) * point(1);
            double mxy_u = point(0) * point(1);
            double rho2_u = mx2_u + my2_u;
            double rad_dist_u = mk1 * rho2_u + mk2 * rho2_u * rho2_u;

            dPoint(0) = point(0) + point(0) * rad_dist_u + 2.0 * mp1 * mxy_u + mp2 * (rho2_u + 2.0 * mx2_u);
            dPoint(1) = point(1) + point(1) * rad_dist_u + 2.0 * mp2 * mxy_u + mp1 * (rho2_u + 2.0 * my2_u);
        }

        void CameraDistortionRadTan::distort(const Eigen::Vector2d& point,
                                             Eigen::Vector2d& dPoint,
                                             Eigen::Matrix2d& jacobian) const
        {
            double mx2_u = point(0) * point(0);
            double my2_u = point(1) * point(1);
            double mxy_u = point(0) * point(1);
            double rho2_u = mx2_u + my2_u;
            double rad_dist_u = mk1 * rho2_u + mk2 * rho2_u * rho2_u;

            dPoint(0) = point(0) + point(0) * rad_dist_u + 2.0 * mp1 * mxy_u + mp2 * (rho2_u + 2.0 * mx2_u);
            dPoint(1) = point(1) + point(1) * rad_dist_u + 2.0 * mp2 * mxy_u + mp1 * (rho2_u + 2.0 * my2_u);

            jacobian(0, 0) = 1.0 + rad_dist_u + 2.0 * mk1 * mx2_u + 4.0 * mk2 * mx2_u * rho2_u + 2.0 * mp1 * point(1) + 6.0 * mp2 * point(0);
            jacobian(0, 1) = 2.0 * mk1 * mxy_u + 4.0 * mk2 * rho2_u * mxy_u + 2.0 * mp1 * point(0) + 2.0 * mp2 * point(1);
            jacobian(1, 0) = jacobian(0, 1);
            jacobian(1, 1) = 1.0 + rad_dist_u + 2.0 * mk1 * my2_u + 4.0 * mk2 * my2_u * rho2_u + 2.0 * mp2 * point(0) + 6.0 * mp1 * point(1);
        }

        void CameraDistortionRadTan::undistort(const Eigen::Vector2f& point, Eigen::Vector2f& uPoint) const
        {
            Eigen::Vector2d point_(point(0), point(1)); Eigen::Vector2d uPoint_;
            undistort(point_, uPoint_);
            uPoint(0) = uPoint_(0);
            uPoint(1) = uPoint_(1);
        }

        void CameraDistortionRadTan::undistort(const Eigen::Vector2d& point, Eigen::Vector2d& udPoint) const
        {
            const int n = 5;

            Eigen::Vector2d udPoint_bar = point;

            for (int i = 0; i < n; ++i)
            {
                Eigen::Vector2d dp;
                Eigen::Matrix2d J;
                distort(udPoint_bar, dp, J);

                Eigen::Vector2d e = point - dp;
                Eigen::Vector2d du = (J.transpose() * J).inverse() * J.transpose() * e;

                udPoint_bar += du;

                if (e.dot(e) < 1e-15)
                {
                    break;
                }
            }
            udPoint = udPoint_bar;
        }

        void CameraDistortionRadTan::distortionJacobian(const Vector2f& inputPoint, Matrix2x2f& jacobian) const
        {
            Matrix2x2d jacobian_;
            distortionJacobian(inputPoint.cast<double>(), jacobian_);
            jacobian = jacobian_.cast<float>();
        }

        void CameraDistortionRadTan::distortionJacobian(const Vector2d& inputPoint, Matrix2x2d& jacobian) const
        {
            Vector2d dPoint;
            distort(inputPoint, dPoint, jacobian);
        }
    }
}