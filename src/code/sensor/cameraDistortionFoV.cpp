/*
 * fovDistortionModel.cpp
 *
 *  Created on: 20 Sep 2017
 *      Author: peidong
 */

#include <core/sensor/cameraDistortionFoV.h>
#include <vector>

namespace VGUGV {
namespace Common {

CameraDistortionFoV::CameraDistortionFoV(double w)
: mW(w)
, m_2tan_halfW ( 2 * tan (0.5 * w) )
{
	mDistortionType = CameraDistortionType::FoV;
}

std::vector<double> CameraDistortionFoV::getDistortionParams() const {
	std::vector<double> params;
	params.push_back(mW);
	return params;
}

void CameraDistortionFoV::distort(const Eigen::Vector2f& point, Eigen::Vector2f& dPoint) const
{
	float mx2_u = point(0) * point(0);
	float my2_u = point(1) * point(1);
	float rho_u = sqrt(mx2_u + my2_u);
	float rho_d = atan(rho_u * m_2tan_halfW) / mW;
	float rho_d_over_rho_u = rho_d / rho_u;

	dPoint(0) = point(0) * rho_d_over_rho_u;
	dPoint(1) = point(1) * rho_d_over_rho_u;
}

void CameraDistortionFoV::distort(const Eigen::Vector2d& point, Eigen::Vector2d& dPoint) const
{
	double mx2_u = point(0) * point(0);
	double my2_u = point(1) * point(1);
	double rho_u = sqrt(mx2_u + my2_u);
	double rho_d = atan(rho_u * m_2tan_halfW) / mW;
	double rho_d_over_rho_u = rho_d / rho_u;

	dPoint(0) = point(0) * rho_d_over_rho_u;
	dPoint(1) = point(1) * rho_d_over_rho_u;
}

void CameraDistortionFoV::undistort(const Eigen::Vector2f& point, Eigen::Vector2f& udPoint) const
{
	float mx2_d = point(0) * point(0);
	float my2_d = point(1) * point(1);
	float rho_d = sqrt(mx2_d + my2_d);
	float rho_u = tan(rho_d * mW) / m_2tan_halfW;
	float rho_u_over_rho_d = rho_u / rho_d;

	udPoint(0) = point(0) * rho_u_over_rho_d;
	udPoint(1) = point(1) * rho_u_over_rho_d;
}

void CameraDistortionFoV::undistort(const Eigen::Vector2d& point, Eigen::Vector2d& udPoint) const
{
	double mx2_d = point(0) * point(0);
	double my2_d = point(1) * point(1);
	double rho_d = sqrt(mx2_d + my2_d);
	double rho_u = tan(rho_d * mW) / m_2tan_halfW;
	double rho_u_over_rho_d = rho_u / rho_d;

	udPoint(0) = point(0) * rho_u_over_rho_d;
	udPoint(1) = point(1) * rho_u_over_rho_d;
}

void CameraDistortionFoV::distortionJacobian(const Vector2f& point, Matrix2x2f& jacobian) const
{
	float mx2_u = point(0) * point(0);
	float my2_u = point(1) * point(1);
	float rho_u = sqrt(mx2_u + my2_u);
	float rho_d = atan(rho_u * m_2tan_halfW) / mW;
	float rho_d_over_rho_u = rho_d / rho_u;

	float mx2_u_p_my2_u = mx2_u + my2_u;
	float a = m_2tan_halfW / (mW * mx2_u_p_my2_u * (mx2_u_p_my2_u * m_2tan_halfW * m_2tan_halfW + 1)) - rho_d_over_rho_u / mx2_u_p_my2_u;

	jacobian(0, 0) = mx2_u * a + rho_d_over_rho_u;
	jacobian(1, 0) = point(0) * point(1) * a;
	jacobian(0, 1) = jacobian(1, 0);
	jacobian(1, 1) = my2_u * a + rho_d_over_rho_u;
}

void CameraDistortionFoV::distortionJacobian(const Vector2d& point, Matrix2x2d& jacobian) const
{
	double mx2_u = point(0) * point(0);
	double my2_u = point(1) * point(1);
	double rho_u = sqrt(mx2_u + my2_u);
	double rho_d = atan(rho_u * m_2tan_halfW) / mW;
	double rho_d_over_rho_u = rho_d / rho_u;

	double mx2_u_p_my2_u = mx2_u + my2_u;
	double a = m_2tan_halfW / (mW * mx2_u_p_my2_u * (mx2_u_p_my2_u * m_2tan_halfW * m_2tan_halfW + 1)) - rho_d_over_rho_u / mx2_u_p_my2_u;

	jacobian(0, 0) = mx2_u * a + rho_d_over_rho_u;
	jacobian(1, 0) = point(0) * point(1) * a;
	jacobian(0, 1) = jacobian(1, 0);
	jacobian(1, 1) = my2_u * a + rho_d_over_rho_u;
}

} /* namespace Common */
} /* namespace VGUGV */
