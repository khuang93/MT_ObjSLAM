#include "unifiedCameraModel.h"
#include <cmath>

namespace VGUGV
{
namespace Common
{
UnifiedCameraModel::UnifiedCameraModel(int height, int width, Eigen::Matrix3f& K, float xi)
: CameraBase(height, width, K)
, mXi(xi)
{
	mCameraModelType = CameraModelType::UNIFIED;
}

std::vector<double> UnifiedCameraModel::getCameraParams() const {
	std::vector<double> params;
	params.push_back(mK(0, 0)); // fx
	params.push_back(mK(1, 1)); // fy
	params.push_back(mK(0, 2)); // cx
	params.push_back(mK(1, 2)); // cy
	params.push_back(mXi);

	return params;
}

bool UnifiedCameraModel::project(const Eigen::Vector3f& scenePoint, Eigen::Vector2f& pixelPoint, int pyramidLevel) const
{
	if (scenePoint(2) < 1e-3) return false;

	float d = scenePoint.norm();
	float rz = 1.0 / (scenePoint(2) + mXi * d);

	// Project the scene point to the normalized plane.
	Vector2f normalizedPoint(scenePoint(0) * rz, scenePoint(1) * rz);
	if(mCameraDistortionModel != nullptr && ! mImageIsGeometricUndistorted)
	{
		mCameraDistortionModel->distort(normalizedPoint, normalizedPoint);
	}

	pixelPoint(0) = mK(0, 0) * normalizedPoint(0) + mK(0, 2);
	pixelPoint(1) = mK(1, 1) * normalizedPoint(1) + mK(1, 2);

	if(!isInImage(pixelPoint, 10)) return false;

	float scale = 1 << pyramidLevel;
	pixelPoint /= scale;
	return true;
}

bool UnifiedCameraModel::project(const Eigen::Vector3d& scenePoint, Eigen::Vector2d& pixelPoint, int pyramidLevel) const
{
	if (scenePoint(2) < 1e-3) return false;

	double d = scenePoint.norm();
	double rz = 1.0 / (scenePoint(2) + mXi * d);

	// Project the scene point to the normalized plane.
	Vector2d normalizedPoint(scenePoint(0) * rz, scenePoint(1) * rz);
	if(mCameraDistortionModel != nullptr && ! mImageIsGeometricUndistorted)
	{
		mCameraDistortionModel->distort(normalizedPoint, normalizedPoint);
	}

	pixelPoint(0) = mK(0, 0) * normalizedPoint(0) + mK(0, 2);
	pixelPoint(1) = mK(1, 1) * normalizedPoint(1) + mK(1, 2);

	if(!isInImage(pixelPoint, 10)) return false;

	double scale = 1 << pyramidLevel;
	pixelPoint /= scale;
	return true;
}
bool UnifiedCameraModel::backProject(const Eigen::Vector2d& pixelPoint, Eigen::Vector3d& ray, int pyramidLevel) const
{
	Eigen::Vector3f fRay;
	bool bSuccess = backProject(pixelPoint.cast<float>(), fRay, pyramidLevel);
	ray = fRay.cast<double>();
	return bSuccess;
}

bool UnifiedCameraModel::backProject(const Eigen::Vector2f& pixelPoint, Eigen::Vector3f& ray, int pyramidLevel) const
{
	// normalize the pixel point with K^(-1)
	Common::Matrix3x3f Kinv = mKinv;
	int scale = 1 << pyramidLevel;
	Kinv(0, 0) *= scale;
	Kinv(1, 1) *= scale;

	Eigen::Vector2f m_u(Kinv(0, 0) * pixelPoint(0) + Kinv(0, 2),
						Kinv(1, 1) * pixelPoint(1) + Kinv(1, 2));

	if(mCameraDistortionModel != nullptr && ! mImageIsGeometricUndistorted)
	{
		mCameraDistortionModel->undistort(m_u, m_u);
	}

	// Compute the unit ray vector that passes through the scene point.
	float rho2_u = m_u.squaredNorm();
	float beta = 1.0 + (1.0 - mXi * mXi) * rho2_u;
	if (beta < 0.0)
	{
		return false;
	}

	float lambda = (mXi + sqrt(beta)) / (1.0 + rho2_u);

	ray(0) = lambda * m_u(0);
	ray(1) = lambda * m_u(1);
	ray(2) = lambda - mXi;
	return true;
}

bool UnifiedCameraModel::projectionJacobian(const Eigen::Vector3f& scenePoint, int pyramidLevel, Eigen::Matrix<float, 2, 3>& jacobian) const
{
	// J1: from 3D point to sphere
	float d = scenePoint.norm();
	Eigen::Vector3f XYZs = scenePoint / d;
	float x = scenePoint(0); float y = scenePoint(1); float z = scenePoint(2);
	float xs = XYZs(0); float ys = XYZs(1); float zs = XYZs(2);
	float a1 = d*d; float a2 = d; float a3 = a1*a2;
	float a1i = 1.0f / a1; float a2i = 1.0f / a2; float a3i = 1.0f / a3;

	float x2 = x*x; float y2 = y*y; float z2 = z*z;
	float xy = x*y; float xz = x*z; float yz = y*z;

	Eigen::Matrix3f J1;
	J1(0, 0) = a2i - a3i*x2;   J1(0, 1) = (-1.0f)*xy*a3i; J1(0, 2) = (-1.0f)*xz*a3i;
	J1(1, 0) = (-1.0f)*xy*a3i; J1(1, 1) = a2i - a3i*y2;   J1(1, 2) = (-1.0f)*yz*a3i;
	J1(2, 0) = (-1.0f)*xz*a3i; J1(2, 1) = (-1.0f)*yz*a3i; J1(2, 2) = a2i - a3i*z2;

	// J2: from sphere to normalized image plane
	float b1 = mXi + zs; float b1i = 1.0f / b1;
	float b2 = b1*b1;    float b2i = 1.0f / b2;

	Eigen::Matrix<float, 2, 3> J2;
	J2(0, 0) = b1i; J2(0, 1) = 0;   J2(0, 2) = (-1.0f)*b2i*xs;
	J2(1, 0) = 0;   J2(1, 1) = b1i; J2(1, 2) = (-1.0f)*b2i*ys;

	// from normalized image plane to pixel
	jacobian = J2*J1;

	// Project the scene point to the normalized plane.
	if(mCameraDistortionModel != nullptr && ! mImageIsGeometricUndistorted)
	{
		double rz = 1.0 / (scenePoint(2) + mXi * d);
		Vector2f normalizedPoint(scenePoint(0) * rz, scenePoint(1) * rz);

		Matrix2x2f distortionJacobian;
		mCameraDistortionModel->distortionJacobian(normalizedPoint, distortionJacobian);
		jacobian = distortionJacobian * jacobian;
	}

	jacobian.row(0) = jacobian.row(0)*mK(0, 0);
	jacobian.row(1) = jacobian.row(1)*mK(1, 1);

	// from pyramid level0 to pyramid pyramidLevel
	float scale = 1.0f / (1 << pyramidLevel);
	jacobian = jacobian * scale;

	return true;
}

bool UnifiedCameraModel::projectionJacobian(const Eigen::Vector3d& scenePoint, int pyramidLevel, Eigen::Matrix<double, 2, 3>& jacobian) const
{
	// J1: from 3D point to sphere
	double d = scenePoint.norm();
	Eigen::Vector3d XYZs = scenePoint / d;
	double x = scenePoint(0); double y = scenePoint(1); double z = scenePoint(2);
	double xs = XYZs(0); double ys = XYZs(1); double zs = XYZs(2);
	double a1 = d*d; double a2 = d; double a3 = a1*a2;
	double a1i = 1.0f / a1; double a2i = 1.0f / a2; double a3i = 1.0f / a3;

	double x2 = x*x; double y2 = y*y; double z2 = z*z;
	double xy = x*y; double xz = x*z; double yz = y*z;

	Eigen::Matrix3d J1;
	J1(0, 0) = a2i - a3i*x2;   J1(0, 1) = (-1.0f)*xy*a3i; J1(0, 2) = (-1.0f)*xz*a3i;
	J1(1, 0) = (-1.0f)*xy*a3i; J1(1, 1) = a2i - a3i*y2;   J1(1, 2) = (-1.0f)*yz*a3i;
	J1(2, 0) = (-1.0f)*xz*a3i; J1(2, 1) = (-1.0f)*yz*a3i; J1(2, 2) = a2i - a3i*z2;

	// J2: from sphere to normalized image plane
	double b1 = mXi + zs; double b1i = 1.0f / b1;
	double b2 = b1*b1;    double b2i = 1.0f / b2;

	Eigen::Matrix<double, 2, 3> J2;
	J2(0, 0) = b1i; J2(0, 1) = 0;   J2(0, 2) = (-1.0f)*b2i*xs;
	J2(1, 0) = 0;   J2(1, 1) = b1i; J2(1, 2) = (-1.0f)*b2i*ys;

	// from normalized image plane to pixel
	jacobian = J2 * J1;

	// Project the scene point to the normalized plane.
	if(mCameraDistortionModel != nullptr && ! mImageIsGeometricUndistorted)
	{
		double rz = 1.0 / (scenePoint(2) + mXi * d);
		Vector2d normalizedPoint(scenePoint(0) * rz, scenePoint(1) * rz);
		Matrix2x2d distortionJacobian;
		mCameraDistortionModel->distortionJacobian(normalizedPoint, distortionJacobian);
		jacobian = distortionJacobian * jacobian;
	}

	jacobian.row(0) = jacobian.row(0) * mK(0, 0);
	jacobian.row(1) = jacobian.row(1) * mK(1, 1);

	// from pyramid level0 to pyramid pyramidLevel
	double scale = 1.0f / (1 << pyramidLevel);
	jacobian = jacobian * scale;

	return true;
}

}
}
