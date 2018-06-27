#include "pinholeCameraModel.h"
#include <iostream>

namespace VGUGV
{
namespace Common
{
PinholeCameraModel::PinholeCameraModel(int height, int width, Eigen::Matrix3f& K)
: CameraBase(height, width, K)
{
	mCameraModelType = CameraModelType::PINHOLE;
}

std::vector<double> PinholeCameraModel::getCameraParams() const {
	std::vector<double> params;
	params.push_back(mK(0, 0)); // fx
	params.push_back(mK(1, 1)); // fy
	params.push_back(mK(0, 2)); // cx
	params.push_back(mK(1, 2)); // cy

	return params;
}

bool PinholeCameraModel::project(const Eigen::Vector3f& scenePoint, Eigen::Vector2f& pixelPoint, int pyramidLevel) const
{
	if (scenePoint(2) < 1e-3) return false;

	// project scene point to normalized image plane
	Vector2f normalizedPoint(scenePoint(0) / scenePoint(2), scenePoint(1) / scenePoint(2));
	if(mCameraDistortionModel != nullptr && ! mImageIsGeometricUndistorted)
	{
		mCameraDistortionModel->distort(normalizedPoint, normalizedPoint);
	}

	pixelPoint(0) = mK(0, 0) * normalizedPoint(0) + mK(0, 2);
	pixelPoint(1) = mK(1, 1) * normalizedPoint(1) + mK(1, 2);

	// check the pixel lies in bound
	if (!isInImage(pixelPoint, 10)) return false;
	float scale = 1 << pyramidLevel;
	pixelPoint /= scale;
	return true;
}

bool PinholeCameraModel::project(const Eigen::Vector3d& scenePoint, Eigen::Vector2d& pixelPoint, int pyramidLevel) const
{
	if (scenePoint(2) < 1e-3)
	{
		return false;
	}
	// project scene point to normalized image plane
	Vector2d normalizedPoint(scenePoint(0) / scenePoint(2), scenePoint(1) / scenePoint(2));
	if(mCameraDistortionModel != nullptr && ! mImageIsGeometricUndistorted)
	{
		mCameraDistortionModel->distort(normalizedPoint, normalizedPoint);
	}

	pixelPoint(0) = mK(0, 0) * normalizedPoint(0) + mK(0, 2);
	pixelPoint(1) = mK(1, 1) * normalizedPoint(1) + mK(1, 2);

	// check the pixel lies in bound
	if (!isInImage(pixelPoint, 10)) return false;
	double scale = 1 << pyramidLevel;
	pixelPoint /= scale;
	return true;
}

bool PinholeCameraModel::backProject(const Eigen::Vector2f& pixelPoint, Eigen::Vector3f& ray, int pyramidLevel) const
{
	// check the pixel lies in bound
	Common::Matrix3x3f Kinv = mKinv;
	int scale = 1 << pyramidLevel;
	Kinv(0, 0) *= scale;
	Kinv(1, 1) *= scale;

	float u = Kinv(0, 0)*pixelPoint(0) + Kinv(0, 2);
	float v = Kinv(1, 1)*pixelPoint(1) + Kinv(1, 2);

	if(mCameraDistortionModel != nullptr && ! mImageIsGeometricUndistorted)
	{
		Vector2f uPoint;
		mCameraDistortionModel->undistort(Vector2f(u, v), uPoint);
		u = uPoint(0);
		v = uPoint(1);
	}

	ray << u, v, 1.0f;
	ray.normalize();

	return true;
}

bool PinholeCameraModel::backProject(const Eigen::Vector2d& pixelPoint, Eigen::Vector3d& ray, int pyramidLevel) const
{
	Eigen::Vector3f fRay;
	bool bSuccess = backProject(pixelPoint.cast<float>(), fRay, pyramidLevel);
	ray = fRay.cast<double>();
	return bSuccess;
}

bool PinholeCameraModel::projectionJacobian(const Eigen::Vector3f& scenePoint, int pyramidLevel, Eigen::Matrix<float, 2, 3>& jacobian) const
{
	Common::Matrix2x3d jacobian_;
	bool success = projectionJacobian(scenePoint.cast<double>(), pyramidLevel, jacobian_);
	jacobian = jacobian_.cast<float>();
	return success;
}

bool PinholeCameraModel::projectionJacobian(const Eigen::Vector3d& scenePoint, int pyramidLevel, Eigen::Matrix<double, 2, 3>& jacobian) const
{
	double x = scenePoint(0); double y = scenePoint(1); double z = scenePoint(2);
	if (fabs(z) < 1e-3) return false;
	// projection from 3D landmark to normalized image plane
	double iz = 1.0f/z;
	double iz2 = iz*iz;

	Eigen::Matrix<double, 2, 3> J1;
	J1(0, 0) = iz;   J1(0, 1) = 0.0f; J1(0, 2) = (-1.0f)*x*iz2;
	J1(1, 0) = 0.0f; J1(1, 1) = iz; J1(1, 2) = (-1.0f)*y*iz2;

	// from normalized image plane to pixel
	jacobian = J1;

	if(mCameraDistortionModel != nullptr && ! mImageIsGeometricUndistorted)
	{
		Matrix2x2d distortionJacobian;
		mCameraDistortionModel->distortionJacobian(Vector2d(x/z, y/z), distortionJacobian);
		jacobian = distortionJacobian * jacobian;
	}

	jacobian.row(0) = jacobian.row(0) * mK(0, 0);
	jacobian.row(1) = jacobian.row(1) * mK(1, 1);

	// from pyramid level0 to pyramid pyramidLevel
	double scale = 1.0f / (1 << pyramidLevel);
	jacobian = scale * jacobian;

	return true;
}

}
}
