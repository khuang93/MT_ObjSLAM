#ifndef _VGUGV_COMMON_PINHOLE_CAMEAR_MODEL_
#define _VGUGV_COMMON_PINHOLE_CAMEAR_MODEL_

#include <memory>
#include "cameraBase.h"
#include "cameraDistortionFoV.h"
#include "cameraDistortionRadTan.h"

using namespace VGUGV::Common;

namespace VGUGV
{
	namespace Common
	{
		class PinholeCameraModel : public CameraBase
		{
		  public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		  public:
			// image size (row, col)
			PinholeCameraModel(int height, int width, Eigen::Matrix3f &K);

			std::vector<double> getCameraParams() const override;
			bool project(const Eigen::Vector3f &scenePoint,
						 Eigen::Vector2f &pixelPoint,
						 int pyramidLevel) const override;
			bool project(const Eigen::Vector3d &scenePoint,
						 Eigen::Vector2d &pixelPoint,
						 int pyramidLevel) const override;
			bool backProject(const Eigen::Vector2f &pixelPoint, Eigen::Vector3f &ray, int pyramidLevel) const override;
			bool backProject(const Eigen::Vector2d &pixelPoint, Eigen::Vector3d &ray, int pyramidLevel) const override;
			bool projectionJacobian(const Eigen::Vector3f &scenePoint,
									int pyramidLevel,
									Eigen::Matrix<float, 2, 3> &jacobian) const override;
			bool projectionJacobian(const Eigen::Vector3d &scenePoint,
									int pyramidLevel,
									Eigen::Matrix<double, 2, 3> &jacobian) const override;
		};

	}
}
#endif
