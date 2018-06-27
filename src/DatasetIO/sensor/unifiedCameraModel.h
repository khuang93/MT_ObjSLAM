#ifndef _VGUGV_COMMON_UNIFIED_CAMERA_MODEL_
#define _VGUGV_COMMON_UNIFIED_CAMERA_MODEL_

#include "cameraBase.h"
#include <memory>
#include "cameraDistortionFoV.h"
#include "cameraDistortionRadTan.h"

namespace VGUGV
{
	namespace Common
	{
		class UnifiedCameraModel : public CameraBase
		{
		  public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW

			UnifiedCameraModel(int height, int width, Eigen::Matrix3f &K, float xi);

		  public:
			std::vector<double> getCameraParams() const override;

			bool project(const Eigen::Vector3f &scenePoint,
						 Eigen::Vector2f &pixelPoint,
						 int pyramidLevel) const override;

			bool project(const Eigen::Vector3d &scenePoint,
						 Eigen::Vector2d &pixelPoint,
						 int pyramidLevel) const override;

			bool backProject(const Eigen::Vector2f &pixelPoint, Eigen::Vector3f &ray, int pyramidLevel) const override;;

			bool backProject(const Eigen::Vector2d &pixelPoint, Eigen::Vector3d &ray, int pyramidLevel) const override;;

			bool projectionJacobian(const Eigen::Vector3f &scenePoint, int pyramidLevel,
									Eigen::Matrix<float, 2, 3> &jacobian) const override;;

			bool projectionJacobian(const Eigen::Vector3d &scenePoint, int pyramidLevel,
									Eigen::Matrix<double, 2, 3> &jacobian) const override;;

		  private:
			double mXi;
		};
	}
}

#endif
