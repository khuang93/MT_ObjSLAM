/*
 * cameraDistortionBase.h
 *
 *  Created on: 20 Sep 2017
 *      Author: peidong
 */

#ifndef SRC_COMMON_SENSOR_CAMERADISTORTIONBASE_H_
#define SRC_COMMON_SENSOR_CAMERADISTORTIONBASE_H_

#include <Eigen/Dense>
#include <memory>
#include <vector>

#include "core/common/builtin_types.h"
#include "core/common/enums.h"

namespace VGUGV
{
	namespace Common
	{
		class CameraDistortionBase
		{
		  public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		  public:
			virtual std::vector<double> getDistortionParams() const = 0;
			virtual void distort(const Eigen::Vector2f &point, Eigen::Vector2f &dPoint) const = 0;
			virtual void distort(const Eigen::Vector2d &point, Eigen::Vector2d &dPoint) const = 0;
			virtual void undistort(const Eigen::Vector2f &point, Eigen::Vector2f &uPoint) const = 0;
			virtual void undistort(const Eigen::Vector2d &point, Eigen::Vector2d &uPoint) const = 0;
			virtual void distortionJacobian(const Vector2f &inputPoint, Matrix2x2f &jacobian) const = 0;
			virtual void distortionJacobian(const Vector2d &inputPoint, Matrix2x2d &jacobian) const = 0;

		  public:
			CameraDistortionType getDistortionType()
			{
				return mDistortionType;
			}

		  protected:
			CameraDistortionType mDistortionType;
		};

	} /* namespace Common */
} /* namespace VGUGV */

#endif /* SRC_COMMON_SENSOR_CAMERADISTORTIONBASE_H_ */
