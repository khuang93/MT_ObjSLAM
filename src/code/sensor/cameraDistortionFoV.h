/*
 * fovDistortionModel.h
 *
 *  Created on: 20 Sep 2017
 *      Author: peidong
 */

#ifndef SRC_COMMON_SENSOR_CAMERADISTORTIONFOV_H_
#define SRC_COMMON_SENSOR_CAMERADISTORTIONFOV_H_

#include <core/sensor/cameraDistortionBase.h>
#include <vector>

namespace VGUGV
{
	namespace Common
	{
		class CameraDistortionFoV : public CameraDistortionBase
		{
		  public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		  public:
			CameraDistortionFoV(double w);

		  public:
			std::vector<double> getDistortionParams() const;
			void distort(const Eigen::Vector2f &point, Eigen::Vector2f &dPoint) const;
			void distort(const Eigen::Vector2d &point, Eigen::Vector2d &dPoint) const;
			void undistort(const Eigen::Vector2f &point, Eigen::Vector2f &uPoint) const;
			void undistort(const Eigen::Vector2d &point, Eigen::Vector2d &uPoint) const;
			void distortionJacobian(const Vector2f &inputPoint, Matrix2x2f &jacobian) const;
			void distortionJacobian(const Vector2d &inputPoint, Matrix2x2d &jacobian) const;

		  private:
			double mW;
			double m_2tan_halfW;
		};

	} /* namespace Common */
} /* namespace VGUGV */

#endif /* SRC_COMMON_SENSOR_CAMERADISTORTIONFOV_H_ */
