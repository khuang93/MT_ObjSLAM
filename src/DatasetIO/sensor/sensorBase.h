/*
 * sensorBase.h
 *
 *  Created on: 19 Sep 2017
 *      Author: peidong
 */

#ifndef SRC_COMMON_SENSOR_SENSORBASE_H_
#define SRC_COMMON_SENSOR_SENSORBASE_H_

#include <memory>
#include "core/transformation/transformation.h"
#include "core/common/builtin_types.h"

namespace VGUGV
{
	namespace Common
	{
		class SensorBase
		{
		  public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		  public:
			const static int nSensorExtrinsicStateDimension = 6;

		  public:
			void setDeviceID(int ID);
			void setDeviceName(const std::string &devName);
			void setT_Body2Sensor(const Transformation &T);
			void setT_Body2Sensor(const Eigen::Matrix4d &T);
			void setT_Body2Sensor(const Eigen::Matrix4d &T, const Eigen::Matrix<double, 6, 6> &covariance);

		  public:
			int getDeviceID() const;
			std::string getDeviceName() const;
			const Transformation &getT_Body2Sensor() const;

			Matrix4x4d getJacobian_t0() const;
			Matrix4x4d getJacobian_t1() const;
			Matrix4x4d getJacobian_t2() const;
			Matrix4x4d getJacobian_w0() const;
			Matrix4x4d getJacobian_w1() const;
			Matrix4x4d getJacobian_w2() const;

		  private:
			void computeJacobians();

		  protected:
			Transformation mT_Body2Sensor;
			std::string mDeviceName;
			int mDeviceID;
			// jacobian of T_b^s * delta_T * T_s^b to se3 of delta_T
			Common::Matrix4x4d mJacobianT_t0, mJacobianT_t1, mJacobianT_t2, mJacobianT_w0, mJacobianT_w1, mJacobianT_w2;
		};
	} // namespace Common
} // namespace VGUGV



#endif /* SRC_COMMON_SENSOR_SENSORBASE_H_ */
