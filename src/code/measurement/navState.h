/*
 * navState.h
 *
 *  Created on: 26 Sep 2017
 *      Author: peidong
 */

#ifndef SRC_COMMON_MEASUREMENT_NAVSTATE_H_
#define SRC_COMMON_MEASUREMENT_NAVSTATE_H_

#include <memory>
#include <mutex>
#include <Eigen/Dense>
#include "core/common/macro.h"
#include "core/common/builtin_types.h"
#include "core/nonMemberFunctions/geometry.h"
#include "core/transformation/transformation.h"

namespace VGUGV
{
	namespace Common
	{

		class NavState
		{
		  public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		  public:
			const static int nNavStatePoseDimension = 6;
			const static int nNavStateVelDimension = 3;
			const static int nNavStateBiasDimension = 6;

		  public:
			NavState();
			explicit NavState(Transformation& T);

		  public:
			Transformation getPose();
			Vector3d getVelocity();
			Vector3d getBiasAcc();
			Vector3d getBiasGyro();
			Vector6d getBiasAccGyro();

			Transformation getPoseZero();
			Vector3d getVelocityZero();
			Vector3d getBiasAccZero();
			Vector3d getBiasGyroZero();
			Vector6d getBiasAccGyroZero();

			void normalizeRotation();
			bool isLinearizationPointFixed();
			Vector6d getDeltaEstPose2LinearizationPoint();
			Vector3d getDeltaEstVel2LinearizationPoint();
			Vector6d getDeltaEstBias2LinearizationPoint();

		  public:
			void setPose(const Transformation &T);
			void setVelocity(const Vector3d &vel);
			void setImuBias(const Vector3d &accBias, const Vector3d &gyroBias);

			void updatePose_right(const Vector6d &tangent);

			void updateVelocity(const Vector3d &deltaVel);

			void updateImuBias(const Vector6d &deltaBias);

			Vector6d getUpdatePoseTangent();
			Vector3d getUpdateVel();
			Vector6d getUpdateImuBias();

			void fixLinearizationPoint();

		  public:
			friend std::ostream &operator<<(std::ostream &os, NavState &object);

		  private:
			Transformation mPose; // transformation from vehicle body frame to global world frame
			Vector3d mVelocity;  // vehicle velocity in global world frame
			Vector6d mBiasAccGyro;

			Transformation mPoseZero;
			Vector3d mVelocityZero;
			Vector6d mBiasAccGyroZero;
			bool mbLinearizationPointIsFixed;

		  private:
			Vector6d mDeltaTangent; //scaled delta translation, omega increment
			Vector3d mDeltaVelocity;  // scaled delta increment
			Vector6d mDeltaImuBias;  // scaled delta increment

			std::mutex mMutexPose;
			std::mutex mMutexVel;
			std::mutex mMutexImuBias;
		};

	} /* namespace Common */
} /* namespace VGUGV */

#endif /* SRC_COMMON_MEASUREMENT_NAVSTATE_H_ */
