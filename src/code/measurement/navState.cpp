//
// Created by peidong on 26.10.17.
//

#include "navState.h"

namespace VGUGV
{
	namespace Common
	{
		NavState::NavState()
		{
			mVelocity.setZero();
			mBiasAccGyro.setZero();

			mVelocityZero.setZero();
			mBiasAccGyroZero.setZero();

			mbLinearizationPointIsFixed = false;
		}

		NavState::NavState(Transformation& T)
		{
			mPose = T;
			mVelocity.setZero();
			mBiasAccGyro.setZero();

			mPoseZero = T;
			mVelocityZero.setZero();
			mBiasAccGyroZero.setZero();

			mbLinearizationPointIsFixed = false;
		}

		Transformation NavState::getPose()
		{
			std::lock_guard<std::mutex> lock(mMutexPose);
			return mPose;
		}

		Vector3d NavState::getVelocity()
		{
			std::lock_guard<std::mutex> lock(mMutexVel);
			return mVelocity;
		}

		Vector3d NavState::getBiasAcc()
		{
			std::lock_guard<std::mutex> lock(mMutexImuBias);
			return mBiasAccGyro.head(3);
		}

		Vector3d NavState::getBiasGyro()
		{
			std::lock_guard<std::mutex> lock(mMutexImuBias);
			return mBiasAccGyro.tail(3);
		}

		Vector6d NavState::getBiasAccGyro()
		{
			std::lock_guard<std::mutex> lock(mMutexImuBias);
			return mBiasAccGyro;
		}

		Transformation NavState::getPoseZero()
		{
			std::lock_guard<std::mutex> lock(mMutexPose);
			return mPoseZero;
		}

		Vector3d NavState::getVelocityZero()
		{
			std::lock_guard<std::mutex> lock(mMutexVel);
			return mVelocityZero;
		}

		Vector3d NavState::getBiasAccZero()
		{
			std::lock_guard<std::mutex> lock(mMutexImuBias);
			return mBiasAccGyroZero.head(3);
		}

		Vector3d NavState::getBiasGyroZero()
		{
			std::lock_guard<std::mutex> lock(mMutexImuBias);
			return mBiasAccGyroZero.tail(3);
		}

		Vector6d NavState::getBiasAccGyroZero()
		{
			std::lock_guard<std::mutex> lock(mMutexImuBias);
			return mBiasAccGyroZero;
		}

		void NavState::normalizeRotation()
		{
			std::lock_guard<std::mutex> lock(mMutexPose);
			mPoseZero.normalize();
			mPose.normalize();
		}

		bool NavState::isLinearizationPointFixed()
		{
			return mbLinearizationPointIsFixed;
		}

		Vector6d NavState::getDeltaEstPose2LinearizationPoint()
		{
			std::lock_guard<std::mutex> lock(mMutexPose);
			Vector6d delta;
			delta.setZero();
			if (!mbLinearizationPointIsFixed) return delta;
			Transformation deltaT = mPoseZero.inverse() * mPose;

			delta = SE3d::log(deltaT);
			delta.head(3) /= SE3_TRANSLATION_SCALING_FACTOR;
			delta.tail(3) /= SE3_ROTATION_SCALING_FACTOR;
			return delta;
		}

		Vector3d NavState::getDeltaEstVel2LinearizationPoint()
		{
			std::lock_guard<std::mutex> lock(mMutexVel);
			if (!mbLinearizationPointIsFixed) return Vector3d(0, 0, 0);
			return (mVelocity - mVelocityZero) / TRANSLATION_VELOCITY_SCALING_FACTOR;
		}

		Vector6d NavState::getDeltaEstBias2LinearizationPoint()
		{
			std::lock_guard<std::mutex> lock(mMutexImuBias);
			Vector6d delta;
			delta.setZero();
			if (!mbLinearizationPointIsFixed) return delta;

			delta = mBiasAccGyro - mBiasAccGyroZero;
			delta.head(3) /= IMU_ACC_BIAS_SCALING_FACTOR;
			delta.tail(3) /= IMU_GYRO_BIAS_SCALING_FACTOR;

			return delta;
		}

		void NavState::setPose(const Transformation &T)
		{
			std::lock_guard<std::mutex> lock(mMutexPose);
			mPose = T;
			mPoseZero = T;
		};

		void NavState::setVelocity(const Vector3d &vel)
		{
			std::lock_guard<std::mutex> lock(mMutexVel);
			mVelocity = vel;
			mVelocityZero = vel;
		}

		void NavState::setImuBias(const Vector3d &accBias, const Vector3d &gyroBias)
		{
			std::lock_guard<std::mutex> lock(mMutexImuBias);
			mBiasAccGyro.head(3) = accBias;
			mBiasAccGyro.tail(3) = gyroBias;

			mBiasAccGyroZero = mBiasAccGyro;
		}

		void NavState::updatePose_right(const Vector6d &tangent)
		{
			std::lock_guard<std::mutex> lock(mMutexPose);
			mDeltaTangent = tangent;

			Vector6d unScaledDeltaTangent = tangent;
			unScaledDeltaTangent.head(3) *= SE3_TRANSLATION_SCALING_FACTOR;
			unScaledDeltaTangent.tail(3) *= SE3_ROTATION_SCALING_FACTOR;

			mPose = mPose * SE3d::exp(unScaledDeltaTangent);

			if (!mbLinearizationPointIsFixed)
			{
				mPoseZero = mPose;
			}
		}

		void NavState::updateVelocity(const Vector3d &deltaVel)
		{
			std::lock_guard<std::mutex> lock(mMutexVel);
			mDeltaVelocity = deltaVel;

			mVelocity += (TRANSLATION_VELOCITY_SCALING_FACTOR * deltaVel);
			if (!mbLinearizationPointIsFixed)
			{
				mVelocityZero = mVelocity;
			}
		}

		void NavState::updateImuBias(const Vector6d &deltaBias)
		{
			std::lock_guard<std::mutex> lock(mMutexImuBias);
			mDeltaImuBias = deltaBias;

			mBiasAccGyro.head(3) += (IMU_ACC_BIAS_SCALING_FACTOR * mDeltaImuBias.head(3));
			mBiasAccGyro.tail(3) += (IMU_GYRO_BIAS_SCALING_FACTOR * mDeltaImuBias.tail(3));

			if (!mbLinearizationPointIsFixed)
			{
				mBiasAccGyroZero = mBiasAccGyro;
			}
		}

		Vector6d NavState::getUpdatePoseTangent()
		{
			return mDeltaTangent;
		}

		Vector3d NavState::getUpdateVel()
		{
			return mDeltaVelocity;
		}

		Vector6d NavState::getUpdateImuBias()
		{
			return mDeltaImuBias;
		}

		void NavState::fixLinearizationPoint()
		{
			mbLinearizationPointIsFixed = true;
		}

		std::ostream &operator<<(std::ostream &os, NavState &object)
		{
			os << "-------------------------------------------------\n";
			os << "|                    NavState                   |\n";
			os << "-------------------------------------------------\n";
			os << "| Pose     : " << object.getPose() << "\n";
			os << "| Velocity : " << object.getVelocity().transpose() << "\n";
			os << "| Bias     : " << object.getBiasAccGyro().transpose() << "\n";
			os << "-------------------------------------------------\n";
			return os;
		}

	} // namespace Common
} // namespace VGUGV