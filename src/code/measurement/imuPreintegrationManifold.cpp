/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  ManifoldPreintegration.cpp
 *  @author Luca Carlone
 *  @author Stephen Williams
 *  @author Richard Roberts
 *  @author Vadim Indelman
 *  @author David Jensen
 *  @author Frank Dellaert
 **/

#include "imuPreintegrationManifold.h"


namespace VGUGV
{
	namespace Common
	{

		ImuPreintegrationManifold::ImuPreintegrationManifold(ImuSensorModel* sensor, const ImuBias &bias)
			: mIMUSensorModel(sensor), mPIM(NULL), mInertialResiduals(nullptr)
		{
			// create gtsam PIM and params
			double accel_noise_sigma = mIMUSensorModel->getAccelerometerNoiseDensity();
			double gyro_noise_sigma = mIMUSensorModel->getGyroscopeNoiseDensity();
			double accel_bias_rw_sigma = mIMUSensorModel->getAccelerometerDriftNoiseDensity();
			double gyro_bias_rw_sigma = mIMUSensorModel->getGyroscopeDriftNoiseDensity();

			Common::Matrix3x3d measured_acc_cov = pow(accel_noise_sigma, 2) * Common::Matrix3x3d::Identity(3, 3);
			Common::Matrix3x3d measured_omega_cov = pow(gyro_noise_sigma, 2) * Common::Matrix3x3d::Identity(3, 3);

			Common::Matrix3x3d integration_error_cov = 1e-9 * Common::Matrix3x3d::Identity(3, 3); // error committed in integrating position from velocities
			Common::Matrix3x3d bias_acc_cov = pow(accel_bias_rw_sigma, 2) * Common::Matrix3x3d::Identity(3, 3);
			Common::Matrix3x3d bias_omega_cov = pow(gyro_bias_rw_sigma, 2) * Common::Matrix3x3d::Identity(3, 3);
			Common::Matrix6x6d bias_acc_omega_int = 1e-5 * Common::Matrix6x6d::Identity(); // error in the bias used for preintegration

			// initialize gtsamWrapper
			boost::shared_ptr<PreintegratedCombinedMeasurements::Params> PIMparams
				= PreintegratedCombinedMeasurements::Params::MakeSharedU(mIMUSensorModel->getEarthAcceleration());

			PIMparams->accelerometerCovariance = measured_acc_cov;
			PIMparams->gyroscopeCovariance = measured_omega_cov;
			PIMparams->integrationCovariance = integration_error_cov;
			PIMparams->biasAccCovariance = bias_acc_cov;
			PIMparams->biasOmegaCovariance = bias_omega_cov;
			PIMparams->biasAccOmegaInt = bias_acc_omega_int;

			GtsamImuBias::ConstantBias gtsambias(bias.accelerometer(), bias.gyroscope());
			mPIM = new PreintegratedCombinedMeasurements(PIMparams, gtsambias);
		}

		ImuPreintegrationManifold* ImuPreintegrationManifold::clone()
		{
			ImuPreintegrationManifold* _copy = new ImuPreintegrationManifold(this->mIMUSensorModel, ImuBias());
			(*_copy->getPIM()) = (*this->mPIM);
			return _copy;
		}

		ImuPreintegrationManifold::~ImuPreintegrationManifold()
		{
			delete mPIM;
			delete mInertialResiduals;
		}

		void ImuPreintegrationManifold::resetIntegration()
		{
			mPIM->resetIntegration();
		}

		void ImuPreintegrationManifold::resetIntegrationAndSetBias(const ImuBias &biasHat)
		{
			mPIM->resetIntegrationAndSetBias(GtsamImuBias::ConstantBias(biasHat.accelerometer(), biasHat.gyroscope()));
		}

		void ImuPreintegrationManifold::integrateMeasurement(const Vector3d &acc_,
															 const Vector3d &omega_, const double dt)
		{
			mPIM->integrateMeasurement(acc_, omega_, dt);
		}

	} // namespace Common
} // namespace VGUGV
