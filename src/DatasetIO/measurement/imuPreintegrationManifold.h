#pragma once

#include <iomanip>
#include <memory>
#include "imuBias.h"
#include "core/sensor/imuSensorModel.h"
#include "core/residual/residualBase.h"
#include "core/gtsamWrapper/CombinedImuFactor.h"

namespace VGUGV {
	namespace Common {

		class ImuPreintegrationManifold
		{
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		public:
			ImuPreintegrationManifold(ImuSensorModel* sensor, const ImuBias& bias);

			ImuPreintegrationManifold* clone();

			~ImuPreintegrationManifold();

			PreintegratedCombinedMeasurements* getPIM() { return mPIM; }

		public:
			/// Re-initialize PreintegratedMeasurements
			void resetIntegration();
			void resetIntegrationAndSetBias(const ImuBias& biasHat);
			void integrateMeasurement(const Vector3d& measuredAcc, const Vector3d& measuredOmega, const double dt); // dt in seconds

		public:
			ResidualBase* getInertialResidual() { return mInertialResiduals; }
			void addInertialResidual(ResidualBase* residual)
			{
				mInertialResiduals = residual;
			}
			void removeInertialResidual()
			{
				mInertialResiduals = nullptr;
			}

		private:
			// use gtsam's code for covariance propagation temporarily
			PreintegratedCombinedMeasurements* mPIM;
			ImuSensorModel* mIMUSensorModel;
			ResidualBase*   mInertialResiduals;
		};

	} // namespace Common
} // namespace VGUGV
