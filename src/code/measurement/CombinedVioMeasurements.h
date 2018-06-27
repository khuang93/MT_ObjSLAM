/*
 * systemState.h
 *
 *  Created on: 26 Sep 2017
 *      Author: peidong
 */

#ifndef SRC_COMMON_MEASUREMENT_SYSTEMMEASUREMENTSTATE_H_
#define SRC_COMMON_MEASUREMENT_SYSTEMMEASUREMENTSTATE_H_

#include <memory>
#include <mutex>
#include "core/feature/featureBase.h"
#include "core/residual/residualBase.h"
#include "imuPreintegrationManifold.h"
#include "multiFrames.h"
#include "navState.h"

namespace VGUGV
{
	namespace Common
	{
		class CombinedVioMeasurements
		{
		  public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		  public:
            CombinedVioMeasurements();
			~CombinedVioMeasurements();

		  public:
			void CreateMultiFrames(size_t nFrames);

			void setImuPIM(ImuPreintegrationManifold* imuPIM);
			void setAsKeyFrame(bool bIsKeyframe);
			void Set_T_refKF2curFrame(Transformation& T);

		  public:
			std::vector<std::pair<ResidualBase*, ImuPreintegrationManifold*> > getConnectedFriendInertialResiduals();
			ImuPreintegrationManifold* getImuPIM();
			MultiFrames *getMultiFrames() const;
			NavState *getNavState();
			const NavState *getNavState() const;
			NavState *getGroundTruthNavState();
			const NavState *getGroundTruthNavState() const;
			const Transformation& Get_T_refKF2curFrame() const;

			bool isKeyFrame();

		  public:
			void cleanUpBeforeDestruction();

			void addFriend(CombinedVioMeasurements* friends);

			void addFriendInertialResidual(CombinedVioMeasurements* friends,
										   ResidualBase* residual,
										   ImuPreintegrationManifold* friendPIM);

			void addFriendVisualResidual(CombinedVioMeasurements* friends,
										 Common::ResidualBase* friendResidual,
										 Common::FeatureBase* parentFeature);

			void removeFriend(CombinedVioMeasurements* _friend);

			void removeResidualRecordFromFriend(CombinedVioMeasurements* _friend);

		  public:
			void fixLinearizationPointNavState();
			void fixLinearizationPointIlluminationParams();

		public:
			void Add_2d3d_residuals(ResidualBase* residual);
			std::vector<ResidualBase*>& Get_2d3d_residuals();

		  public:
			VisualOdometryTimeInMs mTimeConsumptions;

		  private:
			ImuPreintegrationManifold* mImuPIM;
			MultiFrames *mMultiFrames;

			NavState mNavState;
			NavState mGroundTruthNavState;
			Transformation mT_refKF2curFrame;

			std::mutex mMutexKF;

			bool mIsKeyFrame;

		  private:
			std::unordered_map<CombinedVioMeasurements *, std::vector<std::pair<ResidualBase*, FeatureBase*> > > mVisualResidualsFromFriends;
			std::unordered_map<CombinedVioMeasurements *, std::vector<std::pair<ResidualBase*, ImuPreintegrationManifold*> > > mInertialResidualsFromFriends;
			std::set<CombinedVioMeasurements *> mFriendsWhoHasRecords4myResiduals;
			std::vector<ResidualBase*> mResiduals_2d3d;
		};

	} /* namespace Common */
} /* namespace VGUGV */

#endif /* SRC_COMMON_MEASUREMENT_SYSTEMMEASUREMENTSTATE_H_ */
