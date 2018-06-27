/*
 * systemState.cpp
 *
 *  Created on: 26 Sep 2017
 *      Author: peidong
 */

#include <core/measurement/CombinedVioMeasurements.h>

namespace VGUGV
{
	namespace Common
	{

		CombinedVioMeasurements::CombinedVioMeasurements()
		{
			mIsKeyFrame = false;
			mMultiFrames = nullptr;
			mImuPIM = nullptr;
		}

		CombinedVioMeasurements::~CombinedVioMeasurements()
		{
			delete mImuPIM;
			delete mMultiFrames;
			for (auto& residual : mResiduals_2d3d)
			{
				delete residual;
			}
		}

		void CombinedVioMeasurements::CreateMultiFrames(size_t nFrames)
		{
			mMultiFrames = new MultiFrames(nFrames);
		}

		void CombinedVioMeasurements::setImuPIM(ImuPreintegrationManifold* imuPIM)
		{
			mImuPIM = imuPIM;
		}

		void CombinedVioMeasurements::setAsKeyFrame(bool bIsKeyframe)
		{
			std::lock_guard<std::mutex> lock(mMutexKF);
			mIsKeyFrame = bIsKeyframe;
		}

		void CombinedVioMeasurements::Set_T_refKF2curFrame(Transformation& T)
		{
			mT_refKF2curFrame = T;
		}

		std::vector<std::pair<ResidualBase*, ImuPreintegrationManifold*> > CombinedVioMeasurements::getConnectedFriendInertialResiduals()
		{
			std::vector<std::pair<ResidualBase *, ImuPreintegrationManifold*> > temp;
			for (auto it = mInertialResidualsFromFriends.begin(); it != mInertialResidualsFromFriends.end(); ++it)
			{
				std::vector<std::pair<ResidualBase *, ImuPreintegrationManifold*> > inertialResiduals = it->second;
				temp.insert(temp.end(), inertialResiduals.begin(), inertialResiduals.end());
			}
			return temp;
		}

		ImuPreintegrationManifold* CombinedVioMeasurements::getImuPIM()
		{
			return mImuPIM;
		}

		MultiFrames *CombinedVioMeasurements::getMultiFrames() const
		{
			return mMultiFrames;
		}

		NavState *CombinedVioMeasurements::getNavState()
		{
			return &mNavState;
		}

		const NavState *CombinedVioMeasurements::getNavState() const
		{
			return &mNavState;
		}

		NavState *CombinedVioMeasurements::getGroundTruthNavState()
		{
			return &mGroundTruthNavState;
		}

		const NavState *CombinedVioMeasurements::getGroundTruthNavState() const
		{
			return &mGroundTruthNavState;
		}

		const Transformation& CombinedVioMeasurements::Get_T_refKF2curFrame() const
		{
			return mT_refKF2curFrame;
		}

		bool CombinedVioMeasurements::isKeyFrame()
		{
			std::lock_guard<std::mutex> lock(mMutexKF);
			return mIsKeyFrame;
		}

		void CombinedVioMeasurements::cleanUpBeforeDestruction()
		{
			// remove residuals owned by friends but connected to me
			for (auto it = mVisualResidualsFromFriends.begin(); it != mVisualResidualsFromFriends.end(); ++it)
			{
				it->first->removeFriend(this);
				std::vector<std::pair<ResidualBase*, FeatureBase*> > visualResiduals = it->second;
				for (auto it = visualResiduals.begin(); it != visualResiduals.end(); it++)
				{
					if (it->second != nullptr && it->first != nullptr) it->second->removeResidual(it->first);
				}
			}
			mVisualResidualsFromFriends.clear();

			for (auto it = mInertialResidualsFromFriends.begin(); it != mInertialResidualsFromFriends.end(); ++it)
			{
				it->first->removeFriend(this);
				std::vector<std::pair<ResidualBase*, ImuPreintegrationManifold*> > inertialResiduals = it->second;
				for (auto it = inertialResiduals.begin(); it != inertialResiduals.end(); ++it)
				{
					if (it->second != nullptr && it->first != nullptr) it->second->removeInertialResidual();
				}
			}
			mInertialResidualsFromFriends.clear();

			// remove residuals records kept by friends, but these residuals owned by me
			for (auto it = mFriendsWhoHasRecords4myResiduals.begin(); it != mFriendsWhoHasRecords4myResiduals.end();
				 ++it)
			{
				(*it)->removeResidualRecordFromFriend(this);
				(*it)->removeFriend(this);
			}
		}

		void CombinedVioMeasurements::addFriend(CombinedVioMeasurements* friends)
		{
			mFriendsWhoHasRecords4myResiduals.insert(friends);
		}

		void CombinedVioMeasurements::removeFriend(CombinedVioMeasurements* _friend)
		{
			mFriendsWhoHasRecords4myResiduals.erase(_friend);
		}

		void CombinedVioMeasurements::addFriendInertialResidual(CombinedVioMeasurements* friends,
															   ResidualBase* residual,
															   ImuPreintegrationManifold* friendPIM)
		{
			if (mInertialResidualsFromFriends.count(friends) == 0)
			{
				std::vector<std::pair<ResidualBase*, ImuPreintegrationManifold*> >
					newInertialResidualsEntryForFriend;
				newInertialResidualsEntryForFriend.push_back(std::make_pair(residual, friendPIM));
				// add entry
				mInertialResidualsFromFriends.insert(std::make_pair(friends, newInertialResidualsEntryForFriend));
				return;
			}
			mInertialResidualsFromFriends.at(friends).push_back(std::make_pair(residual, friendPIM));
		}

		void CombinedVioMeasurements::addFriendVisualResidual(CombinedVioMeasurements* friends,
															 Common::ResidualBase* friendResidual,
															 Common::FeatureBase* parentFeature)
		{
			if (mVisualResidualsFromFriends.count(friends) == 0)
			{
				// create new entry
				std::vector<std::pair<ResidualBase*, FeatureBase*> > newVisualResidualsEntry4Friend;
				newVisualResidualsEntry4Friend.push_back(std::make_pair(friendResidual, parentFeature));

				// add entry
				mVisualResidualsFromFriends.insert(std::make_pair(friends, newVisualResidualsEntry4Friend));
				return;
			}
			mVisualResidualsFromFriends.at(friends).push_back(std::make_pair(friendResidual, parentFeature));
		}

		void CombinedVioMeasurements::removeResidualRecordFromFriend(CombinedVioMeasurements* _friend)
		{
//    std::cout << __FUNCTION__ << " " << this->getSeqId()
//              << " is going to remove " << _friend->getSeqId() << " s residual records from "
//              << mVisualResidualsFromFriends.count(_friend) << " "
//              << mInertialResidualsFromFriends.count(_friend) << "\n";

			if (mVisualResidualsFromFriends.count(_friend) == 1) mVisualResidualsFromFriends.erase(_friend);
			if (mInertialResidualsFromFriends.count(_friend) == 1)
				mInertialResidualsFromFriends.erase(_friend);

//    std::cout << __FUNCTION__ << " " << this->getSeqId()
//              << " removed " << _friend->getSeqId() << " s residual records from "
//              << mVisualResidualsFromFriends.count(_friend) << " "
//              << mInertialResidualsFromFriends.count(_friend) << "\n";
		}

		void CombinedVioMeasurements::fixLinearizationPointNavState()
		{
			mNavState.fixLinearizationPoint();
		}

		void CombinedVioMeasurements::fixLinearizationPointIlluminationParams()
		{
			for (int i = 0; i < mMultiFrames->getTotalNumberOfFrames(); i++)
			{
				mMultiFrames->getFrame(i)->fixLinearizationPoint();
			}
		}

		void CombinedVioMeasurements::Add_2d3d_residuals(ResidualBase* residual)
		{
			mResiduals_2d3d.push_back(residual);
		}

		std::vector<ResidualBase*>& CombinedVioMeasurements::Get_2d3d_residuals()
		{
			return mResiduals_2d3d;
		}

	} /* namespace Common */
} /* namespace VGUGV */
