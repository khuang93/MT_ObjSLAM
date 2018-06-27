/*
 * multiFrames.cpp
 *
 *  Created on: 26 Sep 2017
 *      Author: peidong
 */

#include "core/measurement/multiFrames.h"

namespace VGUGV 
{
	namespace Common 
	{
		MultiFrames::MultiFrames(int nFrames)
		{
			mMultiFrames.resize(nFrames);
			mTimeStamp = 0;
		}

		void MultiFrames::setTimeStamp(long long timestamp)
		{
			mTimeStamp = timestamp;
		}

		void MultiFrames::setSeqId(long id)
		{
			mSeqId = id;
		}

		Frame* MultiFrames::getFrame(int index)
		{
			auto it = mMultiFrames.begin() + index;
			return &(*it);
		}

		const Frame* MultiFrames::getFrame(int index) const
		{
			auto it = mMultiFrames.begin() + index;
			return &(*it);
		}

		long long MultiFrames::getTimeStamp() const
		{
			return mTimeStamp;
		}

		long MultiFrames::getSeqId() const
		{
			return mSeqId;
		}

		int MultiFrames::getTotalNumberOfFrames() const
		{
			return mMultiFrames.size();
		}

	} /* namespace Common */
} /* namespace VGUGV */
