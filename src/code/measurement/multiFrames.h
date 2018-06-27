/*
 * multiFrames.h
 *
 *  Created on: 26 Sep 2017
 *      Author: peidong
 */

#ifndef SRC_COMMON_MEASUREMENT_MULTIFRAMES_H_
#define SRC_COMMON_MEASUREMENT_MULTIFRAMES_H_

#include <memory>
#include <vector>
#include "frame.h"

namespace VGUGV
{
	namespace Common
	{
		class MultiFrames
		{
		  public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		  public:
			explicit MultiFrames(int nFrames);

		  public:
			void setTimeStamp(long long timestamp);
			void setSeqId(long id);

		  public:
			Frame* getFrame(int index);
			const Frame* getFrame(int index) const;
			int getTotalNumberOfFrames() const;
			long long getTimeStamp() const;
			long getSeqId() const;

		  private:
			std::vector<Frame> mMultiFrames;
			long long mTimeStamp;
			long mSeqId;
		};

	} /* namespace Common */
} /* namespace VGUGV */

#endif /* SRC_COMMON_MEASUREMENT_MULTIFRAMES_H_ */
