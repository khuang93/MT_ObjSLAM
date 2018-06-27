/*
 * frame.h
 *
 *  Created on: 26 Sep 2017
 *      Author: peidong
 */

#ifndef __SRC_COMMON_MEASUREMENT_FRAME_H_
#define __SRC_COMMON_MEASUREMENT_FRAME_H_

#include <memory>
#include "core/common/builtin_types.h"
#include "core/common/custom_types.h"
#include "core/feature/featureBase.h"
#include "core/sensor/cameraBase.h"
#include "core/common/macro.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

namespace VGUGV
{
	namespace Common
	{

		class FeatureBase; // break circular inclusion with FeatureBase.h

		class Frame
		{
		  public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		  public:
			const static int nFrameIlluminationVariableDimension = 2;

		  public:
			Frame();
			~Frame();

		  public:
			void setCamera(const CameraBase *camera);
			void setExposure(unsigned int exposure_us);
			void setGain(unsigned int gain);
			void setImage(const ImageType &image);
			void setDepthImage(const Image<float>& depthmap);
			void setSeq(unsigned long long seq);
			void setTimeStamp(long long timestamp);
			void setIlluminationParams(double a, double b);
			void fixLinearizationPoint();

		  public:
			unsigned long getSeq();

		  public:
			void addFeature(FeatureBase *newFeature);

		  public:
			const CameraBase *getCamera() const;
			unsigned int getExposureTime() const;
			unsigned int getGain() const;
			Eigen::Vector2d getIlluminationParams() const;

			const ImageType *getImage() const;
			ImageType *getImage();

			const Image<float>* getDepthImage() const;
			Image<float>* getDepthImage();

			ImageElementType getImageElement(int r, int c) const;
			int getMaxPyramidLevel() const;
			const ImageType* getPyramidImage(int level) const;
			ImageType *getPyramidImage(int level);

			const Image32F1C *getPyramidImageGradientMag(int level) const;
			const Image<Vector2f> *getPyramidImageGradientVec(int level) const;

			long long getTimeStamp() const;
			int getTotalNumberOfFeatures() const;
			const FeatureBase *getFeature(int index) const;
			const std::vector<FeatureBase *> &getAllFeatures() const;

			bool isLinearizationPointFixed();

			Vector2d getDeltaEstIllu2LinearizationPoint();

		  public:
			void updateIlluminationParameters(const Common::Vector2d &delta_ab);
			Vector2d getUpdateIlluminationParams() const;

		  public:
			void computeImagePyramids(int nLevels = 1);
			void computeImagePyramidsGradients(int nLevels = 1);

		  private:
			long long mTimeStamp;
			unsigned long long mSeq;
			unsigned int mExposureTime_us;
			unsigned int mGain_dB;

		  private:
			ImageType *mImage; // raw single channel image
			Image<float>* mDepthImage; // depth map in meters
			const CameraBase *mCamera;
			std::vector<FeatureBase*> mFeatures;

		  private:
			int mMaxPyramidLevel;
			ImageType **mPyramidImages;
			Image32F1C **mPyramidImageGradientMag; // dx * dx + dy * dy
			Image<Eigen::Vector2f> **mPyramidImageGradientVec; // dx, dy

		  private:
			bool mbLinearizationPointIsFixed;
			double ma, mb; // illumination parameters, which will be used and refined by optimizer
			double maZero, mbZero;
			double mdelta_a, mdelta_b;
		};

	} /* namespace Common */
} /* namespace VGUGV */

#endif /* SRC_COMMON_MEASUREMENT_FRAME_H_ */
