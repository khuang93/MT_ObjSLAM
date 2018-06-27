/*
 * frame.cpp
 *
 *  Created on: 26 Sep 2017
 *      Author: peidong
 */

#include <core/measurement/frame.h>
#include "core/feature/featurePatch.h"

namespace VGUGV {
	namespace Common {

		Frame::Frame()
			: mTimeStamp(0)
			, mSeq(0)
			, mExposureTime_us(0)
			, mGain_dB(0)
			, mImage(nullptr)
			, mDepthImage(nullptr)
			, mMaxPyramidLevel(0)
			, ma(1.0)
			, mb(0.0)
			, mdelta_a(0)
			, mdelta_b(0)
			, mbLinearizationPointIsFixed(false)
			, maZero(1)
			, mbZero(0)
		    , mPyramidImages(nullptr)
		    , mPyramidImageGradientMag(nullptr)
		    , mPyramidImageGradientVec(nullptr)
		{
		}

		Frame::~Frame()
		{
			delete mImage;
			delete mDepthImage;
			for(int i = 0; i < mMaxPyramidLevel; i++)
			{
				delete mPyramidImages[i];
				delete mPyramidImageGradientMag[i];
				delete mPyramidImageGradientVec[i];
			}
			delete mPyramidImages;
			delete mPyramidImageGradientMag;
			delete mPyramidImageGradientVec;

			for(auto& feature : mFeatures)
			{
				delete feature;
			}
		}

		void Frame::setCamera(const CameraBase *camera)
		{
			mCamera = camera;
		}

		void Frame::setExposure(unsigned int exposure_us)
		{
			mExposureTime_us = exposure_us;
		}

		void Frame::setGain(unsigned int gain)
		{
			mGain_dB = gain;
		}

		void Frame::setImage(const ImageType& image)
		{
			mImage = new ImageType(true, false);
			mImage->SetFrom(image, CPU_TO_CPU);
		}

		void Frame::setDepthImage(const Image<float>& depthmap)
		{
			mDepthImage = new Image<float>(true, false);
			mDepthImage->SetFrom(depthmap, CPU_TO_CPU);
		}

		void Frame::setSeq(unsigned long long seq)
		{
			mSeq = seq;
		}

		void Frame::setTimeStamp(long long timestamp)
		{
			mTimeStamp = timestamp;
		}

		void Frame::setIlluminationParams(double a, double b)
		{
			ma = a;
			maZero = ma;
			mb = b;
			mbZero = mb;
		}

		void Frame::fixLinearizationPoint()
		{
			mbLinearizationPointIsFixed = true;
		}

		unsigned long Frame::getSeq()
		{
			return mSeq;
		}

		void Frame::addFeature(FeatureBase *newFeature)
		{
			mFeatures.push_back(newFeature);
		}

		const CameraBase *Frame::getCamera() const
		{
			return mCamera;
		}

		unsigned int Frame::getExposureTime() const
		{
			return mExposureTime_us;
		}

		unsigned int Frame::getGain() const
		{
			return mGain_dB;
		}

		Eigen::Vector2d Frame::getIlluminationParams() const
		{
			return Eigen::Vector2d(ma, mb);
		}

		const ImageType *Frame::getImage() const
		{
			return mImage;
		}

		ImageType *Frame::getImage()
		{
			return mImage;
		}

		const Image<float>* Frame::getDepthImage() const
		{
			return mDepthImage;
		}

		Image<float>* Frame::getDepthImage()
		{
			return mDepthImage;
		}

		ImageElementType Frame::getImageElement(int r, int c) const
		{
			return mImage->GetElement(r, c, Common::MEMORYDEVICE_CPU);
		}

		int Frame::getMaxPyramidLevel() const
		{
			return mMaxPyramidLevel;
		}

		const ImageType* Frame::getPyramidImage(int level) const
		{
			custom_assert(level < mMaxPyramidLevel);
			return mPyramidImages[level];
		}

		ImageType *Frame::getPyramidImage(int level)
		{
			custom_assert(level < mMaxPyramidLevel);
			return mPyramidImages[level];
		}

		const Image32F1C *Frame::getPyramidImageGradientMag(int level) const
		{
			return mPyramidImageGradientMag[level];
		}

		const Image<Vector2f> *Frame::getPyramidImageGradientVec(int level) const
		{
			return mPyramidImageGradientVec[level];
		}

		long long Frame::getTimeStamp() const
		{
			return mTimeStamp;
		}

		int Frame::getTotalNumberOfFeatures() const
		{
			return mFeatures.size();
		}

		const FeatureBase *Frame::getFeature(int index) const
		{
			return mFeatures.at(index);
		}

		const std::vector<FeatureBase *> &Frame::getAllFeatures() const
		{
			return mFeatures;
		}

		bool Frame::isLinearizationPointFixed()
		{
			return mbLinearizationPointIsFixed;
		}

		Vector2d Frame::getDeltaEstIllu2LinearizationPoint()
		{
			if (!mbLinearizationPointIsFixed) return Vector2d(0, 0);
			return Vector2d((ma - maZero),
							(mb - mbZero));
		}

		void Frame::updateIlluminationParameters(const Common::Vector2d &delta_ab)
		{
			ma += (delta_ab(0));
			mb += (delta_ab(1));

			mdelta_a = delta_ab(0);
			mdelta_b = delta_ab(1);

			if (!mbLinearizationPointIsFixed)
			{
				maZero = ma;
				mbZero = mb;
			}
		}

		Vector2d Frame::getUpdateIlluminationParams() const
		{
			return Vector2d(mdelta_a, mdelta_b);
		}

		void Frame::computeImagePyramids(int nLevels)
		{
			if (nLevels <= mMaxPyramidLevel) return;
			mMaxPyramidLevel = nLevels;

			int nRows = mCamera->getCameraHeight();
			int nCols = mCamera->getCameraWidth();

			// dynamically allocate memory
			mPyramidImages = new ImageType*[nLevels];

			// copy first level directly
			mPyramidImages[0] = new ImageType(nRows, nCols, true, false);
			mPyramidImages[0]->SetFrom(mImage, CPU_TO_CPU);

			for (int i = 1; i < nLevels; ++i)
			{
				nRows /= 2;
				nCols /= 2;

				mPyramidImages[i] = new ImageType(nRows, nCols, true, false);

				ImageType* pImageSrc = mPyramidImages[i - 1];
				ImageType* pImageDst = mPyramidImages[i];

				for (int r = 0; r < nRows; ++r)
				{
					const ImageElementType* pImageSrc0 = pImageSrc->GetData(Common::MEMORYDEVICE_CPU) + (r * 2) * pImageSrc->Width();
					const ImageElementType* pImageSrc1 = pImageSrc->GetData(Common::MEMORYDEVICE_CPU) + (r * 2 + 1) * pImageSrc->Width();
					ImageElementType *pImageDst0 = pImageDst->GetData(Common::MEMORYDEVICE_CPU) + r * nCols;

					for (int c = 0; c < nCols; ++c, pImageSrc0 += 2, pImageSrc1 += 2, ++pImageDst0)
					{
						*pImageDst0 = 0.25f * (pImageSrc0[0] + pImageSrc0[1] + pImageSrc1[0] + pImageSrc1[1]);
					}
				}
			}
		}

		void Frame::computeImagePyramidsGradients(int nLevels)
		{
			if (nLevels > mMaxPyramidLevel)
			{
				computeImagePyramids(nLevels);
				delete[] mPyramidImageGradientMag;
				mPyramidImageGradientMag = NULL;
			}
			if (mPyramidImageGradientMag != NULL) return;

			int nRows = mCamera->getCameraHeight();
			int nCols = mCamera->getCameraWidth();

			mPyramidImageGradientMag = new Image32F1C*[nLevels];
			mPyramidImageGradientVec = new Image<Eigen::Vector2f>*[nLevels];

			for (int i = 0; i < nLevels; i++)
			{
				const int scale = 1 << i;

				mPyramidImageGradientMag[i] = new Image32F1C(nRows, nCols, true, false);
				mPyramidImageGradientVec[i] = new Image<Eigen::Vector2f>(nRows, nCols, true, false);

				const ImageElementType*  pImage = mPyramidImages[i]->GetData(Common::MEMORYDEVICE_CPU);
				float*                   pGradientMag = mPyramidImageGradientMag[i]->GetData(Common::MEMORYDEVICE_CPU);
				Eigen::Vector2f*         pGradientVec = mPyramidImageGradientVec[i]->GetData(Common::MEMORYDEVICE_CPU);

				for (int r = 0; r < nRows; ++r)
				{
					if (r == 0 || r == nRows - 1)
					{
						for (int c = 0; c < nCols; ++c, ++pGradientMag, ++pGradientVec)
						{
							*pGradientMag = 0.0;
							pGradientVec->setZero();
						}

						pImage += nCols;
					}
					else
					{
						const int rInTopLevel = scale * r + scale - 1;

						for (int c = 0; c < nCols; ++c, ++pImage, ++pGradientMag, ++pGradientVec)
						{
							const int cInTopLevel = scale * c + scale - 1;

							if (c == 0 || c == nCols - 1 ||
								mCamera->isInImageMask(Eigen::Vector2i(cInTopLevel, rInTopLevel)))
							{
								*pGradientMag = 0.0;
								pGradientVec->setZero();
							}
							else
							{
								const float dx = 0.5f * (pImage[1] - pImage[-1]);
								const float dy = 0.5f * (pImage[nCols] - pImage[-nCols]);

								*pGradientMag = dx * dx + dy * dy;
								*pGradientVec << dx, dy;
							}
						}
					}
				} // end for r c

				nRows /= 2;
				nCols /= 2;
			} // end for i
		}

	} /* namespace Common */
} /* namespace VGUGV */
