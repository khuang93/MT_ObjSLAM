#ifndef _VGUGV_COMMON_CAMERABASE_
#define _VGUGV_COMMON_CAMERABASE_

#include <core/common/cuda_defs.h>
#include "core/common/custom_types.h"
#include <core/common/enums.h>
#include "sensorBase.h"

#include <memory>
#include <vector>
#include <Eigen/Dense>

#ifdef HAVE_IPP
#include <ipp.h>
#endif

#include "cameraDistortionBase.h"
#include "Vision/Entity/InverseResponseFunction.h"
#include "core/common/builtin_types.h"
#include "core/math/Vector.h"
#include "cuda/cuda_camera.h"

namespace VGUGV
{
	namespace Common
	{
		class CameraBase : public SensorBase
		{
		  public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		  public:
			// constructors
			CameraBase(int height, int width, const Eigen::Matrix3f &K);
			virtual ~CameraBase();

		  public:
			virtual std::vector<double> getCameraParams() const = 0;
			virtual bool project(const Eigen::Vector3f &scenePoint, Eigen::Vector2f &pixelPoint, int pyramidLevel) const = 0;
			virtual bool project(const Eigen::Vector3d &scenePoint, Eigen::Vector2d &pixelPoint, int pyramidLevel) const = 0;
			virtual bool backProject(const Eigen::Vector2f &pixelPoint, Eigen::Vector3f &ray, int pyramidLevel) const = 0;
			virtual bool backProject(const Eigen::Vector2d &pixelPoint, Eigen::Vector3d &ray, int pyramidLevel) const = 0;
			virtual bool projectionJacobian(const Eigen::Vector3f &scenePoint, int pyramidLevel, Eigen::Matrix<float, 2, 3> &jacobian) const = 0;
			virtual bool projectionJacobian(const Eigen::Vector3d &scenePoint, int pyramidLevel, Eigen::Matrix<double, 2, 3> &jacobian) const = 0;

		  public:
			/**
             * \brief only valid for pyramid level 0 && integer pixel positions
             */
			bool backProjectLookUpTableLevel0IntegerPos(int x, int y, Eigen::Vector3f &ray) const;
			void kNormalizedPixelXy(const Vector2f &pixelXy, Vector2f &normalizedPixelXy);
			void kUnNormalizedPixelXy(const Vector2f &normalizedPixelXy, Vector2f &pixelXy);
			void loadInverseResponseFunction(const std::string &fileName);
			void setVignettingImage(const Image32F1C &image);

		  public:
			void undistort_photometric(const Image16U1C &inputImage, Image32F1C &outImage);
			void initUndistortLens(void);
			void undistort_lens(const Image32F1C &inputImage, Image32F1C &outImage);

		  public:
			/**
                \brief Checks if the given image point (c, r) or (x, y) lies within the image.
                \return True if the image point is within the image. False otherwise.
             */
			template<typename DerivedI>
			bool isInImage(const Eigen::MatrixBase<DerivedI> &imagePoint) const;
			/**
                \brief Checks if the given image point (c, r) or (x, y) lies within the image with certain margin.
                \return True if the image point is in the image. False otherwise.
             */
			template<typename DerivedI>
			bool isInImage(const Eigen::MatrixBase<DerivedI> &imagePoint, int margin) const;
			/**
                \brief Checks if the given image point (c, r) or (x, y) lies within the image mask.
                \return True if the image point is within the image mask. False otherwise.
             */
			template<typename DerivedI>
			bool isInImageMask(const Eigen::MatrixBase<DerivedI> &imagePoint) const;

		  public:
			void setCameraMask(const Image8U1C &mask);
			void setParamScalingFactor(float paramScalingFactor);
			void setImageScalingFactor(float imageScalingFactor);

		  public:
			void Create_dist_model(CameraDistortionType distType, float k1_w, float k2, float p1, float p2);

		  public:
			// retrieve
			int getCameraWidth() const;
			int getCameraHeight() const;
			void getK(Eigen::Matrix3f &K) const;
			void getK(float &fx, float &fy, float &cx, float &cy) const;
			Eigen::Matrix3f getK() const;
			void getKinv(Eigen::Matrix3f &Kinv) const;
			void getKinv(float &fxInv, float &fyInv, float &cxInv, float &cyInv) const;
			Eigen::Matrix3f getKinv() const;
			float getParamScalingFactor() const;
			float getImageScalingFactor() const;

			/**
             * \brief only valid to pyramid level 0 at integer positions
             */
			Eigen::Vector3f *getRayPtrs() const;
#ifndef COMPILE_WITHOUT_CUDA
			Vec3<float> *getRayPtrsCuda() const;
#endif
			CameraModelType getCameraModelType() const;
			CameraDistortionBase *getCameraDistortionModel() const;

		  public:
			Image8U1C *getCameraMask() const;
			Image32F1C *getVignettingImage() const;

			Cuda::CudaCamera getCudaCamera() const;

		  protected:
			void scaleProjK(Common::Matrix3x3f &K, int pyramidLevel);
			void scaleProjKinv(Common::Matrix3x3f &Kinv, int pyramidLevel);

		  protected:
			Eigen::Matrix3f mK;
			Eigen::Matrix3f mKinv;
			mutable Eigen::Vector3f *mRays;
			mutable Vec3<float> *mRays_CUDA;

			int mCameraWidth;
			int mCameraHeight;
			float mParamScalingFactor;
			float mImageScalingFactor;
			bool mImageIsGeometricUndistorted;
			CameraModelType mCameraModelType;

			CameraDistortionBase *mCameraDistortionModel;
			Image8U1C *mCameraMask;
			Image32F1C *mVignettingImage;
			Image32F2C *mUndistortionRemap;
			Vision::Entity::InverseResponseFunction mInverseResponseFunction;

		  public:
			void test_projectAndUnprojectFunctions(int pyramidLevel);
			void test_projectionJacobian(int pyramidLevel);
			void test_projectionJacobianDouble(int pyramidLevel);
			void test_backprojectLookUpTable();
		};

		template<typename DerivedI>
		bool CameraBase::isInImage(const Eigen::MatrixBase<DerivedI> &imagePoint) const
		{
			return (imagePoint(0) >= 0 && imagePoint(0) < mCameraWidth &&
				imagePoint(1) >= 0 && imagePoint(1) < mCameraHeight);
		}

		template<typename DerivedI>
		bool CameraBase::isInImage(const Eigen::MatrixBase<DerivedI> &imagePoint, int margin) const
		{
			return (imagePoint(0) >= margin && imagePoint(0) < mCameraWidth - margin &&
				imagePoint(1) >= margin && imagePoint(1) < mCameraHeight - margin);
		}

		template<typename DerivedI>
		bool CameraBase::isInImageMask(const Eigen::MatrixBase<DerivedI> &imagePoint) const
		{
			if (!isInImage(imagePoint)) return true;
			if (!mCameraMask) return false;
			int c = imagePoint(0);
			int r = imagePoint(1);
			return mCameraMask->GetElement(r, c, MEMORYDEVICE_CPU) < 100;
		}

	}
}

#endif
