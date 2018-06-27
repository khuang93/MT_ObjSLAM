/*
 * cameraBase.cpp
 *
 *  Created on: 20 Sep 2017
 *      Author: peidong
 */

#include "cameraBase.h"
#include "core/common/builtin_types.h"
#include "core/common/macro.h"
#include "core/nonMemberFunctions/randomNumberGenerator.h"
#include "core/nonMemberFunctions/interpolation.h"
#include "core/sensor/cameraDistortionFoV.h"
#include "core/sensor/cameraDistortionRadTan.h"

namespace VGUGV
{
    namespace Common
    {
        // constructors
        CameraBase::CameraBase(int height, int width, const Eigen::Matrix3f &K)
            : mCameraHeight(height),
              mCameraWidth(width),
              mK(K),
              mKinv(K.inverse()),
              mRays(NULL),
              mRays_CUDA(NULL),
              mParamScalingFactor(1.0),
              mImageScalingFactor(1.0),
              mImageIsGeometricUndistorted(false),
              mCameraMask(nullptr),
              mVignettingImage(nullptr),
              mUndistortionRemap(nullptr),
              mCameraDistortionModel(nullptr)
        {
        };

        CameraBase::~CameraBase()
        {
            delete mRays;
            delete mCameraMask;
            delete mVignettingImage;
            delete mUndistortionRemap;

#ifndef COMPILE_WITHOUT_CUDA
            if (mRays_CUDA != NULL)
            CUDA_SAFE_CALL(cudaFree(mRays_CUDA));
#endif
        }

        bool CameraBase::backProjectLookUpTableLevel0IntegerPos(int x,
                                                                int y,
                                                                Eigen::Vector3f &ray) const
        {
            if (mRays == NULL)
            {
                mRays = new Eigen::Vector3f[mCameraHeight * mCameraWidth];
                for (int r = 0; r < mCameraHeight; r++)
                {
                    for (int c = 0; c < mCameraWidth; c++)
                    {
                        int index = r * mCameraWidth + c;
                        Eigen::Vector2f pixel;
                        pixel << c, r;
                        Eigen::Vector3f unitRay;
                        backProject(pixel, unitRay, 0);
                        mRays[index] = unitRay;
                    }
                }
            }
            int index = y * mCameraWidth + x;
            ray = mRays[index];
            return true;
        }

        void CameraBase::kNormalizedPixelXy(const Vector2f &pixelXy,
                                            Vector2f &normalizedPixelXy)
        {
            normalizedPixelXy(0) = mKinv(0, 0) * pixelXy(0) + mKinv(0, 2);
            normalizedPixelXy(1) = mKinv(1, 1) * pixelXy(1) + mKinv(1, 2);
        }

        void CameraBase::kUnNormalizedPixelXy(const Vector2f &normalizedPixelXy,
                                              Vector2f &pixelXy)
        {
            pixelXy(0) = mK(0, 0) * normalizedPixelXy(0) + mK(0, 2);
            pixelXy(1) = mK(1, 1) * normalizedPixelXy(1) + mK(1, 2);
        }

        void CameraBase::loadInverseResponseFunction(const std::string &fileName)
        {
            if (!mInverseResponseFunction.read(fileName))
            {
                std::cout << __FUNCTION__
                          << ": Failed to load inverse response function file "
                          << fileName << "\n";
                return;
            }
        }

        void CameraBase::undistort_photometric(const Image16U1C &inputImage, Image32F1C &outImage)
        {
            const size_t numPixels = inputImage.Size();

            const uint16_t *inputData = inputImage.GetData(MEMORYDEVICE_CPU);
            float *outputData = outImage.GetData(MEMORYDEVICE_CPU);

            const float *vignettingData = nullptr;
            if (mVignettingImage != nullptr)
            {
                vignettingData = mVignettingImage->GetData(MEMORYDEVICE_CPU);
            }

            if (mInverseResponseFunction.size() == 0)
            {
                for (size_t i = 0; i < numPixels; ++i, ++inputData, ++outputData)
                {
                    const uint16_t rawIntensity = *inputData;

                    float undistortedIntensity = static_cast<float>(rawIntensity);

                    if (mVignettingImage != nullptr)
                    {
                        undistortedIntensity /= (*vignettingData);
                        ++vignettingData;
                    }

                    *outputData = undistortedIntensity;
                }
            } else
            {
                for (size_t i = 0; i < numPixels; ++i, ++inputData, ++outputData)
                {
                    const uint16_t rawIntensity = *inputData;

                    float undistortedIntensity = static_cast<float>(mInverseResponseFunction[rawIntensity]);

                    if (mVignettingImage != nullptr)
                    {
                        undistortedIntensity /= (*vignettingData);
                        ++vignettingData;
                    }

                    *outputData = undistortedIntensity;
                }
            }
        }

        void CameraBase::initUndistortLens(void)
        {
            if (mUndistortionRemap == nullptr)
            {
                mUndistortionRemap = new Image32F2C(mCameraHeight, mCameraWidth, true, false);
                for (int r = 0; r < mCameraHeight; r++)
                {
                    for (int c = 0; c < mCameraWidth; c++)
                    {
                        Vector2f normalizedPixelXy;
                        kNormalizedPixelXy(Vector2f(c, r), normalizedPixelXy);

                        Vector2f distortedNormalizedPixelXy;
                        mCameraDistortionModel->distort(normalizedPixelXy,
                                                        distortedNormalizedPixelXy);

                        Vector2f distortedPixelXy;
                        kUnNormalizedPixelXy(distortedNormalizedPixelXy,
                                             distortedPixelXy);

                        if (distortedPixelXy(0) < 0.01) distortedPixelXy(0) = 0.01;
                        if (distortedPixelXy(1) < 0.01) distortedPixelXy(1) = 0.01;
                        if (distortedPixelXy(0) > mCameraWidth - 1.01)
                            distortedPixelXy(0) = mCameraWidth - 1.01;
                        if (distortedPixelXy(1) > mCameraHeight - 1.01)
                            distortedPixelXy(1) = mCameraHeight - 1.01;
                        mUndistortionRemap->SetElement(r, c, distortedPixelXy, Common::MEMORYDEVICE_CPU);
                    }
                }
            }
            mImageIsGeometricUndistorted = true;
        }

        void CameraBase::undistort_lens(const Image32F1C &inputImage, Image32F1C &outImage)
        {
            const size_t numPixels = inputImage.Size();
            const Vector2f *undistortionRemapData = mUndistortionRemap->GetData(Common::MEMORYDEVICE_CPU);
            float *outputData = outImage.GetData(Common::MEMORYDEVICE_CPU);

            for (size_t i = 0; i < numPixels; ++i, ++undistortionRemapData, ++outputData)
            {
                const Vector2f distortedPixelXy = *undistortionRemapData;
                if (isInImage(distortedPixelXy))
                {
                    *outputData = Common::bilinearInterpolation<float, float>(
                        inputImage.GetData(Common::MEMORYDEVICE_CPU),
                        mCameraWidth,
                        distortedPixelXy(0),
                        distortedPixelXy(1));
                } else
                {
                    *outputData = 0.0f;
                }
            }
        }

        void CameraBase::setCameraMask(const Image8U1C &mask)
        {
            mCameraMask = new Image8U1C(true, false);
            mCameraMask->SetFrom(mask, CPU_TO_CPU);
        }

        void CameraBase::setParamScalingFactor(float paramScalingFactor)
        {
            mParamScalingFactor = paramScalingFactor;
            mK(0, 0) *= paramScalingFactor;
            mK(0, 2) *= paramScalingFactor;
            mK(1, 1) *= paramScalingFactor;
            mK(1, 2) *= paramScalingFactor;
            mKinv = mK.inverse();

            mCameraHeight = mCameraHeight * paramScalingFactor;
            mCameraWidth = mCameraWidth * paramScalingFactor;
        }

        void CameraBase::setImageScalingFactor(const float imageScalingFactor)
        {
            mImageScalingFactor = imageScalingFactor;
        }

        void CameraBase::setVignettingImage(const Image32F1C &image)
        {
            mVignettingImage = new Image32F1C(true, false);
            mVignettingImage->SetFrom(image, Common::CPU_TO_CPU);
        }

        void CameraBase::Create_dist_model(CameraDistortionType distType, float k1_w, float k2, float p1, float p2)
        {
            switch (distType)
            {
                case CameraDistortionType::FoV:
                {
                    mCameraDistortionModel = new CameraDistortionFoV(k1_w);
                    break;
                }
                case CameraDistortionType::RAD_TAN:
                {
                    mCameraDistortionModel = new CameraDistortionRadTan(k1_w, k2, p1, p2);
                    break;
                }
            }
        }

        // retrieve
        int CameraBase::getCameraWidth() const
        {
            return mCameraWidth;
        }

        int CameraBase::getCameraHeight() const
        {
            return mCameraHeight;
        }

        float CameraBase::getParamScalingFactor() const
        {
            return mParamScalingFactor;
        }

        float CameraBase::getImageScalingFactor() const
        {
            return mImageScalingFactor;
        }

        CameraModelType CameraBase::getCameraModelType() const
        {
            return mCameraModelType;
        }

        CameraDistortionBase *CameraBase::getCameraDistortionModel() const
        {
            return mCameraDistortionModel;
        }

        Image8U1C *CameraBase::getCameraMask() const
        {
            return mCameraMask;
        }

        Image32F1C *CameraBase::getVignettingImage() const
        {
            return mVignettingImage;
        }

        Cuda::CudaCamera CameraBase::getCudaCamera() const
        {
#ifndef COMPILE_WITHOUT_CUDA
            Cuda::CudaCamera cuda_camera;
            // get camera params
            std::vector<double> cameraParams = this->getCameraParams();
            std::vector<double> cameraDistParams;

            CameraDistortionBase* cameraDistortionModel = this->getCameraDistortionModel();
            if (cameraDistortionModel != nullptr)
            {
                cameraDistParams = cameraDistortionModel->getDistortionParams();
            }

            CameraDistortionType distType = Common::ZERO;
            float4 distParams = make_float4(0, 0, 0, 0);

            if (cameraDistortionModel != nullptr)
            {
                switch (cameraDistortionModel->getDistortionType())
                {
                    case VGUGV::Common::FoV:
                    {
                        distType = VGUGV::Common::FoV;
                        distParams = make_float4(cameraDistParams[0], 0, 0, 0);
                        break;
                    }
                    case VGUGV::Common::RAD_TAN:
                    {
                        distType = VGUGV::Common::RAD_TAN;
                        distParams = make_float4(cameraDistParams[0], cameraDistParams[1], cameraDistParams[2], cameraDistParams[3]);
                        break;
                    }
                    default: break;
                }
            }

            float xi = 0;
            if (this->getCameraModelType() == VGUGV::Common::UNIFIED)
            {
                xi = cameraParams[4];
            }

            cuda_camera.mCameraModel = this->getCameraModelType();
            cuda_camera.mProjectionParams = make_float4(cameraParams[0], cameraParams[1], cameraParams[2], cameraParams[3]);
            cuda_camera.mCameraDistortion = distType;
            cuda_camera.mDistortionParams = distParams;
            cuda_camera.mXi = xi;
            return cuda_camera;
#endif
        }

        void CameraBase::getK(Eigen::Matrix3f &K) const
        {
            K = mK;
        }

        void CameraBase::getK(float &fx, float &fy, float &cx, float &cy) const
        {
            fx = mK(0, 0);
            fy = mK(1, 1);
            cx = mK(0, 2);
            cy = mK(1, 2);
        }

        Eigen::Matrix3f CameraBase::getK() const
        {
            return mK;
        }

        void CameraBase::getKinv(Eigen::Matrix3f &Kinv) const
        {
            Kinv = mKinv;
        }

        void CameraBase::getKinv(float &fxInv, float &fyInv, float &cxInv, float &cyInv) const
        {
            fxInv = mKinv(0, 0);
            fyInv = mKinv(1, 1);
            cxInv = mKinv(0, 2);
            cyInv = mKinv(1, 2);
        }

        Eigen::Matrix3f CameraBase::getKinv() const
        {
            return mKinv;
        };

        Eigen::Vector3f *CameraBase::getRayPtrs() const
        {
            // make sure look-up table has been created...
            Eigen::Vector3f temp;
            backProjectLookUpTableLevel0IntegerPos(0, 0, temp);
            return mRays;
        }

#ifndef COMPILE_WITHOUT_CUDA
        Vec3<float> *CameraBase::getRayPtrsCuda() const
        {
            // make sure look-up table has been created...
            Eigen::Vector3f temp;
            backProjectLookUpTableLevel0IntegerPos(0, 0, temp);

            if (mRays_CUDA == NULL)
            {
                // According to https://eigen.tuxfamily.org/dox/classEigen_1_1Matrix.html
                // A fixed size Matrix is actually just an array, so we can copy the data directly in one step
                const size_t vecSize = sizeof(Vec3<float>);
                const size_t eigenVecSize = sizeof(Eigen::Vector3f);
                custom_assert_msg(vecSize == eigenVecSize,
                                  std::string("Size of Common::Vec3<float> and Eigen::Vector3f does not match"));
                const size_t numElems = mCameraHeight * mCameraWidth;

                // Allocate memory on GPU
                CUDA_SAFE_CALL(cudaMalloc(&mRays_CUDA, vecSize * numElems));

                CUDA_SAFE_CALL(cudaMemcpy(mRays_CUDA, mRays, vecSize * numElems, cudaMemcpyHostToDevice));
            }

            return mRays_CUDA;
        }
#endif

        void CameraBase::scaleProjK(Common::Matrix3x3f &K, int pyramidLevel)
        {
            K = mK;
            if (pyramidLevel == 0) return;
            int scale = 1 << pyramidLevel;
            float iscale = 1.0 / scale;
            K(0, 0) *= iscale;
            K(1, 1) *= iscale;
            K(0, 2) *= iscale;
            K(1, 2) *= iscale;
        }

        void CameraBase::scaleProjKinv(Common::Matrix3x3f &Kinv, int pyramidLevel)
        {
            Kinv = mKinv;
            if (pyramidLevel == 0) return;
            int scale = 1 << pyramidLevel;
            Kinv(0, 0) *= scale;
            Kinv(1, 1) *= scale;
        }

        void CameraBase::test_projectAndUnprojectFunctions(int pyramidLevel)
        {
            double X = Common::randomFloat(10);
            double Y = Common::randomFloat(10);
            double Z = fabs(Common::randomFloat(10)) + 1;

            Vector2d xy_double;
            Vector2f xy_float;
            Vector3d xyz_double;
            Vector3f xyz_float;

            bool projectionIsGood = true;

            projectionIsGood &=
                this->project(Vector3f(X, Y, Z), xy_float, pyramidLevel);
            projectionIsGood &=
                this->backProject(xy_float, xyz_float, pyramidLevel);

            projectionIsGood &= this->project(Vector3f(X, Y, Z).cast<double>(),
                                              xy_double,
                                              pyramidLevel);
            projectionIsGood &=
                this->backProject(xy_double, xyz_double, pyramidLevel);

            if (projectionIsGood == false)
            {
                std::cout << __FUNCTION__
                          << " projection error, re-sample data...\n";
                return;
            }

            Vector3f oXYZ(X, Y, Z);
            oXYZ.normalize();

            if ((xyz_double - oXYZ.cast<double>()).norm() > 0.001)
            {
                std::cout << "xy_double : " << xy_double.transpose() << "\n";
                std::cout << "xyz_double: " << xyz_double.transpose() << "\n";
                std::cout << "XYZ       : " << oXYZ.transpose() << "\n";
                std::cout << __FUNCTION__ << " Double: Fails...\n";
            } else std::cout << __FUNCTION__ << " Double: Passes...\n";

            if ((xyz_float - oXYZ).norm() > 0.001)
            {
                std::cout << "xy_double : " << xy_float.transpose() << "\n";
                std::cout << "xyz_double: " << xyz_float.transpose() << "\n";
                std::cout << "XYZ       : " << oXYZ.transpose() << "\n";
                std::cout << __FUNCTION__ << " float: Fails...\n";
            } else std::cout << __FUNCTION__ << " float: Passes...\n";
        }

        void CameraBase::test_projectionJacobian(int pyramidLevel)
        {
            double epsilon = 1e-3;
            double X = Common::randomFloat(10);
            double Y = Common::randomFloat(10);
            double Z = fabs(Common::randomFloat(10)) + 1;

            Matrix2x3f jacobian_numerical;
            Eigen::Vector2f xy_left, xy_right;

            bool projectionGood = true;
            // dx/dX dy/dX
            projectionGood &= this->project(Eigen::Vector3f(X + epsilon, Y, Z),
                                            xy_left,
                                            pyramidLevel);
            projectionGood &= this->project(Eigen::Vector3f(X - epsilon, Y, Z),
                                            xy_right,
                                            pyramidLevel);
            jacobian_numerical.col(0) = (xy_left - xy_right) / (2.0 * epsilon);

            // dx/dY dy/dY
            projectionGood &= this->project(Eigen::Vector3f(X, Y + epsilon, Z),
                                            xy_left,
                                            pyramidLevel);
            projectionGood &= this->project(Eigen::Vector3f(X, Y - epsilon, Z),
                                            xy_right,
                                            pyramidLevel);
            jacobian_numerical.col(1) = (xy_left - xy_right) / (2.0 * epsilon);

            // dx/dZ dy/dZ
            projectionGood &= this->project(Eigen::Vector3f(X, Y, Z + epsilon),
                                            xy_left,
                                            pyramidLevel);
            projectionGood &= this->project(Eigen::Vector3f(X, Y, Z - epsilon),
                                            xy_right,
                                            pyramidLevel);
            jacobian_numerical.col(2) = (xy_left - xy_right) / (2.0 * epsilon);

            if (projectionGood == false)
            {
                std::cout << __FUNCTION__
                          << " projection error, re-sample data...\n";
                return;
            }

            Matrix2x3f jacobian_analytical;
            this->projectionJacobian(Eigen::Vector3f(X, Y, Z),
                                     pyramidLevel,
                                     jacobian_analytical);

            double error = (jacobian_numerical - jacobian_analytical).norm()
                / (jacobian_numerical + jacobian_analytical).norm();

            std::cout << __FUNCTION__ << ": jacobian_numerical: \n"
                      << jacobian_numerical << "\n";
            std::cout << __FUNCTION__ << ": jacobian_analytical: \n"
                      << jacobian_analytical << "\n";
            std::cout << __FUNCTION__ << ": error ratio: " << error << "\n\n";
        }

        void CameraBase::test_projectionJacobianDouble(int pyramidLevel)
        {
            double epsilon = 1e-6;
            double X = Common::randomFloat(10);
            double Y = Common::randomFloat(10);
            double Z = fabs(Common::randomFloat(10)) + 1;

            Matrix2x3d jacobian_numerical;
            Eigen::Vector2d xy_left, xy_right;

            bool projectionGood = true;
            // dx/dX dy/dX
            projectionGood &= this->project(Eigen::Vector3d(X + epsilon, Y, Z),
                                            xy_left,
                                            pyramidLevel);
            projectionGood &= this->project(Eigen::Vector3d(X - epsilon, Y, Z),
                                            xy_right,
                                            pyramidLevel);
            jacobian_numerical.col(0) = (xy_left - xy_right) / (2.0 * epsilon);

            // dx/dY dy/dY
            projectionGood &= this->project(Eigen::Vector3d(X, Y + epsilon, Z),
                                            xy_left,
                                            pyramidLevel);
            projectionGood &= this->project(Eigen::Vector3d(X, Y - epsilon, Z),
                                            xy_right,
                                            pyramidLevel);
            jacobian_numerical.col(1) = (xy_left - xy_right) / (2.0 * epsilon);

            // dx/dZ dy/dZ
            projectionGood &= this->project(Eigen::Vector3d(X, Y, Z + epsilon),
                                            xy_left,
                                            pyramidLevel);
            projectionGood &= this->project(Eigen::Vector3d(X, Y, Z - epsilon),
                                            xy_right,
                                            pyramidLevel);
            jacobian_numerical.col(2) = (xy_left - xy_right) / (2.0 * epsilon);

            if (projectionGood == false)
            {
                std::cout << __FUNCTION__
                          << " projection error, re-sample data...\n";
                return;
            }

            Matrix2x3d jacobian_analytical;
            this->projectionJacobian(Eigen::Vector3d(X, Y, Z),
                                     pyramidLevel,
                                     jacobian_analytical);

            double error = (jacobian_numerical - jacobian_analytical).norm()
                / (jacobian_numerical + jacobian_analytical).norm();

            std::cout << __FUNCTION__ << ": jacobian_numerical: \n"
                      << jacobian_numerical << "\n";
            std::cout << __FUNCTION__ << ": jacobian_analytical: \n"
                      << jacobian_analytical << "\n";
            std::cout << __FUNCTION__ << ": error ratio: " << error << "\n\n";
        }

        void CameraBase::test_backprojectLookUpTable()
        {
            Vector3f *rays = getRayPtrs();
            custom_assert(rays != NULL);
            for (int r = 0; r < mCameraHeight; r++)
            {
                for (int c = 0; c < mCameraWidth; c++)
                {
                    Vector2f pixel(c, r);
                    Vector3f ray;
                    backProject(pixel, ray, 0);
                    int index = r * mCameraWidth + c;
                    custom_assert((ray - rays[index]).norm() < 1e-10);
                }
            }
            std::cout << __FUNCTION__ << " passed for camera " << mDeviceID << "\n";
        }
    } /* name space Common */
} /* Name space VGUGV */

