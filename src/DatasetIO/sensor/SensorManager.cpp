//
// Created by peidong on 4/11/18.
//

#include "SensorManager.h"
#include "core/sensor/pinholeCameraModel.h"
#include "core/sensor/unifiedCameraModel.h"
#include "core/sensor/cameraDistortionFoV.h"
#include "core/sensor/cameraDistortionRadTan.h"

namespace VGUGV
{
    namespace Common
    {
        SensorManager::~SensorManager()
        {
          for (auto &cam : mNCameras)
          {
            delete cam;
          }
		  for (auto& imu : mNImus)
		  {
			  delete imu;
		  }
        }

        CameraBase *SensorManager::Create_camera(CameraModelType camType,
                                                 size_t height,
                                                 size_t width,
                                                 Eigen::Matrix3f &K,
                                                 const std::vector<float> &additionalParams)
        {
          switch (camType)
          {
            case CameraModelType::PINHOLE:
            {
              CameraBase *newCamera = new PinholeCameraModel(height, width, K);
              mNCameras.push_back(newCamera);
              break;
            }
            case CameraModelType::UNIFIED:
            {
              float xi = additionalParams[0];
              CameraBase *newCamera = new UnifiedCameraModel(height, width, K, xi);
              mNCameras.push_back(newCamera);
              break;
            }
            default:return nullptr;
          }
          return mNCameras.back();
        }

		ImuSensorModel* SensorManager::Create_IMU()
		{
			ImuSensorModel* imu = new ImuSensorModel();
			mNImus.push_back(imu);
			return imu;
		}
    }
}
