//
// Created by peidong on 4/11/18.
//

#ifndef AUTOVISION_VIO_SENSORMANAGER_H
#define AUTOVISION_VIO_SENSORMANAGER_H

#include <vector>
#include <Eigen/Dense>
#include "core/common/enums.h"
#include "core/sensor/cameraBase.h"
#include "core/sensor/cameraDistortionBase.h"
#include "core/sensor/imuSensorModel.h"

namespace VGUGV
{
  namespace Common
  {
    class SensorManager
    {
      public:
        static SensorManager& getInstance() { static SensorManager manager; return manager; };

        ~SensorManager();
        CameraBase* Create_camera(CameraModelType camType, size_t height, size_t width, Eigen::Matrix3f& K, const std::vector<float>& additionalParams);
		ImuSensorModel* Create_IMU();

      private:
        std::vector<CameraBase*> mNCameras;
		std::vector<ImuSensorModel*> mNImus;
    };
  }
}

#endif //AUTOVISION_VIO_SENSORMANAGER_H
