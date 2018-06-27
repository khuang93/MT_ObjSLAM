/*
 * sensorSystemConfig.h
 *
 *  Created on: 19 Sep 2017
 *      Author: peidong
 */

#ifndef SRC_COMMON_SENSOR_SENSORSYSTEMCONFIG_H_
#define SRC_COMMON_SENSOR_SENSORSYSTEMCONFIG_H_

#include <core/sensor/imuSensorModel.h>
#include <memory>
#include "core/common/enums.h"
#include "cameraBase.h"
#include "core/sensor/pinholeCameraModel.h"
#include "core/sensor/unifiedCameraModel.h"

namespace VGUGV
{
	namespace Common
	{
		class SensorSystemConfig
		{
		  public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		  public:
			void addCamera(int id, const CameraBase *camera, bool bRefCamera)
			{
				mNCameraIdMap.insert({id, camera});
				mNCameraNameMap.insert({camera->getDeviceName(), camera});
				mvAllCameras.push_back(camera);
				if (bRefCamera)
				{
					mvRefCameras.push_back(camera);
				}
			}

			void addIMU(ImuSensorModel* imu, bool refImu = true)
			{
				if (refImu)
				{
					mRefIMU = imu;
				} else
				{
					mvNonRefImus.push_back(imu);
				}
			}

			void addOverlapCameras(const CameraBase *refCamera, const std::vector<int> &auxCamerasID)
			{
				mIDMapOfOverlappingCameras.insert({refCamera, auxCamerasID});
			}

			void finalizeOverlapCameras()
			{
				for (auto it = mIDMapOfOverlappingCameras.begin(); it != mIDMapOfOverlappingCameras.end(); ++it)
				{
					const CameraBase *refCamera = it->first;
					std::vector<int> auxCamerasID = it->second;
					std::vector<const CameraBase *> auxCamerasObj;
					for (int i = 0; i < auxCamerasID.size(); ++i)
					{
						auxCamerasObj.push_back(mNCameraIdMap.at(auxCamerasID.at(i)));
					}
					mObjMapOfOverlappingCameras.insert({refCamera, auxCamerasObj});
				}
			}

		  public:
			std::unordered_map<int, const CameraBase *> mNCameraIdMap;
			std::unordered_map<std::string, const CameraBase *> mNCameraNameMap;
			std::unordered_map<const CameraBase *, std::vector<int> > mIDMapOfOverlappingCameras;
			std::unordered_map<const CameraBase *, std::vector<const CameraBase *> > mObjMapOfOverlappingCameras;

			std::vector<const CameraBase *> mvAllCameras;
			std::vector<const CameraBase *> mvRefCameras;

			ImuSensorModel* mRefIMU;
			std::vector<ImuSensorModel*> mvNonRefImus;
		};
	}
}



#endif /* SRC_COMMON_SENSOR_SENSORSYSTEMCONFIG_H_ */
