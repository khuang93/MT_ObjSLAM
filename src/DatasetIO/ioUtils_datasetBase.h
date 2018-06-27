#ifndef _VGUGV_IO_UTILS_DATASET_BASE_
#define _VGUGV_IO_UTILS_DATASET_BASE_

#include <core/sensor/imuSensorModel.h>
#include <memory>
#include <string>
#include <queue>
#include <Eigen/Dense>

#include "core/measurement/CombinedVioMeasurements.h"
#include "core/sensor/cameraBase.h"
#include "core/sensor/pinholeCameraModel.h"
#include "core/sensor/unifiedCameraModel.h"
#include "core/sensor/sensorSystemConfig.h"
#include "core/common/structs.h"
#include "core/transformation/transformation.h"


namespace VGUGV
{
  namespace Utils
  {
    class IOUtils_datasetBase {
    public:
        typedef std::shared_ptr<IOUtils_datasetBase> Ptr;

    public:
		virtual bool initDataReader4CameraLogReading(Common::SensorSystemConfig* systemSensorConfig, const std::vector<std::string> &camLogFileNames) = 0;

		virtual bool initDataReader4GroundTruthLogReading(const std::string &groundTruthPoseFileName) = 0;

		virtual bool initDataReader4IMULogReading(const std::string &ImuLogFileName) = 0;

		virtual bool initDataReader4MotionTrajLogReading(const std::string &motionTrajFileName) = 0;

		virtual bool initDataReader4DepthLogReading(const std::string& fileName) = 0;

		virtual bool loadImuCalibrations(const std::string& imuCalibFile) = 0;

        virtual bool loadCameraCalibrations(const std::string& camCalibFileName, const std::string& camExtrinsicCalibFileName) = 0;

        virtual void loadVignettingImage(const std::string &fileName, CameraBase* camera) = 0;

    public:
        virtual CameraBase* getCamera(const std::string &devName) = 0;

        virtual ImuSensorModel* getImu(const std::string &devName) = 0;

	public:
		virtual void skipNframes(int nFrames2skip) = 0;
		virtual bool getSyncedMultiFrames(const std::unordered_map<std::string, const CameraBase*>& camNameMap, Common::MultiFrames* multiFrames, Common::ImageQualityType imgType = Common::IMAGE_SHARP) = 0;
		virtual bool getSyncedImuMeasurements(ImuSensorModel* imuSensorModel, long long tillTimeStampInNs, std::queue<Common::ImuMeasurement>& imuMeasurements) = 0;
		virtual bool getSyncedGroundTruthPoseVel(long long tillTimeStampInNs, Common::Transformation& pose, Eigen::Vector3d& velocity) = 0;
		virtual void getMotionTrajectory(long long tillTimeStampIn_ns, long long exposureTimeIn_ns, std::vector<Common::Transformation, Eigen::aligned_allocator<Common::Transformation>>& trajectory) = 0;
		virtual bool getSyncedDepthImages(const std::unordered_map<std::string, const CameraBase*>& camNameMap, long long timestamp_in_ns, Common::MultiFrames* multiFrames) = 0;
    };
  }
}

#endif
