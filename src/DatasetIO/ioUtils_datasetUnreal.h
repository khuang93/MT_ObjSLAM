#ifndef SRC_UTILS_IO_IOUTILS_DATASETUNREAL_H_
#define SRC_UTILS_IO_IOUTILS_DATASETUNREAL_H_

#include "ioUtils_datasetBase.h"
#include <stdio.h>
#include <unordered_map>
#include <fstream>

namespace VGUGV 
{
namespace Utils 
{

class IOUtils_datasetUnreal : public IOUtils_datasetBase
{
public:
	IOUtils_datasetUnreal()
	{
		mGroundTruthDataReader = NULL;
		mImuLogReader = NULL;
		mMotionTrajectoryDataReader = nullptr;
	}
	bool initDataReader4CameraLogReading(Common::SensorSystemConfig* systemSensorConfig, const std::vector<std::string>& camLogFileNames) override;
	bool initDataReader4GroundTruthLogReading(const std::string& groundTruthLogFileName) override;
	bool initDataReader4IMULogReading(const std::string& ImuLogFileName) override;
    bool initDataReader4MotionTrajLogReading(const std::string &motionTrajFileName) override;
	bool initDataReader4DepthLogReading(const std::string& fileName) override;

    bool loadImuCalibrations(const std::string& imuCalibFile);
	bool loadCameraCalibrations(const std::string& camCalibFileName, const std::string& camExtrinsicCalibFileName);
	void loadVignettingImage(const std::string& fileName, CameraBase* camera);

	CameraBase* getCamera(const std::string& devName) override;
	ImuSensorModel* getImu(const std::string& devName) override;

public:
	void skipNframes(int n) override;
	bool getSyncedMultiFrames(const std::unordered_map<std::string, const CameraBase*>& camNameMap, Common::MultiFrames* multiFrames, Common::ImageQualityType imgType = Common::IMAGE_SHARP) override;
	bool getSyncedImuMeasurements(ImuSensorModel* imuSensorModel, long long tillTimeStampInNs, std::queue<Common::ImuMeasurement>& imuMeasurements) override;
	bool getSyncedGroundTruthPoseVel(long long tillTimeStampInNs, Common::Transformation& pose, Eigen::Vector3d& velocity) override;
    void getMotionTrajectory(long long tillTimeStampIn_ns, long long exposureTimeIn_ns, std::vector<Common::Transformation, Eigen::aligned_allocator<Common::Transformation>>& trajectory) override;
	bool getSyncedDepthImages(const std::unordered_map<std::string, const CameraBase*>& camNameMap, long long timestamp_in_ns, Common::MultiFrames* multiFrames) override;

private:
	std::ifstream mCameraLogDataReader;
	FILE* mGroundTruthDataReader;
	FILE* mImuLogReader;
    FILE* mMotionTrajectoryDataReader;
	std::ifstream mDepthMapDataReader;
	std::queue<Common::ImuMeasurement> mPreviousImuMeasurements;
	std::unordered_map<std::string, Common::CameraBase*> mCameras;

	std::string mDir2dataset;
};

} /* namespace Utils */
} /* namespace VGUGV */

#endif /* SRC_UTILS_IO_IOUTILS_DATASETDSO_H_ */
