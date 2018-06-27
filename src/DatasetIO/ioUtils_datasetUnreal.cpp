#include "ioUtils_datasetUnreal.h"
#include <core/common/enums.h>
#include <core/sensor/cameraDistortionFoV.h>
#include <fstream>
#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <boost/filesystem.hpp>

#include "core/measurement/frame.h"
#include "core/measurement/image.h"
#include "core/measurement/multiFrames.h"
#include "core/nonMemberFunctions/nonMemberFunctions.h"
#include "core/sensor/pinholeCameraModel.h"
#include "core/sensor/SensorManager.h"

namespace VGUGV
{
	namespace Utils
	{
		bool IOUtils_datasetUnreal::loadImuCalibrations(const std::string& imuCalibFile)
		{
			return true;
		}

		bool IOUtils_datasetUnreal::loadCameraCalibrations(const std::string& camIntrinsicCalibFileName, const std::string& camExtrinsicCalibFileName)
		{
			std::ifstream intrinsicFileStream;
			intrinsicFileStream.open(camIntrinsicCalibFileName, std::ifstream::in);
			if (intrinsicFileStream.is_open() == false)
			{
				std::cerr << " fails to load in intrinsic camera calibration file " << camIntrinsicCalibFileName << "\n";
				return false;
			}

			enum decodeState {ENUM_DEVNAME, ENUM_RESOLUTION, ENUM_K, ENUM_T, ENUM_SCALEFACTOR};
			decodeState state = ENUM_DEVNAME;

			std::string camDevName;
			int nRows, nCols;
			float fx, fy, cx, cy;

			while (!intrinsicFileStream.eof())
			{
				std::string line;
				getline(intrinsicFileStream, line);

				std::vector<std::string> tokens;
				Common::splitString(line, ' ', tokens);

				if (tokens.empty()) continue;

				if (tokens[0].compare("#") == 0) continue;

				if (state == ENUM_DEVNAME)
				{
					if (tokens[0].compare("devName") == 0 && tokens.size() > 1)
					{
						camDevName = tokens[1];
						state = ENUM_RESOLUTION;
					}
				}

				if (state == ENUM_RESOLUTION)
				{
					if (tokens[0].compare("resolution") == 0 && tokens.size() > 2)
					{
						nCols = std::atoi(tokens[1].c_str());
						nRows = std::atoi(tokens[2].c_str());
						state = ENUM_K;
					}
				}

				if (state == ENUM_K)
				{
					if (tokens[0].compare("K") == 0 && tokens.size() > 4)
					{
						fx = std::atof(tokens[1].c_str());
						fy = std::atof(tokens[2].c_str());
						cx = std::atof(tokens[3].c_str());
						cy = std::atof(tokens[4].c_str());

						state = ENUM_DEVNAME;

						// create camera model
						Eigen::Matrix3f K = Eigen::Matrix3f::Identity();
						K(0, 0) = fx; 
						K(1, 1) = fy;
						K(0, 2) = cx;
						K(1, 2) = cy;
						Common::CameraBase* camera = Common::SensorManager::getInstance().Create_camera(Common::CameraModelType::PINHOLE, nRows, nCols, K, std::vector<float>());
						mCameras.insert(std::make_pair(camDevName, camera));
					}
				}
			}

			// load in extrinsics
			std::ifstream extrinsicFileStream;
			extrinsicFileStream.open(camExtrinsicCalibFileName, std::ifstream::in);
			if (!extrinsicFileStream.is_open())
			{
				std::cerr << " fails to load in extrinsic camera calibration file " << camExtrinsicCalibFileName << "\n";
				return false;
			}

			Common::Transformation T_cam2body;
			float translation_scaleFactor = 1;
			while (!extrinsicFileStream.eof())
			{
				std::string line;
				getline(extrinsicFileStream, line);

				std::vector<std::string> tokens;
				Common::splitString(line, ' ', tokens);

				if (tokens.empty()) continue;

				if (tokens[0].compare("#") == 0) continue;

				if (state == ENUM_DEVNAME)
				{
					if (tokens[0].compare("devName") == 0 && tokens.size() > 1)
					{
						camDevName = tokens[1];
						state = ENUM_SCALEFACTOR;
					}
				}

				if (state == ENUM_SCALEFACTOR)
				{
					if (tokens[0].compare("scalingFactor") == 0 && tokens.size() > 1)
					{
						translation_scaleFactor = std::atof(tokens[1].c_str());
						state = ENUM_T;
					}
				}

				if (state == ENUM_T)
				{
					if (tokens[0].compare("T") == 0)
					{
						state = ENUM_DEVNAME;
						float qw, qx, qy, qz, x, y, z;
						qw = std::atof(tokens[1].c_str());
						qx = std::atof(tokens[2].c_str());
						qy = std::atof(tokens[3].c_str());
						qz = std::atof(tokens[4].c_str());
						x = std::atof(tokens[5].c_str()) * translation_scaleFactor;
						y = std::atof(tokens[6].c_str()) * translation_scaleFactor;
						z = std::atof(tokens[7].c_str()) * translation_scaleFactor;

						Eigen::Quaterniond R(qw, qx, qy, qz); R.normalize();
						Eigen::Vector3d t(x, y, z);
						T_cam2body = Common::Transformation(R, t);

						if (mCameras.count(camDevName) == 1)
						{
							Common::CameraBase* camera = mCameras.at(camDevName);
							camera->setT_Body2Sensor(T_cam2body.inverse());
						}
					}
				}
			}

			return true;
		}

		void IOUtils_datasetUnreal::loadVignettingImage(const std::string& fileName, CameraBase* camera)
		{
		}

	    CameraBase* IOUtils_datasetUnreal::getCamera(const std::string& devName)
		{
			if (mCameras.count(devName) == 0) return nullptr;
			return mCameras.at(devName);
		}

		ImuSensorModel* IOUtils_datasetUnreal::getImu(const std::string& devName)
		{
			ImuSensorModel* imu = Common::SensorManager::getInstance().Create_IMU();
			imu->setAccelerometerNoiseDensity(1e-8);
			imu->setAccelerometerDriftNoiseDensity(1e-8);
			imu->setGyroscopeNoiseDensity(1e-8);
			imu->setGyroscopeDriftNoiseDensity(1e-8);
			imu->setEarthAcceleration(0);
			return imu;
		}


		bool IOUtils_datasetUnreal::initDataReader4CameraLogReading(Common::SensorSystemConfig* systemSensorConfig, const std::vector<std::string>& camLogFileNames)
		{
			if (camLogFileNames.empty())
			{
				std::cerr << " fails to open empty camera log file " << "\n";
				return false;
			}

			mCameraLogDataReader.open(camLogFileNames[0], std::ifstream::in);

			if (mCameraLogDataReader.is_open() == false)
			{
				std::cerr << " fails to open camera log file " << camLogFileNames[0] << "\n";
				return false;
			}

			boost::filesystem::path fullPath(camLogFileNames[0]);
			mDir2dataset = fullPath.parent_path().string();

			return true;
		}

		bool IOUtils_datasetUnreal::initDataReader4GroundTruthLogReading(const std::string& groundTruthPoseFileName)
		{
			mGroundTruthDataReader = fopen(groundTruthPoseFileName.c_str(), "r");
			if (mGroundTruthDataReader == NULL)
			{
				std::cerr << " fails to open ground truth log file " << groundTruthPoseFileName << "\n";
				return false;
			}
			char line[1024];
			fgets(line, 1024, mGroundTruthDataReader);
			return true;
		}

		bool IOUtils_datasetUnreal::initDataReader4IMULogReading(const std::string& ImuLogFileName)
		{
			mImuLogReader = fopen(ImuLogFileName.c_str(), "r");
			if (mImuLogReader == NULL)
			{
				std::cerr << " fails to open imu log file " << ImuLogFileName << "\n";
				return false;
			}
			char line[1024];
			fgets(line, 1024, mImuLogReader);
			return true;
		}

        bool IOUtils_datasetUnreal::initDataReader4MotionTrajLogReading(const std::string &motionTrajFileName)
        {
            mMotionTrajectoryDataReader = fopen(motionTrajFileName.c_str(), "r");
            if (mMotionTrajectoryDataReader == NULL)
            {
                std::cerr << " fails to open ground truth log file " << motionTrajFileName << "\n";
                return false;
            }
            char line[1024];
            fgets(line, 1024, mMotionTrajectoryDataReader);
            return true;
        }

		bool IOUtils_datasetUnreal::initDataReader4DepthLogReading(const std::string& fileName)
		{
			mDepthMapDataReader.open(fileName.c_str(), std::ifstream::in);
			if(!mDepthMapDataReader.is_open())
			{
				std::cerr << " fails to open depth map log file " << fileName << "\n";
				return false;
			}
			// skip first line
			std::string line;
            std::getline(mDepthMapDataReader, line);
			return true;
		}

		void IOUtils_datasetUnreal::skipNframes(int n)
		{
			size_t nImage2Read = mCameras.size();
			Common::MultiFrames* mf = new Common::MultiFrames(nImage2Read);
			for(int i = 0; i < n; ++i)
			{
				this->getSyncedMultiFrames(std::unordered_map<std::string, const CameraBase*>(), mf, Common::IMAGE_SHARP);
			}
			delete mf;
		}

		bool IOUtils_datasetUnreal::getSyncedMultiFrames(const std::unordered_map<std::string, const CameraBase*>& camNameMap,
														 Common::MultiFrames* multiFrames,
														 Common::ImageQualityType imgType)
		{
			size_t nImage2Read = mCameras.size();
			bool bSeqNumInitialized = false;
			int  seqNum = 0;

            std::string imageType;
            if(imgType == Common::IMAGE_SHARP)
            {
                imageType = "rgb";
            }
            else
            {
                imageType = "blur";
            }

			while (nImage2Read > 0 && !mCameraLogDataReader.eof())
			{
				std::string line;
				std::getline(mCameraLogDataReader, line);
				std::vector<std::string> tokens;
				Common::splitString(line, ' ', tokens);
				if (tokens[0].compare("#") == 0) continue;

				if (tokens[2].compare(imageType) == 0)
				{
					nImage2Read--;
					if (!bSeqNumInitialized)
					{
						bSeqNumInitialized = true;
						seqNum = std::atoi(tokens[1].c_str());
					}

					long long timestamp = std::atof(tokens[0].c_str()) * 1e9;
					std::string devName = tokens[3];
					unsigned int exposureTime_us = std::atof(tokens[4].c_str()) * 1e6; // in us
					std::string fileName = tokens[5];

					if (camNameMap.count(devName) == 0) continue;

					// load in image
					std::string path2file = mDir2dataset + "/" + fileName;
					cv::Mat cv_image = cv::imread(path2file, 0);

					const Common::CameraBase* camera = camNameMap.at(devName);
					float imageScalingFactor = camera->getImageScalingFactor();

					if (fabs(imageScalingFactor - 1) > 1e-3)
					{
						cv::resize(cv_image, cv_image, cv::Size(), imageScalingFactor, imageScalingFactor);
					}

					// convert to internal representation
					Common::ImageType internal_image(cv_image.rows, cv_image.cols, true, false);
					memcpy(internal_image.GetData(Common::MEMORYDEVICE_CPU), cv_image.data, cv_image.rows * cv_image.cols * sizeof(Common::ImageElementType));

					// create frame
					Common::Frame* frame = multiFrames->getFrame(camera->getDeviceID());
					frame->setCamera(camera);
					frame->setImage(internal_image);
					frame->setExposure(exposureTime_us);
					frame->setGain(1);
					frame->setSeq(seqNum);
					frame->setTimeStamp(timestamp);
					multiFrames->setTimeStamp(timestamp);
					multiFrames->setSeqId(seqNum);
				}
			}
			return true;
		}

		bool IOUtils_datasetUnreal::getSyncedImuMeasurements(ImuSensorModel* imuSensorModel, long long tillTimeStampInNs, std::queue<Common::ImuMeasurement>& imuMeasurements)
		{
			while (!mPreviousImuMeasurements.empty())
			{
				imuMeasurements.push(mPreviousImuMeasurements.front());
				mPreviousImuMeasurements.pop();
			}

			while (true && !feof(mImuLogReader))
			{
				float t, qw, qx, qy, qz, x, y, z, vx, vy, vz, p, q, r, ax, ay, az;
				if (fscanf(mImuLogReader, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f", &t, &qw, &qx, &qy, &qz, &x, &y, &z, &vx, &vy, &vz, &p, &q, &r, &ax, &ay, &az) != 17)
				{
					std::cerr << " fails to read new imu log data\n";
					return false;
				}

				long long timestamp = t * 1e9;
				
				Common::ImuMeasurement newData;
				newData.timeStamp = timestamp;
				newData.acc << ax, ay, az;
				newData.gyro << p, q, r;

				if (newData.timeStamp < tillTimeStampInNs)
				{
					if (imuMeasurements.size() > 0)
					{
						imuMeasurements.back().dt = newData.timeStamp - imuMeasurements.back().timeStamp;
					}
					imuMeasurements.push(newData);
				}
				else
				{
					imuMeasurements.back().dt = tillTimeStampInNs - imuMeasurements.back().timeStamp;

					Common::ImuMeasurement interpolatedImuMeasurement = imuMeasurements.back();
					interpolatedImuMeasurement.timeStamp = tillTimeStampInNs;
					interpolatedImuMeasurement.dt = newData.timeStamp - tillTimeStampInNs;
					mPreviousImuMeasurements.push(interpolatedImuMeasurement);
					mPreviousImuMeasurements.push(newData);

					break;
				}
			}

			return true;
		}

		bool IOUtils_datasetUnreal::getSyncedGroundTruthPoseVel(long long tillTimeStampInNs, Common::Transformation& pose, Eigen::Vector3d& velocity)
		{
			long long timestamp = 0;
			float t, qw, qx, qy, qz, x, y, z, vx, vy, vz, p, q, r, ax, ay, az;
			while (timestamp < tillTimeStampInNs && !feof(mGroundTruthDataReader))
			{
				if (fscanf(mGroundTruthDataReader, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f", &t, &qw, &qx, &qy, &qz, &x, &y, &z, &vx, &vy, &vz, &p, &q, &r, &ax, &ay, &az) != 17)
				{
					std::cerr << " fails to read new ground truth log data\n";
					return false;
				}

				timestamp = t * 1e9;
			}

			if (timestamp < tillTimeStampInNs || (timestamp - tillTimeStampInNs > 10000000))
			{
				std::cerr << "Could not match ground truth timestamp (" << timestamp << " vs. " << tillTimeStampInNs << ")" << std::endl;
				return false;
			}

			Eigen::Quaterniond R(qw, qx, qy, qz);
			R.normalize();
			pose.setQuaternion(R);
			pose.translation() = Eigen::Vector3d(x, y, z);
			velocity << vx, vy, vz;

			return true;
		}

        void IOUtils_datasetUnreal::getMotionTrajectory(long long tillTimeStampIn_ns, long long exposureTimeIn_ns, std::vector<Common::Transformation, Eigen::aligned_allocator<Common::Transformation>>& trajectory)
        {
            unsigned long long startTimeIn_ns = tillTimeStampIn_ns - exposureTimeIn_ns;
            unsigned long long timestamp = 0;

            float t, qw, qx, qy, qz, x, y, z, vx, vy, vz, p, q, r, ax, ay, az;
            while (timestamp < startTimeIn_ns && !feof(mMotionTrajectoryDataReader))
            {
                if (fscanf(mMotionTrajectoryDataReader, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f", &t, &qw, &qx, &qy, &qz, &x, &y, &z, &vx, &vy, &vz, &p, &q, &r, &ax, &ay, &az) != 17)
                {
                    std::cerr << " fails to read new ground truth log data\n";
                    return;
                }

                timestamp = t * 1e9;
            }

            if (timestamp < startTimeIn_ns || (timestamp - startTimeIn_ns > 10000000))
            {
                std::cerr << "Could not match ground truth timestamp (" << timestamp << " vs. " << startTimeIn_ns << ")" << std::endl;
                return;
            }

			trajectory.clear();

            while (timestamp < tillTimeStampIn_ns && !feof(mMotionTrajectoryDataReader))
            {
                if (fscanf(mMotionTrajectoryDataReader, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f", &t, &qw, &qx, &qy, &qz, &x, &y, &z, &vx, &vy, &vz, &p, &q, &r, &ax, &ay, &az) != 17)
                {
                    std::cerr << " fails to read new ground truth log data\n";
                    return;
                }
                timestamp = t * 1e9;

                Eigen::Quaterniond R(qw, qx, qy, qz); R.normalize();
                Eigen::Vector3d t(x, y, z);
                Common::Transformation T(R, t);
                trajectory.push_back(T);
            }
            return;
        }

		bool IOUtils_datasetUnreal::getSyncedDepthImages(const std::unordered_map<std::string, const CameraBase*>& camNameMap, long long timestamp_in_ns, Common::MultiFrames* multiFrames)
		{
			size_t nImage2Read = camNameMap.size();

			while (nImage2Read > 0 && !mDepthMapDataReader.eof())
			{
				std::string line;
				std::getline(mDepthMapDataReader, line);

				// decode line
				std::istringstream line_reader(line);
				double timestamp;
				int seqNum;
				std::string imageType, camDevName, path2image;
				double exposure_time;

				line_reader >> timestamp; // in seconds
				line_reader >> seqNum;
				line_reader >> imageType;
				line_reader >> camDevName;
				line_reader >> exposure_time;
				line_reader >> path2image;

                if(imageType.compare("depth") != 0) continue;

				path2image = mDir2dataset + path2image;

				if(std::abs(timestamp * 1e9 - timestamp_in_ns) < 100000)
				{
					if(camNameMap.count(camDevName) == 0) continue;

					// load in depth image
					const Common::CameraBase* camera = camNameMap.at(camDevName);
					size_t nRows = camera->getCameraHeight();
					size_t nCols = camera->getCameraWidth();
					Common::Image<float> depthMap (nRows, nCols, true, false);

					std::ifstream depth_file(path2image);
					if (depth_file.fail()) {
						throw std::runtime_error("can't read depth image: " + path2image);
					}

					std::string line_depth;
					float* depthMapPtr = depthMap.GetData(Common::MEMORYDEVICE_CPU);
					for(size_t i = 0; i < nRows; ++i)
					{
						getline(depth_file, line_depth);
						std::istringstream line_depth_reader(line_depth);
						for(size_t j = 0; j < nCols; ++j)
						{
							float depth;
							line_depth_reader >> depth;

							depthMapPtr[0] = 0;
							if(depth > 0 && depth < 1000)
							{
								depthMapPtr[0] = depth;
							}

							depthMapPtr++;
						}
					}

					Common::Frame* frame = multiFrames->getFrame(camera->getDeviceID());
					frame->setDepthImage(depthMap);
					nImage2Read--;
				}
			}
            return true;
		}

    } // namespace Utils
} // namespace VGUGV


