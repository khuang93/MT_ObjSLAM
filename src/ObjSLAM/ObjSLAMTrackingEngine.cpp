//
// Created by khuang on 8/16/18.
//

#include <External/InfiniTAM/InfiniTAM/ITMLib/Engines/LowLevel/ITMLowLevelEngineFactory.h>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Trackers/ITMTrackerFactory.h>
#include <memory>
#include "ObjSLAMTrackingEngine.h"

namespace ObjSLAM {
ObjSLAMTrackingEngine::ObjSLAMTrackingEngine(const ITMLib::ITMLibSettings *_settings,
                                             const ITMLib::ITMRGBDCalib *_calib,
                                             const Vector2i _imgSize)
    : settings(_settings), calib(_calib), imgSize(_imgSize) {
  lowEngine = ITMLib::ITMLowLevelEngineFactory::MakeLowLevelEngine(settings->deviceType);

  tracker = ITMLib::ITMTrackerFactory::Instance().Make(imgSize,
                                                       imgSize,
                                                       settings,
                                                       lowEngine,
                                                       new ITMLib::ITMIMUCalibrator_iPad(),
                                                       this->params.get());

  t_controller = std::make_shared<ITMTrackingController>(tracker, settings);

  t_state = std::make_shared<ITMLib::ITMTrackingState>(imgSize, MEMORYDEVICE_CPU);
  t_state->Reset();

}

ObjSLAMTrackingEngine::~ObjSLAMTrackingEngine(){
  delete this->calib;
  delete this->settings;
  delete this->lowEngine;
  delete tracker;
}

shared_ptr<ITMLib::ITMTrackingState> ObjSLAMTrackingEngine::TrackFrame(ITMLib::ITMView *view) {

  this->t_controller.get()->Track(t_state.get(), view);
  std::cout << t_state->pose_d->GetM();
  outputTrackingResults("trackingResults.txt");
  imgNumber++;
  return t_state;
}

shared_ptr<ITMLib::ITMTrackingState>  ObjSLAMTrackingEngine::getTrackingState() {
  return t_state;
}

shared_ptr<ITMLib::ITMTrackingController> ObjSLAMTrackingEngine::getTrackingController() {
  return t_controller;
}

void ObjSLAMTrackingEngine::outputTrackingResults(std::string path) {

  std::ofstream of;
  if (imgNumber == 1) {
    of.open(path);
  } else {
    of.open(path, ios::app);
  }
  outputTrackingResults(of);
}

void ObjSLAMTrackingEngine::outputTrackingResults(std::ofstream &of) {
  ObjCameraPose obj_cam_pose(*(t_state.get()->pose_d));
  ORUtils::Vector3<float> pos = t_state.get()->pose_d->GetT();
  Eigen::Quaterniond eigen_quat = obj_cam_pose.getQuaternion();
  double time = imgNumber * 0.1;
  of << time << ", " << 1 << ", " << eigen_quat.w() << ", " << eigen_quat.x() << ", " << eigen_quat.y() << ", "
     << eigen_quat.z() << ", " << pos.x << ", " << pos.y << ", " << pos.z << endl;
}

}
