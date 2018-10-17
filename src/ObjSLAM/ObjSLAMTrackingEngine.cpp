//
// Created by khuang on 8/16/18.
//

#include <External/InfiniTAM/InfiniTAM/ITMLib/Engines/LowLevel/ITMLowLevelEngineFactory.h>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Trackers/ITMTrackerFactory.h>
#include <memory>
#include "ObjSLAMTrackingEngine.h"

namespace ObjSLAM {
ObjSLAMTrackingEngine::ObjSLAMTrackingEngine(const std::shared_ptr<ITMLib::ITMLibSettings> _settings,
                                             const std::shared_ptr<ITMLib::ITMLibSettings> _settings_obj,
                                             const std::shared_ptr<ITMLib::ITMRGBDCalib> _calib,
                                             const Vector2i _imgSize)
    : settings(_settings), settings_obj(_settings_obj), calib(_calib), imgSize(_imgSize) {
  lowEngine = ITMLib::ITMLowLevelEngineFactory::MakeLowLevelEngine(settings->deviceType);

  tracker = ITMLib::ITMTrackerFactory::Instance().Make(imgSize,
                                                       imgSize,
                                                       settings.get(),
                                                       lowEngine,
                                                       new ITMLib::ITMIMUCalibrator_iPad(),
                                                       this->params.get());

  t_controller = std::make_shared<ITMTrackingController>(tracker, settings.get());

  t_state = std::make_shared<ITMLib::ITMTrackingState>(imgSize, MEMORYDEVICE_CPU);
  t_state->Reset();
}

ObjSLAMTrackingEngine::~ObjSLAMTrackingEngine(){
//  delete this->calib;
//  delete this->settings;
  delete this->lowEngine;
  delete tracker;
}

shared_ptr<ITMLib::ITMTrackingState> ObjSLAMTrackingEngine::TrackFrame(ITMLib::ITMView *view) {
  this->t_controller.get()->Track(t_state.get(), view);
  if(t_state->trackerResult==ITMTrackingState::TRACKING_FAILED){
    t_state->pose_d->SetFrom(&pose_prev);
  }else{
    pose_prev.SetFrom(t_state->pose_d);
  }
  std::cout << t_state->pose_d->GetM();
  OutputTrackingResults("trackingResults.txt");
  imgNumber++;
  return t_state;
}



shared_ptr<ITMLib::ITMTrackingState>  ObjSLAMTrackingEngine::GetTrackingState() {
  return t_state;
}

shared_ptr<ITMLib::ITMTrackingController> ObjSLAMTrackingEngine::GetTrackingController() {
  return t_controller;
}

void ObjSLAMTrackingEngine::OutputTrackingResults(std::string path) {

  std::ofstream of;
  if (imgNumber == 1) {
    of.open(path);
  } else {
    of.open(path, ios::app);
  }
  OutputTrackingResults(of);
}

void ObjSLAMTrackingEngine::OutputTrackingResults(std::ofstream &of) {
  ObjCameraPose obj_cam_pose(*(t_state.get()->pose_d));
  ORUtils::SE3Pose pose_inv(t_state.get()->pose_d->GetInvM());
  ORUtils::Vector3<float> pos = pose_inv.GetT();
  ObjCameraPose obj_cam_pose_inv(pose_inv);
  Eigen::Quaterniond eigen_quat = obj_cam_pose_inv.GetQuaternion(); //try the inv of the pose
  double time = imgNumber+reader_SkipFrames*(imgNumber-1);

  of << time << ", "<< pos.x << ", " << pos.y << ", " << pos.z-2.25  << ", " << eigen_quat.x() << ", " << eigen_quat.y() << ", "
     << eigen_quat.z() << ", " << eigen_quat.w()<< endl;

  //  of << time << ", " << 1 << ", " << eigen_quat.w() << ", " << eigen_quat.x() << ", " << eigen_quat.y() << ", "
//     << eigen_quat.z() << ", " << pos.x << ", " << pos.y << ", " << pos.z << endl;
}

}
