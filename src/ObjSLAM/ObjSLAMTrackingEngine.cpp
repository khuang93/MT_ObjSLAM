//
// Created by khuang on 8/16/18.
//

#include <External/InfiniTAM/InfiniTAM/ITMLib/Engines/LowLevel/ITMLowLevelEngineFactory.h>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Trackers/ITMTrackerFactory.h>
#include <memory>
#include <External/InfiniTAM/InfiniTAM/ORUtils/FileUtils.h>
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
                                                       this->params);

  t_controller = std::make_shared<ITMTrackingController>(tracker, settings);

  t_state = std::make_shared<ITMLib::ITMTrackingState>(imgSize, MEMORYDEVICE_CPU);
  t_state->Reset();

}

ITMLib::ITMTrackingState* ObjSLAMTrackingEngine::TrackFrame(ITMLib::ITMView *view) {


  std::cout<<"dbgTF\n";
  std::cout<<*t_state.get()->pose_d;
  this->t_controller.get()->Track(t_state.get(),view);
  std::cout<<t_state->pose_d->GetM();
  return t_state.get();
}

ITMLib::ITMTrackingState* ObjSLAMTrackingEngine::getTrackingState(){
  return t_state.get();
}

ITMLib::ITMTrackingController* ObjSLAMTrackingEngine::getTrackingController(){
  return t_controller.get();
}



}
