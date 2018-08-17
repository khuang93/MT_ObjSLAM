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

  t_controller = std::make_shared<ITMTrackingController>(tracker, settings).get();

  t_state = std::make_shared<ITMLib::ITMTrackingState>(imgSize, MEMORYDEVICE_CPU).get();
  t_state->Reset();

}

ITMLib::ITMTrackingState* ObjSLAMTrackingEngine::TrackFrame(ITMLib::ITMView *view) {

  SaveImageToFile(view->depth, "testD");
  std::cout<<"dbg\n";
  std::cout<<t_state->pose_d->GetM();
  this->t_controller->Track(t_state,view);
  std::cout<<t_state->pose_d->GetM();
  return t_state;
}


}
