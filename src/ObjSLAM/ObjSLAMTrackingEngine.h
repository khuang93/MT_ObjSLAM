//
// Created by khuang on 8/16/18.
//

#ifndef MT_OBJSLAM_OBJSLAMTRACKINGENGINE_H
#define MT_OBJSLAM_OBJSLAMTRACKINGENGINE_H
#include <External/InfiniTAM/InfiniTAM/ITMLib/Objects/Tracking/ITMTrackingState.h>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Objects/RenderStates/ITMRenderState.h>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Utils/ITMSceneParams.h>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Core/ITMTrackingController.h>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Engines/LowLevel/Interface/ITMLowLevelEngine.h>
#include "ObjCameraPose.h"

namespace ObjSLAM {
using namespace ITMLib;
/** \brief
    * Tracking Interface to ITMTracker
	*/
class ObjSLAMTrackingEngine {
 protected:
  std::shared_ptr<ITMTrackingState> t_state;
  std::unique_ptr<ITMSceneParams> params = make_unique<ITMSceneParams>(0.5, 4, 0.1, 0.1, 4.0, false);
  std::shared_ptr<ITMTrackingController> t_controller;
  ITMTracker *tracker;
  ITMLowLevelEngine *lowEngine;
  ORUtils::SE3Pose pose_prev;
  Vector2i imgSize;
  const std::shared_ptr<ITMLib::ITMLibSettings> settings;
  const std::shared_ptr<ITMLib::ITMLibSettings> settings_obj;
  const std::shared_ptr<ITMLib::ITMRGBDCalib> calib;
  int imgNumber = 1;

 public:
  ObjSLAMTrackingEngine(const std::shared_ptr<ITMLib::ITMLibSettings> _settings,const std::shared_ptr<ITMLib::ITMLibSettings> _settings_obj,
                        const std::shared_ptr<ITMLib::ITMRGBDCalib> _calib,
                        const Vector2i _imgSize);
  ~ObjSLAMTrackingEngine();
  shared_ptr<ITMLib::ITMTrackingState>  TrackFrame(ITMLib::ITMView * view);

    template<class TVoxel, class TIndex>
  shared_ptr<ITMLib::ITMTrackingState>  TrackFrame(ObjectInstance_ptr<TVoxel, TIndex> obj_inst_ptr);

  shared_ptr<ITMLib::ITMTrackingState>  GetTrackingState();

  shared_ptr<ITMLib::ITMTrackingController> GetTrackingController();

  void OutputTrackingResults(std::string path);

  void OutputTrackingResults(std::ofstream &of);
};

}

#endif //MT_OBJSLAM_OBJSLAMTRACKINGENGINE_H
