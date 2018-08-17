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

namespace ObjSLAM {
using namespace ITMLib;
class ObjSLAMTrackingEngine {
 private:
  ITMTrackingState *t_state;
  ITMRenderState *r_state;
  ITMSceneParams *params = new ITMSceneParams(0.5, 4, 0.01, 0.1, 4.0, false);
//  ITMVisualisationEngine<TVoxel, TIndex> *visualisationEngine;
  ITMTrackingController *t_controller;
  ITMTracker *tracker;
  ITMLowLevelEngine *lowEngine;

  Vector2i imgSize;
  const ITMLib::ITMLibSettings *settings;
  const ITMLib::ITMRGBDCalib *calib;

 public:
  ObjSLAMTrackingEngine(const ITMLib::ITMLibSettings *_settings,
                        const ITMLib::ITMRGBDCalib *_calib,
                        const Vector2i _imgSize);

  ITMLib::ITMTrackingState*  TrackFrame(ITMLib::ITMView * view);

};

}

#endif //MT_OBJSLAM_OBJSLAMTRACKINGENGINE_H
