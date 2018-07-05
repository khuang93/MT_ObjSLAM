//
// Created by khuang on 7/5/18.
//

#ifndef MT_OBJSLAM_OBJSLAMMAPPINGENGINE_H
#define MT_OBJSLAM_OBJSLAMMAPPINGENGINE_H
#include <External/InfiniTAM/InfiniTAM/ITMLib/Objects/Tracking/ITMTrackingState.h>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Objects/RenderStates/ITMRenderState.h>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Core/ITMBasicEngine.h>
#include "ObjectView_New.h"
#include "DatasetReader_LPD_Dataset.h"
namespace ObjSLAM {
template <typename TVoxel, typename TIndex>
class ObjSLAMMappingEngine {

 private:
  ObjectView_New* view;
  ITMLib::ITMTrackingState* t_state;
  ITMLib::ITMRenderState* r_state;
  ITMLib::ITMBasicEngine<TVoxel, TIndex>* itmBasicEngine;
  DatasetReader_LPD_Dataset reader;
  Vector2i imgSize;

 public:
  //Constructor with LPD Dataset
  ObjSLAMMappingEngine(string path, Vector2i _imgSize);


};
}
#endif //MT_OBJSLAM_OBJSLAMMAPPINGENGINE_H
