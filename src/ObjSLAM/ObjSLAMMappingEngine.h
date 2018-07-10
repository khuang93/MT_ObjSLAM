//
// Created by khuang on 7/5/18.
//


#ifndef MT_OBJSLAM_OBJSLAMMAPPINGENGINE_H
#define MT_OBJSLAM_OBJSLAMMAPPINGENGINE_H

#include <vector>

#include <External/InfiniTAM/InfiniTAM/ITMLib/Objects/Tracking/ITMTrackingState.h>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Objects/RenderStates/ITMRenderState.h>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Core/ITMBasicEngine.h>
#include <External/InfiniTAM/InfiniTAM/ITMLib/ITMLibDefines.h>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Engines/Reconstruction/CPU/ITMSceneReconstructionEngine_CPU.h>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Utils/ITMMath.h>

#include "ObjectView_New.h"
#include "DatasetReader_LPD_Dataset.h"
#include "ObjectInstanceScene_old.h"
#include "ObjectInstanceScene.h"

namespace ObjSLAM {
//using ObjectInstanceSceneVector = std::vector<ObjectInstanceScene*>;
//using ObjectInstancePair = std::pair<ObjectInstance, ObjectInstanceScene_old>;
//using ObjectInstanceVector = vector<ObjectInstancePair>;

template<typename TVoxel, typename TIndex>
class ObjSLAMMappingEngine {

 public:
  ObjectView_New *view;
  ITMLib::ITMTrackingState *t_state;
  ITMLib::ITMRenderState *r_state;
  ITMLib::ITMBasicEngine<TVoxel, TIndex> *itmBasicEngine;
  ITMLib::ITMSceneParams* params = new ITMLib::ITMSceneParams(0.5, 4, 0.01, 0.1, 4.0, false);
  DatasetReader_LPD_Dataset reader;
  Vector2i imgSize;
  std::vector<ObjectInstanceScene<TVoxel,TIndex>*> object_instance_scene_vector;
//  std::vector<ObjectInstanceScene_old> listOfObjectScenes;

 public:
  //Constructor with LPD Dataset
  ObjSLAMMappingEngine(string path, Vector2i _imgSize);
  void bla();

//  void GetNextFrame();
//
//  void CreateObjectScenes();
//
//  void ProcessFrame();



//  void



};
}
#include "ObjSLAMMappingEngine.tpp"
#endif //MT_OBJSLAM_OBJSLAMMAPPINGENGINE_H