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

#include "ObjectView.h"
#include "DatasetReader_LPD_Dataset.h"
#include "ObjectInstanceScene_old.h"
#include "ObjectInstanceScene.h"
#include "ObjectView_New.h"

namespace ObjSLAM {
//using ObjectInstanceSceneVector = std::vector<ObjectInstanceScene*>;
//using ObjectInstancePair = std::pair<ObjectInstance, ObjectInstanceScene_old>;
//using ObjectInstanceVector = vector<ObjectInstancePair>;

template<typename TVoxel, typename TIndex>
class ObjSLAMMappingEngine {

 private:
  ObjectView *view;
  ObjectView_New<TVoxel, TIndex> * view_new;
  ITMLib::ITMTrackingState *t_state;
  ITMLib::ITMTrackingState *t_state_orig;
  ITMLib::ITMRenderState *r_state;
  ITMLib::ITMBasicEngine<TVoxel, TIndex> *itmBasicEngine;
  ITMLib::ITMSceneParams* params = new ITMLib::ITMSceneParams(0.5, 4, 0.01, 0.1, 4.0, false);
  ITMLib::ITMVisualisationEngine<TVoxel, TIndex>  * visualisationEngine;
  ITMLib::ITMTrackingController * t_controller;
  ITMLib::ITMTracker * tracker;
  ITMLib::ITMLowLevelEngine * lowEngine;
  DatasetReader_LPD_Dataset reader;
  Vector2i imgSize;
  std::vector<ObjectInstanceScene<TVoxel,TIndex>*> object_instance_scene_vector;
  const ITMLib::ITMLibSettings * settings;
  const ITMLib::ITMRGBDCalib* calib;
//  std::vector<ObjectInstanceScene_old> listOfObjectScenes;

  ITMLib::ITMDenseMapper<TVoxel, TIndex> *denseMapper;


 public:
  //Constructor with LPD Dataset
  ObjSLAMMappingEngine(ITMLib::ITMLibSettings* _settings,string path, Vector2i _imgSize);

  ObjSLAMMappingEngine(const ITMLib::ITMLibSettings* _settings, const ITMLib::ITMRGBDCalib* _calib, const Vector2i _imgSize);

  void CreateView(ObjCameraPose pose, ObjFloatImage* _depth, ObjUChar4Image* _rgb, LabelImgVector _label_img_vector);

  void ProcessFrame();

  void ProcessOneObject(Object_View_Tuple& view_tuple, ObjectInstanceScene<TVoxel, TIndex>* scene, int obj_idx);

  void UpdateTrackingState(const ORUtils::SE3Pose* _pose);

  void UpdateTrackingState_Orig(const ORUtils::SE3Pose* _pose);

  void deleteAll();

  ~ObjSLAMMappingEngine(){
    deleteAll();
  }
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
