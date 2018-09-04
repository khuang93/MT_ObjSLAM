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

#include "LPD_Dataset_Reader.h"
#include "ObjectInstanceScene.h"
#include "ObjectView_New.h"

namespace ObjSLAM {
//using ObjectInstanceSceneVector = std::vector<ObjectInstanceScene*>;
//using ObjectInstancePair = std::pair<ObjectInstance, ObjectInstanceScene_old>;
//using ObjectInstanceVector = vector<ObjectInstancePair>;
template<typename TVoxel, typename TIndex>
using obj_inst_ptr = std::shared_ptr<ObjectInstance_New<TVoxel, TIndex>>;

template<typename TVoxel, typename TIndex>
class ObjSLAMMappingEngine {

 private:
  int imgNumber;

  shared_ptr<ObjectView_New<TVoxel, TIndex>> view_new;
  vector<shared_ptr<ObjectView_New<TVoxel, TIndex>>> view_new_vec;
  shared_ptr<ITMLib::ITMTrackingState> t_state = NULL;
  ITMLib::ITMTrackingState *t_state_orig = NULL;
  ITMLib::ITMRenderState *r_state;
  ITMLib::ITMRenderState *r_state_BG;

  ITMLib::ITMVisualisationEngine<TVoxel, TIndex> *visualisationEngine;
  shared_ptr<ITMLib::ITMTrackingController> t_controller;
  ITMLib::ITMTracker *tracker;
  ITMLib::ITMLowLevelEngine *lowEngine;
  Vector2i imgSize;
  std::vector<ObjectInstanceScene<TVoxel, TIndex> *> object_instance_scene_vector;
  std::vector<obj_inst_ptr<TVoxel, TIndex>> obj_inst_ptr_vector;
  const ITMLib::ITMLibSettings *settings;
  const ITMLib::ITMRGBDCalib *calib;
//  std::vector<ObjectInstanceScene_old> listOfObjectScenes;

  ITMLib::ITMDenseMapper<TVoxel, TIndex> *denseMapper;
  std::vector<std::shared_ptr<ObjectClassLabel_Group<TVoxel, TIndex>>> label_ptr_vector;

  std::vector<int> rgb_d_pixel_idx_vec; //index is index in rgb img and value is index in depth img

 public:
  //Constructor with LPD Dataset
  ObjSLAMMappingEngine(ITMLib::ITMLibSettings *_settings, string path, Vector2i _imgSize);

  ObjSLAMMappingEngine(const ITMLib::ITMLibSettings *_settings,
                       const ITMLib::ITMRGBDCalib *_calib,
                       const Vector2i _imgSize);

  shared_ptr<ObjectView_New<TVoxel, TIndex>> CreateView(ObjFloatImage *_depth,
                                                        ObjUChar4Image *_rgb,
                                                        LabelImgVector _label_img_vector);

  void ProcessFrame();

  void init_rgb_d_pair_idx();

  std::vector<int> get_rgb_d_pair_idx();

  void ProcessOneObject(std::shared_ptr<ITMLib::ITMView> &itmview, ObjectInstanceScene<TVoxel, TIndex> *scene, std::shared_ptr<ObjectInstance_New<TVoxel,TIndex>> obj_inst_ptr);

  bool checkIsNewObject(obj_inst_ptr<TVoxel, TIndex> obj_ptr);

  bool checkIsSameObject(obj_inst_ptr<TVoxel, TIndex> obj_ptr_1, obj_inst_ptr<TVoxel, TIndex> obj_ptr_2);

  bool checkIsSameObject2D(obj_inst_ptr<TVoxel, TIndex> obj_ptr_1, obj_inst_ptr<TVoxel, TIndex> obj_ptr_2);

  bool checkBoundingCubeOverlap(ORUtils::Vector6<float> first, ORUtils::Vector6<float> second);

  double calculateCubeVolume(ORUtils::Vector6<float> corners);

  bool checkImageOverlap(ObjSLAM::ObjFloatImage *first, ObjSLAM::ObjFloatImage *second);

  ORUtils::Vector4<int> getBoundingBox(ObjFloatImage *input);

  void UpdateTrackingState(const ORUtils::SE3Pose *_pose);

  void UpdateTrackingState(shared_ptr<ITMLib::ITMTrackingState> _t_state);

  void UpdateTrackingState_Orig(const ORUtils::SE3Pose *_pose);

  void UpdateViewPose();

  void SetTrackingController(shared_ptr<ITMLib::ITMTrackingController> _t_controller);

  void UpdateImgNumber(int _imgNum) { imgNumber = _imgNum; };

  void outputAllLabelStats();

  void outputAllObjImages();

  void deleteAll();

  std::vector<obj_inst_ptr<TVoxel, TIndex>> getObjInstPtrVec();
  std::vector<std::shared_ptr<ObjectClassLabel_Group<TVoxel, TIndex>>> getLabelPtrVec();

  void SaveSceneToMesh(const char *objFileName, std::shared_ptr<ObjectInstanceScene<TVoxel, TIndex>> scene_ptr);

  ~ObjSLAMMappingEngine() {
    deleteAll();
  }
//  void GetNextFrame();
//
//  void CreateObjectScenes();
//

};
}
#include "ObjSLAMMappingEngine.tpp"
#endif //MT_OBJSLAM_OBJSLAMMAPPINGENGINE_H
