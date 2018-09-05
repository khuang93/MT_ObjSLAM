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
#include <External/InfiniTAM/InfiniTAM/ORUtils/FileUtils.h>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Engines/Visualisation/ITMVisualisationEngineFactory.h>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Engines/LowLevel/ITMLowLevelEngineFactory.h>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Trackers/ITMTrackerFactory.h>
#include "External/InfiniTAM/InfiniTAM/ITMLib/Objects/RenderStates/ITMRenderStateFactory.h"
#include <External/InfiniTAM/InfiniTAM/ITMLib/Utils/ITMLibSettings.h>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Trackers/ITMTrackerFactory.h>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Core/ITMTrackingController.h>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Objects/Meshing/ITMMesh.h>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Engines/Meshing/ITMMeshingEngineFactory.h>


#include "DatasetReader.h"
#include "ObjectInstanceScene.h"
#include "ObjectView_New.h"
#include "ObjSLAMCamera.h"
#include "ObjSLAMDataTypes.h"




namespace ObjSLAM {


template<class TVoxel, class TIndex>
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
  ITMLib::ITMVisualisationEngine<TVoxel, TIndex> *visualisationEngine_BG;
  shared_ptr<ITMLib::ITMTrackingController> t_controller;
  ITMLib::ITMTracker *tracker;
//  ITMLib::ITMLowLevelEngine *lowEngine;
  Vector2i imgSize;
  std::vector<ObjectInstanceScene<TVoxel, TIndex> *> object_instance_scene_vector;
  std::vector<ObjectInstance_New_ptr<TVoxel, TIndex>> obj_inst_ptr_vector;
  const ITMLib::ITMLibSettings *settings;
  const ITMLib::ITMRGBDCalib *calib;


  ITMLib::ITMDenseMapper<TVoxel, TIndex> *denseMapper;
  std::vector<std::shared_ptr<ObjectClassLabel_Group<TVoxel, TIndex>>> label_ptr_vector;

  void deleteAll();

 public:
  //Constructor with LPD Dataset
  ObjSLAMMappingEngine(const ITMLib::ITMLibSettings *_settings,
                       const ITMLib::ITMRGBDCalib *_calib,
                       const Vector2i _imgSize) : settings(_settings), calib(_calib), imgSize(_imgSize) {

    denseMapper = new ITMLib::ITMDenseMapper<TVoxel, TIndex>(settings);

    sceneIsBackground = false;
    r_state =
        ITMLib::ITMRenderStateFactory<TIndex>::CreateRenderState(imgSize, &(settings->sceneParams),
                                                                 MEMORYDEVICE_CPU);

    visualisationEngine =
        ITMLib::ITMVisualisationEngineFactory::MakeVisualisationEngine<TVoxel, TIndex>(settings->deviceType);

    sceneIsBackground = true;
    r_state_BG =
        ITMLib::ITMRenderStateFactory<TIndex>::CreateRenderState(imgSize, &(settings->sceneParams),
                                                                 MEMORYDEVICE_CPU);

    visualisationEngine_BG =
        ITMLib::ITMVisualisationEngineFactory::MakeVisualisationEngine<TVoxel, TIndex>(settings->deviceType);
    //TODO Temp
//    lowEngine = ITMLib::ITMLowLevelEngineFactory::MakeLowLevelEngine(settings->deviceType);
  }

  void CreateView(ObjFloatImage *_depth,
                                                        ObjUChar4Image *_rgb,
                                                        LabelImgVector _label_img_vector);

  void ProcessFrame();

  void ProcessOneObject(std::shared_ptr<ITMLib::ITMView> &itmview,
                        ObjectInstanceScene<TVoxel, TIndex> *scene,
                        std::shared_ptr<ObjectInstance_New<TVoxel, TIndex>> obj_inst_ptr);

  bool checkIsNewObject(ObjectInstance_New_ptr<TVoxel, TIndex> obj_ptr);

  bool checkIsSameObject(ObjectInstance_New_ptr<TVoxel, TIndex> obj_ptr_1,
                         ObjectInstance_New_ptr<TVoxel, TIndex> obj_ptr_2);

  bool checkIsSameObject2D(ObjectInstance_New_ptr<TVoxel, TIndex> obj_ptr_1,
                           ObjectInstance_New_ptr<TVoxel, TIndex> obj_ptr_2);

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



  std::vector<ObjectInstance_New_ptr<TVoxel, TIndex>> getObjInstPtrVec(){return this->obj_inst_ptr_vector;}
  std::vector<std::shared_ptr<ObjectClassLabel_Group<TVoxel, TIndex>>> getLabelPtrVec(){   return this->label_ptr_vector;}

  void SaveSceneToMesh(const char *objFileName, std::shared_ptr<ITMLib::ITMScene<TVoxel, TIndex>> scene_ptr);

  ~ObjSLAMMappingEngine() {    deleteAll();  }


};
}
#include "ObjSLAMMappingEngine.tpp"
#endif //MT_OBJSLAM_OBJSLAMMAPPINGENGINE_H
