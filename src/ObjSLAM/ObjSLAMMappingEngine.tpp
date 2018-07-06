//
// Created by khuang on 7/5/18.
//

#pragma once
#include "ObjSLAMMappingEngine.h"
#include <External/InfiniTAM/InfiniTAM/ORUtils/FileUtils.h>


namespace ObjSLAM{

//Constructor with LPD Dataset
template <typename TVoxel, typename TIndex>
ObjSLAMMappingEngine<TVoxel, TIndex>::ObjSLAMMappingEngine(string path, Vector2i _imgSize):imgSize(_imgSize){
  reader = DatasetReader_LPD_Dataset(path, imgSize);

  //process one frame
  reader.readNext();

  //float mu, int maxW, float voxelSize, float viewFrustum_min, float viewFrustum_max, bool stopIntegratingAtMaxW
  auto *params = new ITMLib::ITMSceneParams(0.5, 4, 0.01, 0.1, 4.0, false);

  auto* calib = reader.getCalib();

  auto* view0 = new ObjSLAM::ObjectView_New(*calib, imgSize, imgSize, false, *reader.getPose(), reader.depth_img, reader.rgb_img, reader.label_img);

  vector<ObjSLAM::ObjectView_New *> ListofAllViews = {view0};

  //const ITMLib::ITMSceneParams *_sceneParams, bool _useSwapping, MemoryDeviceType _memoryType, ViewVector* _viewVector
  auto *object = new ObjSLAM::ObjectInstanceScene<TVoxel, TIndex>(
      params, true, MEMORYDEVICE_CPU, ListofAllViews);

  //Tracking State
  t_state = new ITMLib::ITMTrackingState(imgSize, MEMORYDEVICE_CPU);

  ORUtils::SE3Pose pose_tstate = reader.getPose()->getSE3Pose();
  t_state->pose_d = &(pose_tstate);

  //RenderState
  r_state = new ITMLib::ITMRenderState_VH(1,
                                                    imgSize,
                                                    object->sceneParams->viewFrustum_min,
                                                    object->sceneParams->viewFrustum_min,
                                                    MEMORYDEVICE_CPU);

  auto *engine_cpu = new ITMLib::ITMSceneReconstructionEngine_CPU<ITMVoxel, ITMVoxelIndex>;

  engine_cpu->ResetScene(object);

  ObjSLAM::Object_View_Tuple view_tuple = view0->getObjMap().find(58)->second;
  ObjSLAM::ObjUChar4Image *img = new ObjSLAM::ObjUChar4Image(imgSize,MEMORYDEVICE_CPU);

  //basic engine
  ITMLib::ITMLibSettings *internalSettings = new ITMLib::ITMLibSettings();
  internalSettings->deviceType=internalSettings->DEVICE_CPU;
  int obj_class_num =58;

  itmBasicEngine = new ITMLib::ITMBasicEngine<ITMVoxel,ITMVoxelIndex>(internalSettings,*calib,imgSize);

  itmBasicEngine->ProcessFrame(std::get<1>(view0->getObjMap().find(obj_class_num)->second)->rgb,std::get<1>(view0->getObjMap().find(obj_class_num)->second)->depth);

  itmBasicEngine->GetImage(img,itmBasicEngine->InfiniTAM_IMAGE_COLOUR_FROM_VOLUME,itmBasicEngine->GetTrackingState()->pose_d,&(calib->intrinsics_d));
  SaveImageToFile(img,"vol.ppm");


}




}