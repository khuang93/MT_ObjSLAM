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
#ifndef COMPILE_WITHOUT_CUDA
  cout<<"cuda"<<endl;
#endif
  reader = DatasetReader_LPD_Dataset(path, imgSize);

  //process one frame
  reader.readNext();

  //float mu, int maxW, float voxelSize, float viewFrustum_min, float viewFrustum_max, bool stopIntegratingAtMaxW
  auto *params = new ITMLib::ITMSceneParams(0.5, 4, 0.01, 0.1, 4.0, false);

  auto* calib = reader.getCalib();

  auto* view0_o = new ObjSLAM::ObjectView_New(*calib, imgSize, imgSize, false, *reader.getPose(), reader.depth_img, reader.rgb_img, reader.label_img);

  auto* view0 = new ObjSLAM::ObjectView_New(*calib, imgSize, imgSize, false, *reader.getPose(), reader.depth_img, reader.rgb_img, reader.label_img_vector);

  vector<ObjSLAM::ObjectView_New *> ListofAllViews = {view0};

  //const ITMLib::ITMSceneParams *_sceneParams, bool _useSwapping, MemoryDeviceType _memoryType, ViewVector* _viewVector
  auto *object = new ObjSLAM::ObjectInstanceScene_old<TVoxel, TIndex>(
      params, true, MEMORYDEVICE_CPU, view0);
  //TODO DEBUG
//  std::cout << "ObjectInstanceScene: " << std::is_copy_assignable<ObjSLAM::ObjectInstanceScene<ITMVoxel,ITMVoxelIndex>>::value << std::endl;

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
  ITMLib::ITMLibSettings *internalSettings = new ITMLib::ITMLibSettings();
  internalSettings->deviceType=internalSettings->DEVICE_CPU;
#pragma omp parallel
#pragma omp for
  for(size_t t =0; t<view0->getObjMap().size();t++){
    ObjectClassLabel lbl = std::get<0>(view0->getObjMap().at(t))->getClassLabel();
    std::cout<<t<<endl;
    ObjSLAM::ObjectInstanceScene<TVoxel, TIndex>* obj_inst_scene = NULL;
    if(t==0)
       obj_inst_scene = new ObjSLAM::ObjectInstanceScene<TVoxel, TIndex>(lbl,t,params,true, MEMORYDEVICE_CPU, view0);
    else
       obj_inst_scene = new ObjSLAM::ObjectInstanceScene<TVoxel, TIndex>(lbl,t,params,false, MEMORYDEVICE_CPU, view0);
    object_instance_scene_vector.push_back(obj_inst_scene);
    engine_cpu->ResetScene(object);
    ObjSLAM::ObjUChar4Image *img = new ObjSLAM::ObjUChar4Image(imgSize,MEMORYDEVICE_CPU);
    int obj_class_num =t;
    itmBasicEngine = new ITMLib::ITMBasicEngine<ITMVoxel,ITMVoxelIndex>(internalSettings,*calib,imgSize);
    itmBasicEngine->ProcessFrame(std::get<1>(view0->getObjMap().find(obj_class_num)->second)->rgb,std::get<1>(view0->getObjMap().find(obj_class_num)->second)->depth);
    itmBasicEngine->GetImage(img,itmBasicEngine->InfiniTAM_IMAGE_COLOUR_FROM_VOLUME,itmBasicEngine->GetTrackingState()->pose_d,&(calib->intrinsics_d));
    string name = "vol"+obj_class_num;
    name +=".ppm";
    SaveImageToFile(img,name.c_str());
//    itmBasicEngine->GetImage(img,itmBasicEngine->InfiniTAM_IMAGE_ORIGINAL_DEPTH,itmBasicEngine->GetTrackingState()->pose_d,&(calib->intrinsics_d));
//    SaveImageToFile(img,"orig.ppm");
  }




//  engine_cpu->ResetScene(object);

//  ObjSLAM::Object_View_Tuple view_tuple = view0->getObjMap().find(58)->second;


  //basic engine

//  int obj_class_num =0;

//  itmBasicEngine = new ITMLib::ITMBasicEngine<ITMVoxel,ITMVoxelIndex>(internalSettings,*calib,imgSize);

//  itmBasicEngine->ProcessFrame(std::get<1>(view0->getObjMap().find(obj_class_num)->second)->rgb,std::get<1>(view0->getObjMap().find(obj_class_num)->second)->depth);

//  itmBasicEngine->GetImage(img,itmBasicEngine->InfiniTAM_IMAGE_COLOUR_FROM_VOLUME,itmBasicEngine->GetTrackingState()->pose_d,&(calib->intrinsics_d));

//  reader.readNext();
//  auto * view1 = new ObjSLAM::ObjectView_New(*calib, imgSize, imgSize, false, *reader.getPose(), reader.depth_img, reader.rgb_img, reader.label_img_vector);
//  itmBasicEngine->ProcessFrame(std::get<1>(view1->getObjMap().find(obj_class_num)->second)->rgb,std::get<1>(view1->getObjMap().find(obj_class_num)->second)->depth);
//  reader.readNext();
//   view1 = new ObjSLAM::ObjectView_New(*calib, imgSize, imgSize, false, *reader.getPose(), reader.depth_img, reader.rgb_img, reader.label_img_vector);
//  itmBasicEngine->ProcessFrame(std::get<1>(view1->getObjMap().find(obj_class_num)->second)->rgb,std::get<1>(view1->getObjMap().find(obj_class_num)->second)->depth);


//  reader.readNext();
//  itmBasicEngine->ProcessFrame(std::get<1>(view0->getObjMap().find(obj_class_num)->second)->rgb,std::get<1>(view0->getObjMap().find(obj_class_num)->second)->depth);
//
//  itmBasicEngine->GetImage(img,itmBasicEngine->InfiniTAM_IMAGE_COLOUR_FROM_VOLUME,itmBasicEngine->GetTrackingState()->pose_d,&(calib->intrinsics_d));
//
//  SaveImageToFile(img,"vol.ppm");
//  itmBasicEngine->GetImage(img,itmBasicEngine->InfiniTAM_IMAGE_ORIGINAL_DEPTH,itmBasicEngine->GetTrackingState()->pose_d,&(calib->intrinsics_d));
//  SaveImageToFile(img,"orig.ppm");

//  cout<<itmBasicEngine->GetTrackingState()->pose_d->GetM();
//  cout<<reader.getPose()->getSE3Pose().GetM();
}

template <typename TVoxel, typename TIndex>
void ObjSLAMMappingEngine<TVoxel, TIndex>::bla(){
  std::cout<<"bla"<<endl;
}


}