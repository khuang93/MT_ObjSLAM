//
// Created by khuang on 7/5/18.
//

#pragma once
#include "ObjSLAMMappingEngine.h"
#include <External/InfiniTAM/InfiniTAM/ORUtils/FileUtils.h>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Engines/Visualisation/ITMVisualisationEngineFactory.h>
#include "External/InfiniTAM/InfiniTAM/ITMLib/Objects/RenderStates/ITMRenderStateFactory.h"

namespace ObjSLAM {

//Constructor with LPD Dataset
template<typename TVoxel, typename TIndex>
ObjSLAMMappingEngine<TVoxel, TIndex>::ObjSLAMMappingEngine(ITMLib::ITMLibSettings *_settings,
                                                           string path,
                                                           Vector2i _imgSize):settings(_settings), imgSize(_imgSize) {
#ifndef COMPILE_WITHOUT_CUDA
  cout << "cuda" << endl;
#endif
  reader = DatasetReader_LPD_Dataset(path, imgSize);

  //process one frame
  reader.readNext();

  //float mu, int maxW, float voxelSize, float viewFrustum_min, float viewFrustum_max, bool stopIntegratingAtMaxW
  auto *params = new ITMLib::ITMSceneParams(0.5, 4, 0.01, 0.1, 4.0, false);

  auto *calib = reader.getCalib();

/*  auto *view0_o = new ObjSLAM::ObjectView_New(*calib,
                                              imgSize,
                                              imgSize,
                                              false,
                                              *reader.getPose(),
                                              reader.depth_img,
                                              reader.rgb_img,
                                              reader.label_img);*/

  auto *view0 = new ObjSLAM::ObjectView_New(*calib,
                                            imgSize,
                                            imgSize,
                                            false,
                                            *reader.getPose(),
                                            reader.depth_img,
                                            reader.rgb_img,
                                            reader.label_img_vector);

  vector<ObjSLAM::ObjectView_New *> ListofAllViews = {view0};

//  const ITMLib::ITMSceneParams *_sceneParams, bool _useSwapping, MemoryDeviceType _memoryType, ViewVector* _viewVector
//  auto *object = new ObjSLAM::ObjectInstanceScene_old<TVoxel, TIndex>(
//      params, true, MEMORYDEVICE_CPU, view0);

//TODO DEBUG
//  std::cout << "ObjectInstanceScene: " << std::is_copy_assignable<ObjSLAM::ObjectInstanceScene<ITMVoxel,ITMVoxelIndex>>::value << std::endl;

  //Tracking State
  t_state = new ITMLib::ITMTrackingState(imgSize, MEMORYDEVICE_CPU);

  ORUtils::SE3Pose pose_tstate = reader.getPose()->getSE3Pose();
  t_state->pose_d = &(pose_tstate);

  auto *engine_cpu = new ITMLib::ITMSceneReconstructionEngine_CPU<ITMVoxel, ITMVoxelIndex>;
  ITMLib::ITMLibSettings *internalSettings = new ITMLib::ITMLibSettings();
  internalSettings->deviceType = internalSettings->DEVICE_CPU;

//#pragma omp parallel
//#pragma omp for
  for (int t = 0; t < view0->getObjMap().size() - 10; t++) {
/*//    ObjectClassLabel lbl = std::get<0>(view0->getObjMap().at(t))->getClassLabel();
//    std::cout<<t<<endl;
//    ObjSLAM::ObjectInstanceScene<TVoxel, TIndex>* obj_inst_scene = NULL;
//    if(t==0)
//       obj_inst_scene = new ObjSLAM::ObjectInstanceScene<TVoxel, TIndex>(lbl,t,params,true, MEMORYDEVICE_CPU, view0);
//    else
//       obj_inst_scene = new ObjSLAM::ObjectInstanceScene<TVoxel, TIndex>(lbl,t,params,false, MEMORYDEVICE_CPU, view0);
//    object_instance_scene_vector.push_back(obj_inst_scene);
//
//    //RenderState
//    r_state = new ITMLib::ITMRenderState_VH(1,
//                                            imgSize,
//                                            obj_inst_scene->sceneParams->viewFrustum_min,
//                                            obj_inst_scene->sceneParams->viewFrustum_min,
//                                            MEMORYDEVICE_CPU);
//
//    engine_cpu->ResetScene(obj_inst_scene);*/
    ObjSLAM::ObjUChar4Image *img = new ObjSLAM::ObjUChar4Image(imgSize, MEMORYDEVICE_CPU);
    int obj_class_num = t;
    itmBasicEngine = new ITMLib::ITMBasicEngine<ITMVoxel, ITMVoxelIndex>(internalSettings, *calib, imgSize);
    itmBasicEngine->ProcessFrame(std::get<1>(view0->getObjMap().find(obj_class_num)->second)->rgb,
                                 std::get<1>(view0->getObjMap().find(obj_class_num)->second)->depth,
                                 NULL,
                                 reader.getPose()->getSE3Pose());

    itmBasicEngine->GetImage(img,
                             itmBasicEngine->InfiniTAM_IMAGE_COLOUR_FROM_VOLUME,
                             itmBasicEngine->GetTrackingState()->pose_d,
                             &(calib->intrinsics_d));

    string name = to_string(t) + ".ppm";

    SaveImageToFile(img, name.c_str());
//    itmBasicEngine->GetImage(img,itmBasicEngine->InfiniTAM_IMAGE_ORIGINAL_DEPTH,itmBasicEngine->GetTrackingState()->pose_d,&(calib->intrinsics_d));
//    SaveImageToFile(img,"orig.ppm");
  }

}

template<typename TVoxel, typename TIndex>
ObjSLAMMappingEngine<TVoxel, TIndex>::ObjSLAMMappingEngine(const ITMLib::ITMLibSettings *_settings,
                                                           const ITMLib::ITMRGBDCalib *_calib,
                                                           const Vector2i _imgSize):
    settings(_settings), calib(_calib), imgSize(_imgSize) {

  denseMapper = new ITMLib::ITMDenseMapper<TVoxel, TIndex>(settings);

  t_state = new ITMLib::ITMTrackingState(imgSize, MEMORYDEVICE_CPU);

  t_state->trackerResult = ITMLib::ITMTrackingState::TRACKING_GOOD;

  r_state =
      ITMLib::ITMRenderStateFactory<TIndex>::CreateRenderState(imgSize, &(settings->sceneParams), MEMORYDEVICE_CPU);

  visualisationEngine =
      ITMLib::ITMVisualisationEngineFactory::MakeVisualisationEngine<TVoxel, TIndex>(settings->deviceType);

}

template<typename TVoxel, typename TIndex>
void ObjSLAMMappingEngine<TVoxel, TIndex>::CreateView(ObjCameraPose pose,
                                                      ObjFloatImage *_depth,
                                                      ObjUChar4Image *_rgb,
                                                      LabelImgVector _label_img_vector) {
  if (settings->deviceType != ITMLib::ITMLibSettings::DEVICE_CUDA) {
    this->view = new ObjectView_New(*calib, imgSize, imgSize, false, pose, _depth, _rgb, _label_img_vector);
  } else {
    this->view = new ObjectView_New(*calib, imgSize, imgSize, true, pose, _depth, _rgb, _label_img_vector);
  }
};

template<typename TVoxel, typename TIndex>
void ObjSLAMMappingEngine<TVoxel, TIndex>::ProcessFrame() {
  bool useSwapping = (settings->swappingMode == ITMLib::ITMLibSettings::SWAPPINGMODE_ENABLED);
  if (view->getObjMap().size() > 0) {
    for (int t = 0; t < view->getObjMap().size(); t++) {
      Object_View_Tuple view_tuple = view->getObjMap().at(t);
      ObjectClassLabel label = std::get<0>(view_tuple)->getClassLabel();
      auto *obj_inst_scene = new ObjSLAM::ObjectInstanceScene<TVoxel, TIndex>(label,
                                                                              t,
                                                                              &(settings->sceneParams),
                                                                              useSwapping,
                                                                              MEMORYDEVICE_CPU,
                                                                              view);
      this->object_instance_scene_vector.push_back(obj_inst_scene);
      //ProcessOneObject
      ProcessOneObject(view_tuple, obj_inst_scene);
      delete obj_inst_scene;
    }
  }
}

template<typename TVoxel, typename TIndex>
void ObjSLAMMappingEngine<TVoxel, TIndex>::ProcessOneObject(Object_View_Tuple &view_tuple,
                                                            ObjectInstanceScene<TVoxel, TIndex> *scene) {

  std::shared_ptr<ITMLib::ITMView> itmView = std::get<1>(view_tuple);

  int index = std::get<0>(view_tuple)->getClassLabel().getLabelIndex();
  string name = to_string(index) + "_new.ppm";

  cout<<t_state->pose_d->GetM();

  denseMapper->ProcessFrame(itmView.get(), t_state, scene, r_state, true);
  cout << "dbg" << endl;
  ObjUChar4Image *img = new ObjUChar4Image(imgSize, MEMORYDEVICE_CPU);

  visualisationEngine->RenderImage(scene,
                                   t_state->pose_d,
                                   &itmView->calib.intrinsics_d,
                                   r_state,
                                   r_state->raycastImage,
                                   ITMLib::ITMVisualisationEngine<TVoxel, TIndex>::RENDER_COLOUR_FROM_VOLUME,
                                   ITMLib::ITMVisualisationEngine<TVoxel, TIndex>::RENDER_FROM_OLD_RAYCAST);

  itmBasicEngine = new ITMLib::ITMBasicEngine<ITMVoxel,ITMVoxelIndex>(settings,*calib,imgSize);
  itmBasicEngine->SetScene(scene);
  itmBasicEngine->GetImage(img,itmBasicEngine->InfiniTAM_IMAGE_COLOUR_FROM_VOLUME);

//  img->ChangeDims(r_state->raycastImage->noDims);
//  img->SetFrom(r_state->raycastImage, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);

  SaveImageToFile(img, name.c_str());
  cout << "dbg" << endl;
}

template<typename TVoxel, typename TIndex>
void ObjSLAMMappingEngine<TVoxel, TIndex>::UpdateTrackingState(const ORUtils::SE3Pose *_pose) {
  if (t_state == NULL) {
    t_state = new ITMLib::ITMTrackingState(imgSize, MEMORYDEVICE_CPU);
  }
  t_state->pose_d->SetFrom(_pose);
  t_state->pose_d->Coerce();
//  t_state->Reset();
}

template<typename TVoxel, typename TIndex>
void ObjSLAMMappingEngine<TVoxel, TIndex>::deleteAll() {
  delete this->itmBasicEngine;
  delete this->visualisationEngine;
  delete this->denseMapper;
  delete this->calib;
  delete this->settings;
  delete this->view;
  delete this->t_state;
  delete this->r_state;
}

}