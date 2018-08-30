//
// Created by khuang on 7/5/18.
//

#pragma once

#include "ObjSLAMMappingEngine.h"
#include <External/InfiniTAM/InfiniTAM/ORUtils/FileUtils.h>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Engines/Visualisation/ITMVisualisationEngineFactory.h>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Engines/LowLevel/ITMLowLevelEngineFactory.h>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Trackers/ITMTrackerFactory.h>
#include "External/InfiniTAM/InfiniTAM/ITMLib/Objects/RenderStates/ITMRenderStateFactory.h"
#include "../../External/InfiniTAM/InfiniTAM/ITMLib/Utils/ITMLibSettings.h"
#include "../../External/InfiniTAM/InfiniTAM/ITMLib/Trackers/ITMTrackerFactory.h"
#include "../../External/InfiniTAM/InfiniTAM/ITMLib/Core/ITMTrackingController.h"
#include "ObjSLAMCamera.h"
#include "ObjectInstance_New.h"
#include <math.h>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Objects/Tracking/ITMTrackingState.h>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Objects/Meshing/ITMMesh.h>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Engines/Meshing/ITMMeshingEngineFactory.h>
//#include <omp.h>
namespace ObjSLAM {

//New Constructor with LPD Dataset
template<typename TVoxel, typename TIndex>
ObjSLAMMappingEngine<TVoxel, TIndex>::ObjSLAMMappingEngine(const ITMLib::ITMLibSettings *_settings,
                                                           const ITMLib::ITMRGBDCalib *_calib,
                                                           const Vector2i _imgSize):
    settings(_settings), calib(_calib), imgSize(_imgSize) {

  denseMapper = new ITMLib::ITMDenseMapper<TVoxel, TIndex>(settings);

//  t_state = new ITMLib::ITMTrackingState(imgSize, MEMORYDEVICE_CPU);

//  t_state->trackerResult = ITMLib::ITMTrackingState::TRACKING_GOOD;

  r_state =
      ITMLib::ITMRenderStateFactory<TIndex>::CreateRenderState(imgSize, &(settings->sceneParams),
                                                               MEMORYDEVICE_CPU);

  visualisationEngine =
      ITMLib::ITMVisualisationEngineFactory::MakeVisualisationEngine<TVoxel, TIndex>(settings->deviceType);
  //TODO Temp
  lowEngine = ITMLib::ITMLowLevelEngineFactory::MakeLowLevelEngine(settings->deviceType);

//  init_rgb_d_pair_idx();

}


template<typename TVoxel, typename TIndex>
shared_ptr<ObjectView_New<TVoxel, TIndex>> ObjSLAMMappingEngine<TVoxel, TIndex>::CreateView(ObjFloatImage *_depth,
                                                                                            ObjUChar4Image *_rgb,
                                                                                            LabelImgVector _label_img_vector) {

  if (settings->deviceType != ITMLib::ITMLibSettings::DEVICE_CUDA) {

    this->view_new = std::make_shared<ObjectView_New<TVoxel, TIndex>>(*calib,
                                                                      imgSize,
                                                                      imgSize,
                                                                      false,
                                                                      _depth,
                                                                      _rgb,
                                                                      _label_img_vector);
    this->view_new_vec.push_back(view_new);

  } else {
    this->view_new = std::make_shared<ObjectView_New<TVoxel, TIndex>>(*calib,
                                                                      imgSize,
                                                                      imgSize,
                                                                      false,
                                                                      _depth,
                                                                      _rgb,
                                                                      _label_img_vector);
    this->view_new_vec.push_back(view_new);
  }
  //init all objects in view
  view_new->setListOfObjects(label_ptr_vector);
  return view_new;
};

template<typename TVoxel, typename TIndex>
void ObjSLAMMappingEngine<TVoxel, TIndex>::ProcessFrame() {

  bool useSwapping = (settings->swappingMode != ITMLib::ITMLibSettings::SWAPPINGMODE_DISABLED);

  cout << "ProcessFrame...\n";

/*  for (int i = 0; i < label_ptr_vector.size(); i++) {
    cout << label_ptr_vector.at(i).get()->getLabelClassName() << " ";
  }
  cout << endl;*/

  if (view_new->getObjVec().size() > 0) {
    bool newObject = true;
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
    for (int t = 0; t < view_new->getObjVec().size(); t++) {

      //TODO hard code skipped background
//      if (t== 0) continue;

      Object_View_Tup<TVoxel, TIndex> view_tuple = view_new->getObjVec().at(t);

      auto obj_inst_ptr = std::get<0>(view_tuple);
      auto label_ptr = obj_inst_ptr->getClassLabel();

      int labelIndex = label_ptr->getLabelIndex();
      //TODO skid 76 to reduce memory
//          if(labelIndex!=0 && labelIndex!=58) continue;
//      if (/*labelIndex != 1 && labelIndex != 63 && */labelIndex != 0&&labelIndex!=78/*&& labelIndex!=67*/&&labelIndex!=58) continue;

      std::shared_ptr<ITMLib::ITMView> itmview = std::get<1>(view_tuple);
      string
          name =
          "Input_Frame" + to_string(imgNumber) + ".Label." + to_string(labelIndex) + "." + to_string(t) +
              ".ppm";
//      SaveImageToFile(itmview.get()->depth, name.c_str());


      auto obj_ptr_vec_val = label_ptr->getObjPtrVector();
      auto obj_ptr_vec = &obj_ptr_vec_val;

//      cout << *label_ptr.get() << endl << "obj_ptr_vec size before " << obj_ptr_vec->size() << endl;

      shared_ptr<ObjSLAM::ObjectInstanceScene<TVoxel, TIndex>> obj_inst_scene_ptr;

      if (obj_ptr_vec->size() == 0) {
        obj_inst_ptr->addObjectInstanceToLabel();
      } else {
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif

        for (size_t i = 0; i < obj_ptr_vec->size(); ++i) {
          std::shared_ptr<ObjectInstance_New<TVoxel, TIndex>> existing_obj_ptr = obj_ptr_vec->at(i);

          if (obj_inst_ptr->getClassLabel()->getLabelIndex() == 0) {
            newObject = false;
          } else {
            newObject = !this->checkIsSameObject2D(existing_obj_ptr, obj_inst_ptr);
          }

          if (!newObject) {
            //this is an existing object, no need to compare with further objs
            obj_inst_scene_ptr = existing_obj_ptr->getScene();
            break;
          }
        }
        if (newObject) obj_inst_ptr->addObjectInstanceToLabel();
      }
//      cout << "isNew? " << newObject << endl;

      if (newObject) {
        obj_inst_scene_ptr = std::make_shared<ObjectInstanceScene<TVoxel, TIndex>>(&(settings->sceneParams),
                                                                                   useSwapping,
                                                                                   MEMORYDEVICE_CPU);
        obj_inst_ptr->setScene(obj_inst_scene_ptr);
        denseMapper->ResetScene(obj_inst_scene_ptr.get());

      }
//      cout << "obj_ptr_vec size aft " << obj_ptr_vec->size() << endl;

      //ProcessOneObject
      ProcessOneObject(view_tuple, obj_inst_scene_ptr.get());

      //re-init the bool of newObject
      newObject = true;
    }
  }
}

//new method
template<typename TVoxel, typename TIndex>
void ObjSLAMMappingEngine<TVoxel, TIndex>::ProcessOneObject(Object_View_Tup<TVoxel, TIndex> &view_tuple,
                                                            ObjectInstanceScene<TVoxel, TIndex> *scene) {

  std::shared_ptr<ITMLib::ITMView> itmView = std::get<1>(view_tuple);
  auto obj_inst_ptr = std::get<0>(view_tuple);

  denseMapper->ProcessFrame(itmView.get(), t_state.get(), scene, r_state, true);
//  cout<<"ProcessFrame\n";
  denseMapper->UpdateVisibleList(itmView.get(), t_state.get(), scene, r_state, true);
//  cout<<"UpdateVisibleList\n";

//  cout<<"test"<<scene->index.GetEntries()[100].pos<<endl;

}

template<typename TVoxel, typename TIndex>
void ObjSLAMMappingEngine<TVoxel, TIndex>::outputAllObjImages() {
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
  for (size_t i = 0; i < this->label_ptr_vector.size(); ++i) {
    std::shared_ptr<ObjectClassLabel_Group<TVoxel, TIndex>> label_ptr = label_ptr_vector.at(i);
    std::vector<std::shared_ptr<ObjectInstance_New<TVoxel, TIndex>>>
        obj_inst_vec = (label_ptr.get()->getObjPtrVector());
    cout << *label_ptr.get() << " : " << obj_inst_vec.size() << endl;
//#ifdef WITH_OPENMP
//#pragma omp parallel for
//#endif
    for (size_t j = 0; j < obj_inst_vec.size(); ++j) {
      std::shared_ptr<ObjectInstance_New<TVoxel, TIndex>> obj_inst_ptr = obj_inst_vec.at(j);

      auto scene = obj_inst_ptr.get()->getScene();

//      ObjUChar4Image *img = new ObjUChar4Image(imgSize, MEMORYDEVICE_CPU);
      auto img = std::make_shared<ObjUChar4Image>(imgSize, MEMORYDEVICE_CPU);

      //ITMLib::ITMTrackingState *tmp_t_state = new ITMLib::ITMTrackingState(imgSize, MEMORYDEVICE_CPU);
      auto tmp_t_state = std::make_shared<ITMLib::ITMTrackingState>(imgSize, MEMORYDEVICE_CPU);
      tmp_t_state->pose_d->SetFrom(t_state->pose_d);

      if (obj_inst_ptr.get()->getClassLabel().get()->getLabelIndex() != 0) {

        t_controller->Prepare(tmp_t_state.get(),
                              scene.get(),
                              obj_inst_ptr.get()->getAnchorView_ITM().get(),
                              visualisationEngine,
                              r_state);

        visualisationEngine->RenderImage(scene.get(),
                                         t_state->pose_d,
                                         &obj_inst_ptr.get()->getAnchorView_ITM()->calib.intrinsics_d,
                                         r_state,
                                         r_state->raycastImage,
                                         ITMLib::ITMVisualisationEngine<TVoxel, TIndex>::RENDER_COLOUR_FROM_VOLUME,
                                         ITMLib::ITMVisualisationEngine<TVoxel, TIndex>::RENDER_FROM_OLD_RAYCAST);

      } else {
        //needed for tracking
        t_controller->Prepare(t_state.get(),
                              scene.get(),
                              obj_inst_ptr.get()->getAnchorView_ITM().get(),
                              visualisationEngine,
                              r_state);

        visualisationEngine->RenderImage(scene.get(),
                                         t_state->pose_d,
                                         &obj_inst_ptr.get()->getAnchorView_ITM()->calib.intrinsics_d,
                                         r_state,
                                         r_state->raycastImage,
                                         ITMLib::ITMVisualisationEngine<TVoxel, TIndex>::RENDER_COLOUR_FROM_VOLUME,
                                         ITMLib::ITMVisualisationEngine<TVoxel, TIndex>::RENDER_FROM_OLD_RAYCAST);

      }

      string name =
          "Label" + label_ptr.get()->getLabelClassName() + ".Object" + to_string(j) + ".Frame" +
              to_string(imgNumber)
              + ".ppm";

      img->ChangeDims(r_state->raycastImage->noDims);
      img->SetFrom(r_state->raycastImage, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);

      SaveImageToFile(img.get(), name.c_str());


    }
  }

  if (imgNumber % 10 == 0) {
    for (size_t i = 0; i < this->label_ptr_vector.size(); ++i) {
      std::vector<std::shared_ptr<ObjectInstance_New<TVoxel, TIndex>>>
          obj_inst_vec = (label_ptr_vector.at(i)->getObjPtrVector());
      for (size_t j = 0; j < obj_inst_vec.size(); ++j) {
        std::shared_ptr<ObjectInstance_New<TVoxel, TIndex>> obj_inst_ptr = obj_inst_vec.at(j);

        auto scene = obj_inst_ptr.get()->getScene();
        string stlname = to_string(label_ptr_vector.at(i)->getLabelIndex()) + "." + to_string(j) + ".stl";
        SaveSceneToMesh(stlname.c_str(), scene);
      }
    }
  }

};

template<typename TVoxel, typename TIndex>
bool ObjSLAMMappingEngine<TVoxel, TIndex>::checkIsNewObject(std::shared_ptr<ObjectInstance_New<TVoxel,TIndex>> obj_ptr) {
  bool isNew = true;

  auto new_obj_anchor = obj_ptr.get()->getAnchorView();

  auto *cam = new ObjSLAMCamera(this->calib, this->imgSize);
  std::shared_ptr<ObjectInstance_New<TVoxel, TIndex>> existing_obj_ptr;
  for (auto it = obj_inst_ptr_vector.begin(); it != obj_inst_ptr_vector.end(); ++it) {
    existing_obj_ptr = *it;

  }
}

//check if same obj by 2d overlap
template<typename TVoxel, typename TIndex>
bool ObjSLAMMappingEngine<TVoxel, TIndex>::checkIsSameObject2D(obj_inst_ptr<TVoxel,TIndex> obj_ptr_1, obj_inst_ptr<TVoxel,TIndex> obj_ptr_2){
//  cout<<"checkIsSameObject2D\n";
  bool isSame = false;
  ObjSLAM::ObjFloatImage* first = obj_ptr_1.get()->getAnchorView_ITM().get()->depth;
  ObjSLAM::ObjFloatImage* second = obj_ptr_2.get()->getAnchorView_ITM().get()->depth;

  auto *cam = new ObjSLAMCamera(this->calib, this->imgSize);

  auto* pcl = new ORUtils::Image<Vector4f>(imgSize,MEMORYDEVICE_CPU);//in world coordinate
  //TODO change pose to the pose from obj anchor view. add pose to itm view or let the obj inst save the pose itself.
  cam->projectImg2PointCloud(second,pcl, obj_ptr_2->getAnchorView()->getCameraPose().getSE3Pose());
//  cout<<*t_state->pose_d;

  ObjFloatImage *out = new ObjFloatImage(imgSize, MEMORYDEVICE_CPU);
  cam->projectPointCloud2Img(pcl, out, obj_ptr_1->getAnchorView()->getCameraPose().getSE3Pose());
//  cout<<*t_state_orig->pose_d;


  return checkImageOverlap(first, out);
}

template<typename TVoxel, typename TIndex>
bool ObjSLAMMappingEngine<TVoxel, TIndex>::checkImageOverlap(ObjSLAM::ObjFloatImage *first,
                                                             ObjSLAM::ObjFloatImage *second) {
//  cout<<"checkImageOverlap\n";
  //parameter to set which % of the pixels must match
  double threshold_areaChange = 0.5;
  double threshold_overlap = 0.7;

  int x1_min=imgSize.x-1;
  int x2_min=imgSize.x-1;
  int x1_max=0;
  int x2_max=0;

  int y1_min=imgSize.y-1;
  int y2_min=imgSize.y-1;
  int y1_max=0;
  int y2_max=0;

  for (size_t j = 0; j < imgSize.y; ++j) {
    for (size_t i = 0; i < imgSize.x; ++i) {
      int idx=j*imgSize.x+i;
      if(first->GetElement(idx, MEMORYDEVICE_CPU) > 0){
        if(i<x1_min) x1_min=i;
        if(i>x1_max) x1_max=i;
        if(j<y1_min) y1_min=j;
        if(j>y1_max) y1_max=j;
      }
      if(second->GetElement(idx, MEMORYDEVICE_CPU) > 0){
        if(i<x2_min) x2_min=i;
        if(i>x2_max) x2_max=i;
        if(j<y2_min) y2_min=j;
        if(j>y2_max) y2_max=j;
      }
    }
  }
//  cout<<x1_min<<" "<<x2_min<<" "<<x1_max<<" "<<x2_max<<"\n";
//  cout<<y1_min<<" "<<y2_min<<" "<<y1_max<<" "<<y2_max<<"\n";
  if(x1_min>x2_max || x2_min>x1_max || y1_min>y2_max || y2_min>y1_max) return false;

  float area_1 = (x1_max-x1_min)*(y1_max-y1_min);
  float area_2 = (x2_max-x2_min)*(y2_max-y2_min);

  int x_min_overlap = max(x1_min,x2_min);
  int y_min_overlap = max(y1_min,y2_min);
  int x_max_overlap = min(x1_max,x2_max);
  int y_max_overlap = min(y1_max,y2_max);

  float area_overlap = (x_max_overlap-x_min_overlap)*(y_max_overlap-y_min_overlap);

//  cout << "check" << min(area_1, area_2) / max(area_1, area_2) << " " << area_overlap / min(area_1, area_2) << endl;

  if (area_1 <= area_2) {
    return area_1 / area_2 > threshold_areaChange && area_overlap / area_1 > threshold_overlap;
  } else {
    return area_2 / area_1 > threshold_areaChange && area_overlap / area_2 > threshold_overlap;
  }
};



//check if same obj by 3d overlap
template<typename TVoxel, typename TIndex>
bool ObjSLAMMappingEngine<TVoxel, TIndex>::checkIsSameObject(obj_inst_ptr<TVoxel, TIndex> obj_ptr_1,
                                                             obj_inst_ptr<TVoxel, TIndex> obj_ptr_2) {

  ObjSLAM::ObjFloatImage *first = obj_ptr_1.get()->getAnchorView_ITM().get()->depth;
  ObjSLAM::ObjFloatImage *second = obj_ptr_2.get()->getAnchorView_ITM().get()->depth;

  auto *cam = new ObjSLAMCamera(this->calib, this->imgSize);

  auto *pcl1 = new ORUtils::Image<Vector4f>(imgSize, MEMORYDEVICE_CPU);//in world coordinate
  auto *pcl2 = new ORUtils::Image<Vector4f>(imgSize, MEMORYDEVICE_CPU);//in world coordinate
  //TODO change pose to the pose from obj anchor view. add pose to itm view or let the obj inst save the pose itself.
//  ORUtils::Vector6<float> cube1 = cam->projectImg2PointCloud(first,pcl1, *t_state_orig->pose_d);
  ORUtils::Vector6<float> cube1 =
      cam->projectImg2PointCloud(first, pcl1,
                                 obj_ptr_1.get()->getAnchorView().get()->getCameraPose().getSE3Pose());
  ORUtils::Vector6<float> cube2 = cam->projectImg2PointCloud(second, pcl2, *t_state->pose_d);

  delete cam;
  delete pcl1;
  delete pcl2;

  return checkBoundingCubeOverlap(cube1, cube2);
}


template<typename TVoxel, typename TIndex>
ORUtils::Vector4<int> ObjSLAMMappingEngine<TVoxel, TIndex>::getBoundingBox(ObjFloatImage *input) {

}

//using overlapping volumes. TODO add centroid position to further refine the estiamtion
template<typename TVoxel, typename TIndex>
bool ObjSLAMMappingEngine<TVoxel, TIndex>::checkBoundingCubeOverlap(ORUtils::Vector6<float> first,
                                                                    ORUtils::Vector6<float> second) {

  double threshold_volumeChange = 0.4;
  double threshold_overlap = 0.6;

  //case where there is 0 overlap
  if (second[0] > first[3] || second[1] > first[4] || second[2] > first[5] || first[0] > second[3]
      || first[1] > second[4] || first[2] > second[5])
    return false;

  double max_of_min_x = max(first[0], second[0]);
  double max_of_min_y = max(first[1], second[1]);
  double max_of_min_z = max(first[2], second[2]);

  double min_of_max_x = min(first[3], second[3]);
  double min_of_max_y = min(first[4], second[4]);
  double min_of_max_z = min(first[5], second[5]);

  ORUtils::Vector6<float> overlap(max_of_min_x, max_of_min_y, max_of_min_z, min_of_max_x, min_of_max_y,
                                  min_of_max_z);

  double v1 = calculateCubeVolume(first);
  double v2 = calculateCubeVolume(second);
  double v_overlap = calculateCubeVolume(overlap);

//  cout << "check" << min(v1, v2) / max(v1, v2) << " " << v_overlap / min(v1, v2) << endl;

  if (v1 <= v2) {
    return v1 / v2 > threshold_volumeChange && v_overlap / v1 > threshold_overlap;
  } else {
    return v2 / v1 > threshold_volumeChange && v_overlap / v2 > threshold_overlap;
  }
};

template<typename TVoxel, typename TIndex>
double ObjSLAMMappingEngine<TVoxel, TIndex>::calculateCubeVolume(ORUtils::Vector6<float> corners) {
  double l = (corners[3] - corners[0]);
  double w = (corners[4] - corners[1]);
  double h = (corners[5] - corners[2]);
  return l * w * h;
};






template<typename TVoxel, typename TIndex>
void ObjSLAMMappingEngine<TVoxel, TIndex>::UpdateTrackingState(const ORUtils::SE3Pose *_pose) {
//  cout << "UpdateTrackingState...\n";
  if (t_state == NULL) {
    t_state = new ITMLib::ITMTrackingState(imgSize, MEMORYDEVICE_CPU);
  }
  t_state->pose_d->SetFrom(_pose);
  t_state->pose_d->Coerce();

  this->UpdateViewPose();

//  t_state->Reset();
};

template<typename TVoxel, typename TIndex>
void ObjSLAMMappingEngine<TVoxel, TIndex>::UpdateTrackingState(shared_ptr<ITMLib::ITMTrackingState> _t_state) {
  t_state = _t_state;
  this->UpdateViewPose();
};

template<typename TVoxel, typename TIndex>
void ObjSLAMMappingEngine<TVoxel, TIndex>::UpdateTrackingState_Orig(const ORUtils::SE3Pose *_pose) {
//  cout << "UpdateTrackingState_Orig...\n";
  if (t_state_orig == NULL) {
    t_state_orig = new ITMLib::ITMTrackingState(imgSize, MEMORYDEVICE_CPU);
  }
  t_state_orig->pose_d->SetFrom(_pose);
  t_state_orig->pose_d->Coerce();

//  t_state->Reset();
};

template<typename TVoxel, typename TIndex>
void ObjSLAMMappingEngine<TVoxel, TIndex>::UpdateViewPose() {
  ObjCameraPose _pose(*this->t_state->pose_d);
  this->view_new->setCameraPose(_pose);
};

template<typename TVoxel, typename TIndex>
void ObjSLAMMappingEngine<TVoxel, TIndex>::SetTrackingController(
    shared_ptr<ITMLib::ITMTrackingController> _t_controller) {
  t_controller = _t_controller;
};

template<typename TVoxel, typename TIndex>
void ObjSLAMMappingEngine<TVoxel, TIndex>::outputAllLabelStats() {
  for (size_t i = 0; i < this->label_ptr_vector.size(); ++i) {
    std::shared_ptr<ObjectClassLabel_Group<TVoxel, TIndex>> label_ptr = label_ptr_vector.at(i);
    std::vector<std::shared_ptr<ObjectInstance_New<TVoxel, TIndex>>>
        obj_inst_vec = (label_ptr.get()->getObjPtrVector());
    cout << "Label " << *label_ptr.get()/*->getLabelClassName()*/<< " : " << obj_inst_vec.size() << endl;
  }
};

template<typename TVoxel, typename TIndex>
void ObjSLAMMappingEngine<TVoxel, TIndex>::deleteAll() {
  delete this->visualisationEngine;
  delete this->denseMapper;
  delete this->calib;
  delete this->settings;
//  delete this->view;
  delete this->t_state;
  delete this->r_state;
};

template<typename TVoxel, typename TIndex>
void ObjSLAMMappingEngine<TVoxel, TIndex>::SaveSceneToMesh(const char *objFileName,
                                                           std::shared_ptr<ObjectInstanceScene<TVoxel,
                                                                                               TIndex>> scene_ptr) {
  auto meshingEngine = ITMLib::ITMMeshingEngineFactory::MakeMeshingEngine<TVoxel, TIndex>(settings->deviceType);

  ITMLib::ITMMesh *mesh = new ITMLib::ITMMesh(settings->GetMemoryType());

  meshingEngine->MeshScene(mesh, scene_ptr.get());
  mesh->WriteSTL(objFileName);

  delete mesh;
};

template<typename TVoxel, typename TIndex>
std::vector<obj_inst_ptr<TVoxel, TIndex>> ObjSLAMMappingEngine<TVoxel, TIndex>::getObjInstPtrVec() {
  return this->obj_inst_ptr_vector;
};

template<typename TVoxel, typename TIndex>
std::vector<std::shared_ptr<ObjectClassLabel_Group<TVoxel, TIndex>>>
ObjSLAMMappingEngine<TVoxel, TIndex>::getLabelPtrVec() {
  return this->label_ptr_vector;
};

template<typename TVoxel, typename TIndex>
void ObjSLAMMappingEngine<TVoxel, TIndex>::init_rgb_d_pair_idx() {

  auto *cam = new ObjSLAMCamera(calib, imgSize);
  int size_x = imgSize.x;
  int size_y = imgSize.y;



  this->rgb_d_pixel_idx_vec = std::vector<int>(size_x * size_y, -1);
//  std::fill(rgb_d_pixel_idx_vec.begin(), rgb_d_pixel_idx_vec.end(), -1);
  if(calib->trafo_rgb_to_depth.calib.m00==1 &&
      calib->trafo_rgb_to_depth.calib.m11==1 &&
      calib->trafo_rgb_to_depth.calib.m22==1){
    for (int i = 0; i < rgb_d_pixel_idx_vec.size(); ++i) {

        this->rgb_d_pixel_idx_vec.at(i)=i;
      }
    return;
  }


  for (int y = 0; y < size_y; ++y) {
    for (int x = 0; x < size_x; ++x) {

      int idx_rgb = y * size_x + x;

      Vector4f pix_rgb((float) x, (float) y, 1.0f, 1.0f);
      Matrix4f K_rgb_inv;
      cam->getK_rgb().inv(K_rgb_inv);
      Vector4f pix_d = cam->getK_d() * calib->trafo_rgb_to_depth.calib * K_rgb_inv * pix_rgb;
//      cout<<"pix_d"<<pix_d<<endl;
      int x_d = round(pix_d.x);
      int y_d = round(pix_d.y);
      if (x_d >= 0 && y_d >= 0 && x_d < size_x && y_d < size_y) {
        this->rgb_d_pixel_idx_vec.at(idx_rgb) = y_d * size_x + x_d;
      }
    }
  }

  delete cam;
};

template<typename TVoxel, typename TIndex>
std::vector<int> ObjSLAMMappingEngine<TVoxel, TIndex>::get_rgb_d_pair_idx() {
  return this->rgb_d_pixel_idx_vec;
};

}