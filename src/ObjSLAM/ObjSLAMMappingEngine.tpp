//
// Created by khuang on 7/5/18.
//

#pragma once

#include "ObjSLAMMappingEngine.h"

#include <math.h>

#include "ObjSLAMVoxelSceneParams.h"
#include <omp.h>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Objects/Scene/ITMVoxelBlockHash.h>

//define the global variable
bool sceneIsBackground = false;

namespace ObjSLAM {

template<class TVoxel, class TIndex>
void ObjSLAMMappingEngine<TVoxel, TIndex>::CreateView(ObjFloatImage *_depth,
                                                      ObjUChar4Image *_rgb,
                                                      LabelImgVector _label_img_vector) {

  if (settings->deviceType != ITMLib::ITMLibSettings::DEVICE_CUDA) {
    this->view = std::make_shared<ObjectView<TVoxel, TIndex>>(*calib,
                                                                      imgSize,
                                                                      imgSize,
                                                                      false,
                                                                      _depth,
                                                                      _rgb,
                                                                      _label_img_vector);
    this->view_vec.push_back(view);

  } else {
    this->view = std::make_shared<ObjectView<TVoxel, TIndex>>(*calib,
                                                                      imgSize,
                                                                      imgSize,
                                                                      false,
                                                                      _depth,
                                                                      _rgb,
                                                                      _label_img_vector);
    this->view_vec.push_back(view);
  }
  //init all objects in view
  view->setListOfObjects(label_ptr_vector);
  this->UpdateViewPose();
//    return view;
}

template<class TVoxel, class TIndex>
void ObjSLAMMappingEngine<TVoxel, TIndex>::ProcessFrame() {

  bool useSwapping = (settings->swappingMode != ITMLib::ITMLibSettings::SWAPPINGMODE_DISABLED);

  cout << "ProcessFrame...\n";

  if (view->getObjVec().size() > 0) {
    bool newObject = true;
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
    for (int t = 0; t < view->getObjVec().size(); t++) {

      Object_View_Tup<TVoxel, TIndex> view_tuple = view->getObjVec().at(t);

      auto obj_inst_ptr = std::get<0>(view_tuple);
      auto label_ptr = obj_inst_ptr->getClassLabel();

      int labelIndex = label_ptr->getLabelIndex();

      //set the flag of background or not
      sceneIsBackground = labelIndex == 0 ? true : false;


      //TODO skid 76 to reduce memory
//          if(labelIndex!=0 && labelIndex!=78) continue;
//      if (/*labelIndex != 1 && labelIndex != 63 && */labelIndex != 0&&labelIndex!=78/*&& labelIndex!=67*/&&labelIndex!=58) continue;

      std::shared_ptr<ITMLib::ITMView> itmview = std::get<1>(view_tuple);
//      string
//          name =
//          "Input_Frame" + to_string(imgNumber) + ".Label." + to_string(labelIndex) + "." + to_string(t) +
//              ".ppm";
//      SaveImageToFile(itmview.get()->depth, name.c_str());


      auto& obj_ptr_vec_val = label_ptr->getObjPtrVector();
      auto obj_ptr_vec = &obj_ptr_vec_val;

      shared_ptr<ObjSLAM::ObjectInstanceScene<TVoxel, TIndex>> obj_inst_scene_ptr;

      if (obj_ptr_vec->size() == 0) {
        obj_inst_ptr->addObjectInstanceToLabel();
        newObject = true;
      } else {
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif

        for (size_t i = 0; i < obj_ptr_vec->size(); ++i) {
          ObjectInstance_New_ptr<TVoxel, TIndex> existing_obj_ptr = obj_ptr_vec->at(i);

          if (obj_inst_ptr->getClassLabel()->getLabelIndex() == 0) {
            newObject = false;

          } else {
            //TODO hard coded each label 1 obj
            newObject =  /*false; */!this->checkIsSameObject2D(existing_obj_ptr, obj_inst_ptr);
          }
          if (!newObject) {
            //this is an existing object, no need to compare with further objs
            obj_inst_scene_ptr = existing_obj_ptr->getScene();

            obj_inst_ptr = existing_obj_ptr;
            break;
          }
        }
        if (newObject) {
          obj_inst_ptr->addObjectInstanceToLabel();
        }
      }
//      cout << "isNew? " << newObject << endl;

      if (newObject) {

        obj_inst_scene_ptr = std::make_shared<ObjectInstanceScene<TVoxel, TIndex>>(&(settings->sceneParams),
                                                                                   useSwapping,
                                                                                   MEMORYDEVICE_CPU);
        obj_inst_ptr->setScene(obj_inst_scene_ptr);

        std::shared_ptr<ITMLib::ITMRenderState> renderState_ptr( new ITMLib::ITMRenderState_VH((sceneIsBackground
                                                                                                ? ITMLib::ITMVoxelBlockHash::noTotalEntries_BG
                                                                                                : ITMLib::ITMVoxelBlockHash::noTotalEntries),
                                                                                               imgSize,
                                                                                               settings->sceneParams.viewFrustum_min,
                                                                                               settings->sceneParams.viewFrustum_max,
                                                                                               MEMORYDEVICE_CPU));
        obj_inst_ptr->setRenderState(renderState_ptr);

        auto t_state_ptr = std::make_shared<ITMLib::ITMTrackingState>(imgSize, MEMORYDEVICE_CPU);
        obj_inst_ptr->setTrackingState(t_state_ptr);

        denseMapper->ResetScene(obj_inst_scene_ptr.get());

      }
//      cout << "obj_ptr_vec size aft " << obj_ptr_vec->size() << endl;

      //ProcessOneObject
      ProcessOneObject(itmview, obj_inst_scene_ptr.get(), obj_inst_ptr);

      //re-init the bool of newObject
      newObject = true;
    }
  }
}

template<class TVoxel, class TIndex>
void ObjSLAMMappingEngine<TVoxel, TIndex>::ProcessOneObject(std::shared_ptr<ITMLib::ITMView> &itmview,
                                                            ObjectInstanceScene<TVoxel, TIndex> *scene,
                                                            ObjectInstance_New_ptr<TVoxel, TIndex> obj_inst_ptr) {

//  std::shared_ptr<ITMLib::ITMView> itmView = std::get<1>(view_tuple);
//  auto obj_inst_ptr = std::get<0>(view_tuple);

//  tmp_t_state->pose_d->SetFrom(t_state->pose_d);

  if (!obj_inst_ptr.get()->isBackground) {
    sceneIsBackground = false;
    std::shared_ptr<ITMLib::ITMTrackingState> tmp_t_state = obj_inst_ptr->getTrackingState();
//    tmp_t_state->pose_d->SetFrom(t_state->pose_d);
    obj_inst_ptr->getTrackingState()->Reset();
    obj_inst_ptr->getTrackingState()->pose_d->SetFrom(t_state->pose_d);
    denseMapper->ProcessFrame(itmview.get(), obj_inst_ptr->getTrackingState().get(), scene, obj_inst_ptr->getRenderState().get(), true);

    denseMapper->UpdateVisibleList(itmview.get(), obj_inst_ptr->getTrackingState().get(), scene, obj_inst_ptr->getRenderState().get(), true);

    t_controller->Prepare(obj_inst_ptr->getTrackingState().get(),
                          scene,
                          obj_inst_ptr.get()->getAnchorView_ITM(),
                          visualisationEngine,
                          obj_inst_ptr->getRenderState().get());
  } else {
    sceneIsBackground = true;
    denseMapper->ProcessFrame(itmview.get(), t_state.get(), scene, obj_inst_ptr->getRenderState().get(), true);

    denseMapper->UpdateVisibleList(itmview.get(), t_state.get(), scene, obj_inst_ptr->getRenderState().get(), true);

    t_controller->Prepare(t_state.get(),
                          scene,
                          obj_inst_ptr.get()->getAnchorView_ITM(),
                          visualisationEngine_BG,
                          obj_inst_ptr->getRenderState().get());
  }

}

template<class TVoxel, class TIndex>
void ObjSLAMMappingEngine<TVoxel, TIndex>::outputAllObjImages() {
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
  for (size_t i = 0; i < this->label_ptr_vector.size(); ++i) {
    std::shared_ptr<ObjectClassLabel_Group<TVoxel, TIndex>> label_ptr = label_ptr_vector.at(i);
    std::vector<ObjectInstance_New_ptr<TVoxel, TIndex>>&
        obj_inst_vec = (label_ptr.get()->getObjPtrVector());
    cout << *label_ptr.get() << " : " << obj_inst_vec.size() << endl;
//#ifdef WITH_OPENMP
//#pragma omp parallel for
//#endif
    for (size_t j = 0; j < obj_inst_vec.size(); ++j) {
      ObjectInstance_New_ptr<TVoxel, TIndex> obj_inst_ptr = obj_inst_vec.at(j);

      auto scene = obj_inst_ptr->getScene();

//      ObjUChar4Image *img = new ObjUChar4Image(imgSize, MEMORYDEVICE_CPU);
      auto img = std::make_shared<ObjUChar4Image>(imgSize, MEMORYDEVICE_CPU);


//        auto tmp_t_state = std::make_shared<ITMLib::ITMTrackingState>(imgSize, MEMORYDEVICE_CPU);
//        tmp_t_state->pose_d->SetFrom(t_state->pose_d);

      if (obj_inst_ptr->getLabelIndex() != 0) {
        sceneIsBackground = false;

        visualisationEngine->RenderImage(scene.get(),
                                         t_state->pose_d,
                                         &obj_inst_ptr.get()->getAnchorView_ITM()->calib.intrinsics_d,
                                         obj_inst_ptr->getRenderState().get(),
                                         obj_inst_ptr->getRenderState()->raycastImage,
                                         ITMLib::ITMVisualisationEngine<TVoxel, TIndex>::RENDER_COLOUR_FROM_VOLUME,
                                         ITMLib::ITMVisualisationEngine<TVoxel, TIndex>::RENDER_FROM_NEW_RAYCAST);
        img->ChangeDims(obj_inst_ptr->getRenderState().get()->raycastImage->noDims);
        img->SetFrom(obj_inst_ptr->getRenderState().get()->raycastImage, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);

      } else {
        sceneIsBackground = true;


        visualisationEngine_BG->RenderImage(scene.get(),
                                            t_state->pose_d,
                                            &obj_inst_ptr.get()->getAnchorView_ITM()->calib.intrinsics_d,
                                            obj_inst_ptr->getRenderState().get(),
                                            obj_inst_ptr->getRenderState()->raycastImage,
                                            ITMLib::ITMVisualisationEngine<TVoxel, TIndex>::RENDER_COLOUR_FROM_VOLUME,
                                            ITMLib::ITMVisualisationEngine<TVoxel, TIndex>::RENDER_FROM_NEW_RAYCAST);
        img->ChangeDims(obj_inst_ptr->getRenderState().get()->raycastImage->noDims);
        img->SetFrom(obj_inst_ptr->getRenderState().get()->raycastImage, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);

      }

      string name =
          "Label" + label_ptr.get()->getLabelClassName() + ".Object" + to_string(j) + ".Frame" +
              to_string(imgNumber)
              + ".ppm";

      SaveImageToFile(img.get(), name.c_str());

    }
  }
 //save stl
  if (saveSTL && imgNumber % STL_Frequency == 0) {
    for (size_t i = 0; i < this->label_ptr_vector.size(); ++i) {
      std::vector<ObjectInstance_New_ptr<TVoxel, TIndex>>
          obj_inst_vec = (label_ptr_vector.at(i)->getObjPtrVector());
      for (size_t j = 0; j < obj_inst_vec.size(); ++j) {
        ObjectInstance_New_ptr<TVoxel, TIndex> obj_inst_ptr = obj_inst_vec.at(j);
        if (obj_inst_ptr->getClassLabel()->getLabelIndex() == 0) sceneIsBackground = true;
        else sceneIsBackground = false;
        auto scene = obj_inst_ptr.get()->getScene();
        string stlname = to_string(label_ptr_vector.at(i)->getLabelIndex()) + "." + to_string(j) + ".stl";
        SaveSceneToMesh(stlname.c_str(), scene);
      }
    }
  }

}

//check if same obj by 2d overlap
template<class TVoxel, class TIndex>
bool ObjSLAMMappingEngine<TVoxel, TIndex>::checkIsSameObject2D(ObjectInstance_New_ptr<TVoxel, TIndex> obj_ptr_1,
                                                               ObjectInstance_New_ptr<TVoxel, TIndex> obj_ptr_2) {
//  cout<<"checkIsSameObject2D\n";
  bool isSame = false;
  ObjSLAM::ObjFloatImage *first = obj_ptr_1.get()->getAnchorView_ITM()->depth;
  ObjSLAM::ObjFloatImage *second = obj_ptr_2.get()->getAnchorView_ITM()->depth;

  auto * firstUchar4 = projectObjectToImg(obj_ptr_1);


  auto *cam = new ObjSLAMCamera(this->calib, this->imgSize);

  auto *pcl = new ORUtils::Image<Vector4f>(imgSize, MEMORYDEVICE_CPU);//in world coordinate
  //TODO change pose to the pose from obj anchor view. add pose to itm view or let the obj inst save the pose itself.
  cam->projectImg2PointCloud(second, pcl, obj_ptr_2->getAnchorView()->getCameraPose().getSE3Pose());
  //  cout<<*t_state->pose_d;

  ObjFloatImage *out = new ObjFloatImage(imgSize, MEMORYDEVICE_CPU);
  cam->projectPointCloud2Img(pcl, out, obj_ptr_1->getAnchorView()->getCameraPose().getSE3Pose());


    delete cam;
    delete pcl;
  return checkImageOverlap(firstUchar4, second)||checkImageOverlap(first, out);
//  return checkImageOverlap(first, out);
}

template<class TVoxel, class TIndex>
ORUtils::Image<Vector4u> * ObjSLAMMappingEngine<TVoxel, TIndex>::projectObjectToImg(ObjectInstance_New_ptr<TVoxel, TIndex> obj_inst_ptr){

  sceneIsBackground = obj_inst_ptr->isBackground;

  auto scene = obj_inst_ptr->getScene();

  visualisationEngine->RenderImage(scene.get(),
                                   t_state->pose_d,
                                   &obj_inst_ptr.get()->getAnchorView_ITM()->calib.intrinsics_d,
                                   obj_inst_ptr->getRenderState().get(),
                                   obj_inst_ptr->getRenderState()->raycastImage,
                                   ITMLib::ITMVisualisationEngine<TVoxel, TIndex>::RENDER_COLOUR_FROM_VOLUME,
                                   ITMLib::ITMVisualisationEngine<TVoxel, TIndex>::RENDER_FROM_NEW_RAYCAST);
  return obj_inst_ptr->getRenderState()->raycastImage;
}

template<class TVoxel, class TIndex>
bool ObjSLAMMappingEngine<TVoxel, TIndex>::checkImageOverlap(ObjSLAM::ObjFloatImage *first,
                                                             ObjSLAM::ObjFloatImage *second) {
//  cout<<"checkImageOverlap\n";
  //parameter to set which % of the pixels must match
  double threshold_areaChange = 0.05;
  double threshold_overlap = 0.4;

  int x1_min = imgSize.x - 1;
  int x2_min = imgSize.x - 1;
  int x1_max = 0;
  int x2_max = 0;

  int y1_min = imgSize.y - 1;
  int y2_min = imgSize.y - 1;
  int y1_max = 0;
  int y2_max = 0;

  for (size_t j = 0; j < imgSize.y; ++j) {
    for (size_t i = 0; i < imgSize.x; ++i) {
      int idx = j * imgSize.x + i;
      if (first->GetElement(idx, MEMORYDEVICE_CPU) > 0) {
        if (i < x1_min) x1_min = i;
        if (i > x1_max) x1_max = i;
        if (j < y1_min) y1_min = j;
        if (j > y1_max) y1_max = j;
      }
      if (second->GetElement(idx, MEMORYDEVICE_CPU) > 0) {
        if (i < x2_min) x2_min = i;
        if (i > x2_max) x2_max = i;
        if (j < y2_min) y2_min = j;
        if (j > y2_max) y2_max = j;
      }
    }
  }
//  cout<<x1_min<<" "<<x2_min<<" "<<x1_max<<" "<<x2_max<<"\n";
//  cout<<y1_min<<" "<<y2_min<<" "<<y1_max<<" "<<y2_max<<"\n";
  if (x1_min > x2_max || x2_min > x1_max || y1_min > y2_max || y2_min > y1_max) return false;

  float area_1 = (x1_max - x1_min) * (y1_max - y1_min);
  float area_2 = (x2_max - x2_min) * (y2_max - y2_min);

  int x_min_overlap = max(x1_min, x2_min);
  int y_min_overlap = max(y1_min, y2_min);
  int x_max_overlap = min(x1_max, x2_max);
  int y_max_overlap = min(y1_max, y2_max);

  float area_overlap = (x_max_overlap - x_min_overlap) * (y_max_overlap - y_min_overlap);

//  cout << "check" << min(area_1, area_2) / max(area_1, area_2) << " " << area_overlap / min(area_1, area_2) << endl;

  if (area_1 <= area_2) {
    return area_1 / area_2 > threshold_areaChange && area_overlap / area_1 > threshold_overlap;
  } else {
    return area_2 / area_1 > threshold_areaChange && area_overlap / area_2 > threshold_overlap;
  }
}

template<class TVoxel, class TIndex>
bool ObjSLAMMappingEngine<TVoxel, TIndex>::checkImageOverlap(ORUtils::Image<Vector4u>  *first,
                                                             ObjSLAM::ObjFloatImage *second) {
//  cout<<"checkImageOverlap\n";
  //parameter to set which % of the pixels must match
  double threshold_areaChange = 0.05;
  double threshold_overlap = 0.4;

  int x1_min = imgSize.x - 1;
  int x2_min = imgSize.x - 1;
  int x1_max = 0;
  int x2_max = 0;

  int y1_min = imgSize.y - 1;
  int y2_min = imgSize.y - 1;
  int y1_max = 0;
  int y2_max = 0;

  for (size_t j = 0; j < imgSize.y; ++j) {
    for (size_t i = 0; i < imgSize.x; ++i) {
      int idx = j * imgSize.x + i;
      if (first->GetElement(idx, MEMORYDEVICE_CPU) > 0) {
        if (i < x1_min) x1_min = i;
        if (i > x1_max) x1_max = i;
        if (j < y1_min) y1_min = j;
        if (j > y1_max) y1_max = j;
      }
      if (second->GetElement(idx, MEMORYDEVICE_CPU) > 0) {
        if (i < x2_min) x2_min = i;
        if (i > x2_max) x2_max = i;
        if (j < y2_min) y2_min = j;
        if (j > y2_max) y2_max = j;
      }
    }
  }
//  cout<<x1_min<<" "<<x2_min<<" "<<x1_max<<" "<<x2_max<<"\n";
//  cout<<y1_min<<" "<<y2_min<<" "<<y1_max<<" "<<y2_max<<"\n";
  if (x1_min > x2_max || x2_min > x1_max || y1_min > y2_max || y2_min > y1_max) return false;

  float area_1 = (x1_max - x1_min) * (y1_max - y1_min);
  float area_2 = (x2_max - x2_min) * (y2_max - y2_min);

  int x_min_overlap = max(x1_min, x2_min);
  int y_min_overlap = max(y1_min, y2_min);
  int x_max_overlap = min(x1_max, x2_max);
  int y_max_overlap = min(y1_max, y2_max);

  float area_overlap = (x_max_overlap - x_min_overlap) * (y_max_overlap - y_min_overlap);

//  cout << "check" << min(area_1, area_2) / max(area_1, area_2) << " " << area_overlap / min(area_1, area_2) << endl;

  if (area_1 <= area_2) {
    return area_1 / area_2 > threshold_areaChange && area_overlap / area_1 > threshold_overlap;
  } else {
    return area_2 / area_1 > threshold_areaChange && area_overlap / area_2 > threshold_overlap;
  }
}

//check if same obj by 3d overlap
template<class TVoxel, class TIndex>
bool ObjSLAMMappingEngine<TVoxel, TIndex>::checkIsSameObject(ObjectInstance_New_ptr<TVoxel, TIndex> obj_ptr_1,
                                                             ObjectInstance_New_ptr<TVoxel, TIndex> obj_ptr_2) {

  ObjSLAM::ObjFloatImage *first = obj_ptr_1.get()->getAnchorView_ITM()->depth;
  ObjSLAM::ObjFloatImage *second = obj_ptr_2.get()->getAnchorView_ITM()->depth;

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

template<class TVoxel, class TIndex>
ORUtils::Vector4<int> ObjSLAMMappingEngine<TVoxel, TIndex>::getBoundingBox(ObjFloatImage *input) {

}

//using overlapping volumes. TODO add centroid position to further refine the estiamtion
template<class TVoxel, class TIndex>
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
}

template<class TVoxel, class TIndex>
double ObjSLAMMappingEngine<TVoxel, TIndex>::calculateCubeVolume(ORUtils::Vector6<float> corners) {
  double l = (corners[3] - corners[0]);
  double w = (corners[4] - corners[1]);
  double h = (corners[5] - corners[2]);
  return l * w * h;
}

template<class TVoxel, class TIndex>
void ObjSLAMMappingEngine<TVoxel, TIndex>::UpdateTrackingState(const ORUtils::SE3Pose *_pose) {
//  cout << "UpdateTrackingState...\n";
  if (t_state == NULL) {
    t_state = new ITMLib::ITMTrackingState(imgSize, MEMORYDEVICE_CPU);
  }
  t_state->pose_d->SetFrom(_pose);
  t_state->pose_d->Coerce();

  this->UpdateViewPose();

//  t_state->Reset();
}

template<class TVoxel, class TIndex>
void ObjSLAMMappingEngine<TVoxel, TIndex>::UpdateTrackingState(shared_ptr<ITMLib::ITMTrackingState> _t_state) {
  t_state = _t_state;
//  this->UpdateViewPose();
}

//template<class TVoxel, class TIndex>
//void ObjSLAMMappingEngine<TVoxel, TIndex>::UpdateTrackingState_Orig(const ORUtils::SE3Pose *_pose) {
////  cout << "UpdateTrackingState_Orig...\n";
//  if (t_state_orig == NULL) {
//    t_state_orig = new ITMLib::ITMTrackingState(imgSize, MEMORYDEVICE_CPU);
//  }
//  t_state_orig->pose_d->SetFrom(_pose);
//  t_state_orig->pose_d->Coerce();
//
////  t_state->Reset();
//}

template<class TVoxel, class TIndex>
void ObjSLAMMappingEngine<TVoxel, TIndex>::UpdateViewPose() {
  ObjCameraPose _pose(*(this->t_state->pose_d));
  this->view->setCameraPose(_pose);
}

template<class TVoxel, class TIndex>
void ObjSLAMMappingEngine<TVoxel, TIndex>::SetTrackingController(
    shared_ptr<ITMLib::ITMTrackingController> _t_controller) {
  t_controller = _t_controller;
}

template<class TVoxel, class TIndex>
void ObjSLAMMappingEngine<TVoxel, TIndex>::outputAllLabelStats() {
  for (size_t i = 0; i < this->label_ptr_vector.size(); ++i) {
    std::shared_ptr<ObjectClassLabel_Group<TVoxel, TIndex>> label_ptr = label_ptr_vector.at(i);
    std::vector<ObjectInstance_New_ptr<TVoxel, TIndex>>&
        obj_inst_vec = (label_ptr.get()->getObjPtrVector());
    cout << "Label " << *label_ptr.get()/*->getLabelClassName()*/<< " : " << obj_inst_vec.size() << endl;
  }
}

template<class TVoxel, class TIndex>
void ObjSLAMMappingEngine<TVoxel, TIndex>::deleteAll() {
  delete this->visualisationEngine;
  delete this->visualisationEngine_BG;
  delete this->denseMapper;
//  delete this->calib;
//  delete this->settings;
//  delete this->t_state;
//  delete this->r_state;
//  delete this->r_state_BG;
}

template<class TVoxel, class TIndex>
void ObjSLAMMappingEngine<TVoxel, TIndex>::SaveSceneToMesh(const char *objFileName,
                                                           std::shared_ptr<ITMLib::ITMScene<TVoxel,
                                                                                            TIndex>> scene_ptr) {
  auto meshingEngine = ITMLib::ITMMeshingEngineFactory::MakeMeshingEngine<TVoxel, TIndex>(settings->deviceType);

  ITMLib::ITMMesh *this_mesh = new ITMLib::ITMMesh(settings->GetMemoryType());

  meshingEngine->MeshScene(this_mesh, scene_ptr.get());
  this_mesh->WriteSTL(objFileName);

  delete this_mesh;
}

}