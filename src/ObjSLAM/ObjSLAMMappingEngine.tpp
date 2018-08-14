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

/*  auto *view0_o = new ObjSLAM::ObjectView(*calib,
                                              imgSize,
                                              imgSize,
                                              false,
                                              *reader.getPose(),
                                              reader.depth_img,
                                              reader.rgb_img,
                                              reader.label_img);*/

  auto *view0 = new ObjSLAM::ObjectView(*calib,
                                            imgSize,
                                            imgSize,
                                            false,
                                            *reader.getPose(),
                                            reader.depth_img,
                                            reader.rgb_img,
                                            reader.label_img_vector);

  vector<ObjSLAM::ObjectView *> ListofAllViews = {view0};

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
/*    itmBasicEngine->ProcessFrame(std::get<1>(view0->getObjMap().find(obj_class_num)->second)->rgb,
                                 std::get<1>(view0->getObjMap().find(obj_class_num)->second)->depth,
                                 NULL,
                                 reader.getPose()->getSE3Pose());*/

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
//New Constructor with LPD Dataset
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
  //TODO Temp
  lowEngine = ITMLib::ITMLowLevelEngineFactory::MakeLowLevelEngine(settings->deviceType);

}

template<typename TVoxel, typename TIndex>
void ObjSLAMMappingEngine<TVoxel, TIndex>::CreateView(ObjCameraPose pose,
                                                      ObjFloatImage *_depth,
                                                      ObjUChar4Image *_rgb,
                                                      LabelImgVector _label_img_vector) {
//  if(this->view!=NULL) delete this->view;
  if (settings->deviceType != ITMLib::ITMLibSettings::DEVICE_CUDA) {
//    this->view = new ObjectView(*calib, imgSize, imgSize, false, pose, _depth, _rgb, _label_img_vector);
    this->view_new = std::make_shared<ObjectView_New<TVoxel, TIndex>>(*calib, imgSize, imgSize, false, pose, _depth, _rgb, _label_img_vector);
  } else {
//    this->view = new ObjectView(*calib, imgSize, imgSize, true, pose, _depth, _rgb, _label_img_vector);
    this->view_new = std::make_shared<ObjectView_New<TVoxel, TIndex>>(*calib, imgSize, imgSize, false, pose, _depth, _rgb, _label_img_vector);
  }
};

template<typename TVoxel, typename TIndex>
void ObjSLAMMappingEngine<TVoxel, TIndex>::ProcessFrame() {
  bool useSwapping = (settings->swappingMode == ITMLib::ITMLibSettings::SWAPPINGMODE_DISABLED);

  //init all objects in view
//  std::vector<shared_ptr<ObjectClassLabel_Group<TVoxel,TIndex>>> label_ptr_vector;
//  auto label_ptr_vector_temp = view_new->setListOfObjects();
  view_new->setListOfObjects(label_ptr_vector);
  cout<<"ProcessFrame...\n";
//  for(int i = 0; i<label_ptr_vector_temp.size();i++) {
//    auto label_ptr_tmp = label_ptr_vector_temp.at(i);
//    ObjectView_New<TVoxel, TIndex>::addLabelToVector(this->label_ptr_vector, label_ptr_tmp);
//  }


  for(int i = 0; i<label_ptr_vector.size();i++){
    cout<<label_ptr_vector.at(i).get()->getLabelClassName()<<" ";
  }
  cout<<endl;

  if (view_new->getObjVec().size() > 0) {
    bool newObject = true;
    for (int t = 0; t < view_new->getObjVec().size(); t++) {

      //TODO hard code skipped background
//      if (t== 0) continue;

      Object_View_Tup<TVoxel,TIndex> view_tuple = view_new->getObjVec().at(t);

//      ObjectClassLabel label = std::get<0>(view_tuple).get()->getClassLabel();
      auto obj_inst_ptr  = std::get<0>(view_tuple);
      auto label_ptr = obj_inst_ptr.get()->getClassLabel();

      //TODO skid 76 to reduce memory
          int labelIndex = label_ptr.get()->getLabelIndex();

      if(/*labelIndex!=76&&*/labelIndex!=58) continue;


      std::shared_ptr<ITMLib::ITMView> itmview = std::get<1>(view_tuple);
      string name = "Input_Frame" +  to_string(imgNumber) +".Label." + to_string(labelIndex)  +"."+ to_string(t)+".ppm";

//      SaveImageToFile(itmview.get()->depth, name.c_str());


      auto obj_ptr_vec = label_ptr.get()->getObjPtrVector();

      cout<<*label_ptr.get()<<endl<<"obj_ptr_vec size before "<<obj_ptr_vec->size()<<endl;

      ObjSLAM::ObjectInstanceScene<TVoxel, TIndex> *obj_inst_scene = NULL;
      shared_ptr<ObjSLAM::ObjectInstanceScene<TVoxel, TIndex>> obj_inst_scene_ptr;


      if(obj_ptr_vec->size()==0){
        obj_inst_ptr.get()->addObjectInstanceToLabel();
      }else{
        for(size_t i = 0; i < obj_ptr_vec->size();++i){
          std::shared_ptr<ObjectInstance_New<TVoxel, TIndex>> existing_obj_ptr = obj_ptr_vec->at(i);

          if(obj_inst_ptr.get()->getClassLabel().get()->getLabelIndex()==0){

            newObject=false;
          }else{
            cout<<"isNew? "<<endl;
            newObject=!this->checkIsSameObject(existing_obj_ptr, obj_inst_ptr);
          }

          if(!newObject){
            //this is an existing object
            //TODO take existing scene and do further recon
            obj_inst_scene_ptr = existing_obj_ptr.get()->getScene();
            break;
          }
        }
        if(newObject) obj_inst_ptr.get()->addObjectInstanceToLabel();
      }
      cout<<"isNew? "<<newObject<<endl;
      if(newObject/*1*/){

        //TODO create scene and start recon

          obj_inst_scene_ptr = std::make_shared<ObjectInstanceScene<TVoxel, TIndex>>(&(settings->sceneParams),
                                                                                   useSwapping,
                                                                                   MEMORYDEVICE_CPU);
        obj_inst_ptr.get()->setScene(obj_inst_scene_ptr);
        this->object_instance_scene_vector.push_back(obj_inst_scene);
        denseMapper->ResetScene(obj_inst_scene_ptr.get());

      }else{

      }
      cout<<"obj_ptr_vec size aft "<<obj_ptr_vec->size()<<endl;







        //projection tests

//      }



//      obj_inst_ptr_vector.push_back(obj_inst_ptr);



//      if (object_instance_scene_vector.size() == 0) {
//
//        /*auto* */obj_inst_scene = new ObjSLAM::ObjectInstanceScene<TVoxel, TIndex>(/*label,
//                                                                                    t,*/
//                                                                                    &(settings->sceneParams),
//                                                                                    useSwapping,
//                                                                                    MEMORYDEVICE_CPU,
//                                                                                    view);
//        obj_inst_scene_ptr = std::make_shared<ObjectInstanceScene<TVoxel, TIndex>>(&(settings->sceneParams),
//                                                                                   useSwapping,
//                                                                                   MEMORYDEVICE_CPU,
//                                                                                   view);
//
//        obj_inst_ptr.get()->setScene(obj_inst_scene_ptr);
//        this->object_instance_scene_vector.push_back(obj_inst_scene);
//        denseMapper->ResetScene(obj_inst_scene);
//
//
//
//      } else {
//        obj_inst_scene =new ObjSLAM::ObjectInstanceScene<TVoxel, TIndex>(/*label,
//                                                                                    t,*/
//            &(settings->sceneParams),
//            useSwapping,
//            MEMORYDEVICE_CPU,
//            view);
//        denseMapper->ResetScene(obj_inst_scene);
////        obj_inst_scene= object_instance_scene_vector.at(0);
//      }

      //ProcessOneObject
      tracker = ITMLib::ITMTrackerFactory::Instance().Make(imgSize,
                                                           imgSize,
                                                           settings,
                                                           lowEngine,
                                                           new ITMLib::ITMIMUCalibrator_iPad(),
                                                           obj_inst_scene_ptr.get()->sceneParams);

      t_controller = new ITMLib::ITMTrackingController(tracker, settings);
//      cout<<"dbg\n";
      //TODO method for match old object
      ProcessOneObject(view_tuple, obj_inst_scene_ptr.get(), t);
      newObject = true;
       delete obj_inst_scene;//prevent memory full, but it will be needed for mapping
    }
    //re-init the bool of newObject

  }
}


//new method
template<typename TVoxel, typename TIndex>
void ObjSLAMMappingEngine<TVoxel, TIndex>::ProcessOneObject(Object_View_Tup<TVoxel,TIndex> &view_tuple,
                                                            ObjectInstanceScene<TVoxel, TIndex> *scene, int obj_idx) {



  std::shared_ptr<ITMLib::ITMView> itmView = std::get<1>(view_tuple);



  denseMapper->ProcessFrame(itmView.get(), t_state, scene, r_state, true);
  denseMapper->UpdateVisibleList(itmView.get(), t_state, scene, r_state, true);
//  cout << "dbg" << endl;
  ObjUChar4Image *img = new ObjUChar4Image(imgSize, MEMORYDEVICE_CPU);

//  t_controller->Prepare(t_state, scene, itmView.get(), visualisationEngine, r_state);

/*  visualisationEngine->RenderImage(scene,
                                   t_state->pose_d,
                                   &itmView->calib.intrinsics_d,
                                   r_state,
                                   r_state->raycastImage,
                                   ITMLib::ITMVisualisationEngine<TVoxel, TIndex>::RENDER_COLOUR_FROM_VOLUME,
                                   ITMLib::ITMVisualisationEngine<TVoxel, TIndex>::RENDER_FROM_OLD_RAYCAST);
  int index = std::get<0>(view_tuple).get()->getClassLabel().get()->getLabelIndex();
  string name = to_string(imgNumber) + "." + to_string(index) + ".ppm";
  img->ChangeDims(r_state->raycastImage->noDims);
  img->SetFrom(r_state->raycastImage, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);

  SaveImageToFile(img, name.c_str());*/

//  for(int i = 0; i<640*480;i++)
//  cout << r_state->raycastResult->GetData(MEMORYDEVICE_CPU)[0];
//  cout << t_state->pointCloud->locations->GetElement(0,MEMORYDEVICE_CPU);

//  delete tracker;
//  delete t_controller;

  delete img;

}

template<typename TVoxel, typename TIndex>
void ObjSLAMMappingEngine<TVoxel, TIndex>::ProcessOneObject(Object_View_Tuple &view_tuple,
                                                            ObjectInstanceScene<TVoxel, TIndex> *scene, int obj_idx) {



  std::shared_ptr<ITMLib::ITMView> itmView = std::get<1>(view_tuple);

  int index = std::get<0>(view_tuple)->getClassLabel().getLabelIndex();
  string name = to_string(obj_idx) + "." + to_string(index) + ".ppm";

  denseMapper->ProcessFrame(itmView.get(), t_state, scene, r_state, true);
  denseMapper->UpdateVisibleList(itmView.get(), t_state, scene, r_state, true);
  cout << "dbg" << endl;
  ObjUChar4Image *img = new ObjUChar4Image(imgSize, MEMORYDEVICE_CPU);

  t_controller->Prepare(t_state_orig, scene, itmView.get(), visualisationEngine, r_state);

  visualisationEngine->RenderImage(scene,
                                   t_state_orig->pose_d,
                                   &itmView->calib.intrinsics_d,
                                   r_state,
                                   r_state->raycastImage,
                                   ITMLib::ITMVisualisationEngine<TVoxel, TIndex>::RENDER_COLOUR_FROM_VOLUME,
                                   ITMLib::ITMVisualisationEngine<TVoxel, TIndex>::RENDER_FROM_OLD_RAYCAST);
  //Old stuffs with itmbasicengine
  /*
//  itmBasicEngine = new ITMLib::ITMBasicEngine<ITMVoxel,ITMVoxelIndex>(settings,*calib,imgSize);
//  cout<<t_state->pose_d->GetM();
//  itmBasicEngine->ProcessFrame(itmView.get()->rgb, itmView.get()->depth, t_state->pose_d);
////    itmBasicEngine->SetScene(scene);
//  itmBasicEngine->GetImage(img,itmBasicEngine->InfiniTAM_IMAGE_COLOUR_FROM_VOLUME);
*/





  img->ChangeDims(r_state->raycastImage->noDims);
  img->SetFrom(r_state->raycastImage, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);

  SaveImageToFile(img, name.c_str());

//  for(int i = 0; i<640*480;i++)
//  cout << r_state->raycastResult->GetData(MEMORYDEVICE_CPU)[0];
//  cout << t_state->pointCloud->locations->GetElement(0,MEMORYDEVICE_CPU);

//  delete tracker;
//  delete t_controller;

  delete img;

}
template<typename TVoxel, typename TIndex>
bool ObjSLAMMappingEngine<TVoxel, TIndex>::checkIsNewObject(std::shared_ptr<ObjectInstance_New<TVoxel,TIndex>> obj_ptr){
  bool isNew = true;

  auto new_obj_anchor = obj_ptr.get()->getAnchorView();

  auto *cam = new ObjSLAMCamera(this->calib, this->imgSize);
  std::shared_ptr<ObjectInstance_New<TVoxel,TIndex>> existing_obj_ptr;
  for(auto it=obj_inst_ptr_vector.begin();it!=obj_inst_ptr_vector.end();++it){
    existing_obj_ptr=*it;

  }
}

//check if same obj by 2d overlap
/*template<typename TVoxel, typename TIndex>
bool ObjSLAMMappingEngine<TVoxel, TIndex>::checkIsSameObject(obj_inst_ptr<TVoxel,TIndex> obj_ptr_1, obj_inst_ptr<TVoxel,TIndex> obj_ptr_2){
  bool isSame = false;
  ObjSLAM::ObjFloatImage* first = obj_ptr_1.get()->getAnchorView_ITM().get()->depth;
  ObjSLAM::ObjFloatImage* second = obj_ptr_2.get()->getAnchorView_ITM().get()->depth;

  auto *cam = new ObjSLAMCamera(this->calib, this->imgSize);

  auto* pcl = new ORUtils::Image<Vector4f>(imgSize,MEMORYDEVICE_CPU);//in world coordinate
  //TODO change pose to the pose from obj anchor view. add pose to itm view or let the obj inst save the pose itself.
  cam->projectImg2PointCloud(second,pcl, *t_state->pose_d);
//  cout<<*t_state->pose_d;

  ObjFloatImage *out = new ObjFloatImage(imgSize, MEMORYDEVICE_CPU);
  cam->projectPointCloud2Img(pcl, out, *t_state_orig->pose_d);
//  cout<<*t_state_orig->pose_d;


  return checkImageOverlap(first, out);
}*/

//check if same obj by 3d overlap
template<typename TVoxel, typename TIndex>
bool ObjSLAMMappingEngine<TVoxel, TIndex>::checkIsSameObject(obj_inst_ptr<TVoxel,TIndex> obj_ptr_1, obj_inst_ptr<TVoxel,TIndex> obj_ptr_2){
  bool isSame = false;
  ObjSLAM::ObjFloatImage* first = obj_ptr_1.get()->getAnchorView_ITM().get()->depth;
  ObjSLAM::ObjFloatImage* second = obj_ptr_2.get()->getAnchorView_ITM().get()->depth;

  auto *cam = new ObjSLAMCamera(this->calib, this->imgSize);

  auto* pcl1 = new ORUtils::Image<Vector4f>(imgSize,MEMORYDEVICE_CPU);//in world coordinate
  auto* pcl2 = new ORUtils::Image<Vector4f>(imgSize,MEMORYDEVICE_CPU);//in world coordinate
  //TODO change pose to the pose from obj anchor view. add pose to itm view or let the obj inst save the pose itself.
  ORUtils::Vector6<float> cube1 = cam->projectImg2PointCloud(first,pcl1, *t_state_orig->pose_d);
  ORUtils::Vector6<float> cube2 = cam->projectImg2PointCloud(second,pcl2, *t_state->pose_d);



  return checkBoundingCubeOverlap(cube1, cube2);
}

/*
template<typename TVoxel, typename TIndex>
bool ObjSLAMMappingEngine<TVoxel, TIndex>::checkImageOverlap(ObjSLAM::ObjFloatImage* first, ObjSLAM::ObjFloatImage* second){
  bool has_enough_overlap = false;
  double threshold = 0.9;
  double true_count = 0.0;
  double unmatch_count = 0.0;

  cout<<"t "<<true_count<<"umatch"<<unmatch_count<<endl;
  has_enough_overlap = 1 - unmatch_count/true_count > threshold;
  cout<<"match? "<<has_enough_overlap<<endl;


  string name = "sec.ppm";
  SaveImageToFile(second, name.c_str());
  string nameIn = "first.ppm";
  SaveImageToFile(first, nameIn.c_str());

  return has_enough_overlap;
}
*/

template<typename TVoxel, typename TIndex>
ORUtils::Vector4<int> ObjSLAMMappingEngine<TVoxel, TIndex>::getBoundingBox(ObjFloatImage* input){

}


//using overlapping volumes. TODO add centroid position to further refine the estiamtion
template<typename TVoxel, typename TIndex>
bool ObjSLAMMappingEngine<TVoxel, TIndex>::checkBoundingCubeOverlap(ORUtils::Vector6<float> first,ORUtils::Vector6<float> second){

  double threshold_volumeChange = 0.35;
  double threshold_overlap = 0.55;

  //case where there is 0 overlap
  if(second[0]>first[3]||second[1]>first[4]||second[2]>first[5]||first[0]>second[3]||first[1]>second[4]||first[2]>second[5])
    return false;

  double max_of_min_x = max(first[0],second[0]);
  double max_of_min_y = max(first[1],second[1]);
  double max_of_min_z = max(first[2],second[2]);

  double min_of_max_x = min(first[3],second[3]);
  double min_of_max_y = min(first[4],second[4]);
  double min_of_max_z = min(first[5],second[5]);

  ORUtils::Vector6<float> overlap(max_of_min_x,max_of_min_y,max_of_min_z,min_of_max_x,min_of_max_y,min_of_max_z);

  double v1 = calculateCubeVolume(first);
  double v2 = calculateCubeVolume(second);
  double v_overlap = calculateCubeVolume(overlap);

  cout<<"check"<<min(v1,v2)/max(v1,v2)<<" "<<v_overlap/min(v1,v2)<<endl;

  if(v1<=v2){
    return v1/v2>threshold_volumeChange && v_overlap/v1>threshold_overlap;
  }else{
    return v2/v1>threshold_volumeChange && v_overlap/v2>threshold_overlap;
  }
};

template<typename TVoxel, typename TIndex>
double ObjSLAMMappingEngine<TVoxel, TIndex>::calculateCubeVolume(ORUtils::Vector6<float> corners){
  double l=(corners[3]-corners[0]);
  double w=(corners[4]-corners[1]);
  double h=(corners[5]-corners[2]);
  return l*w*h;
};

template<typename TVoxel, typename TIndex>
bool ObjSLAMMappingEngine<TVoxel, TIndex>::checkImageOverlap(ObjSLAM::ObjFloatImage* first, ObjSLAM::ObjFloatImage* second){
  bool has_enough_overlap = false;
  //parameter to set which % of the pixels must match
  double threshold = 0.7;
  double true_count_1 = 0.0;
  double true_count_2= 0.0;
  double unmatch_count = 0.0;
  for(size_t i= 0; i<imgSize.x*imgSize.y; ++i){
    bool first_pix = first->GetElement(i ,MEMORYDEVICE_CPU)>0;
    if (first_pix) true_count_1++;

    bool second_pix = second->GetElement(i ,MEMORYDEVICE_CPU)>0;
    if (second_pix) true_count_2++;
    if(first_pix!=second_pix) unmatch_count++;
  }
  double true_count = max(true_count_1,true_count_2);
  cout<<"t "<<true_count<<"umatch"<<unmatch_count<<endl;
  has_enough_overlap = 1 - unmatch_count/true_count > threshold;
//  cout<<"match? "<<has_enough_overlap<<endl;


  string name = "sec.ppm";
  SaveImageToFile(second, name.c_str());
  string nameIn = "first.ppm";
  SaveImageToFile(first, nameIn.c_str());

  return has_enough_overlap;
}

template<typename TVoxel, typename TIndex>
void ObjSLAMMappingEngine<TVoxel, TIndex>::UpdateTrackingState(const ORUtils::SE3Pose *_pose) {
  if (t_state == NULL) {
    t_state = new ITMLib::ITMTrackingState(imgSize, MEMORYDEVICE_CPU);
  }
  t_state->pose_d->SetFrom(_pose);
  t_state->pose_d->Coerce();

  this->UpdateViewPose();
//  t_state->Reset();
}

template<typename TVoxel, typename TIndex>
void ObjSLAMMappingEngine<TVoxel, TIndex>::UpdateTrackingState_Orig(const ORUtils::SE3Pose *_pose) {
  if (t_state_orig == NULL) {
    t_state_orig = new ITMLib::ITMTrackingState(imgSize, MEMORYDEVICE_CPU);
  }
  t_state_orig->pose_d->SetFrom(_pose);
  t_state_orig->pose_d->Coerce();
//  t_state->Reset();
}

template<typename TVoxel, typename TIndex>
void ObjSLAMMappingEngine<TVoxel, TIndex>::UpdateViewPose(){
  ObjCameraPose _pose(*this->t_state->pose_d);
  this->view_new->setCameraPose(_pose);
};


template<typename TVoxel, typename TIndex>
void ObjSLAMMappingEngine<TVoxel, TIndex>::outputAllLabelStats() {
  for(size_t i = 0; i< this->label_ptr_vector.size();++i){
    std::shared_ptr<ObjectClassLabel_Group<TVoxel, TIndex>> label_ptr = label_ptr_vector.at(i);
    std::vector<std::shared_ptr<ObjectInstance_New<TVoxel, TIndex>>> obj_inst_vec = *(label_ptr.get()->getObjPtrVector());
    cout<<"Label "<<*label_ptr.get()/*->getLabelClassName()*/<<" : "<<obj_inst_vec.size()<<endl;
//    for(size_t j = 0; j<obj_inst_vec.size();++j){
//      std::shared_ptr<ObjectInstance_New<TVoxel, TIndex>> obj_inst_ptr = obj_inst_vec.at(j);
//      cout<<j<<" ";
//    }
  }
//  cout<<endl;
};


template<typename TVoxel, typename TIndex>
void ObjSLAMMappingEngine<TVoxel, TIndex>::outputAllObjImages() {
  for(size_t i = 0; i< this->label_ptr_vector.size();++i){
    std::shared_ptr<ObjectClassLabel_Group<TVoxel, TIndex>> label_ptr = label_ptr_vector.at(i);
    std::vector<std::shared_ptr<ObjectInstance_New<TVoxel, TIndex>>> obj_inst_vec = *(label_ptr.get()->getObjPtrVector());
    cout<<*label_ptr.get()/*->getLabelClassName()*/<<" : "<<obj_inst_vec.size()<<endl;
    for(size_t j = 0; j<obj_inst_vec.size();++j){
      std::shared_ptr<ObjectInstance_New<TVoxel, TIndex>> obj_inst_ptr = obj_inst_vec.at(j);




      auto scene = obj_inst_ptr.get()->getScene();

      ObjUChar4Image *img = new ObjUChar4Image(imgSize, MEMORYDEVICE_CPU);
      t_controller->Prepare(t_state, scene.get(), obj_inst_ptr.get()->getAnchorView_ITM().get(), visualisationEngine, r_state);

      visualisationEngine->RenderImage(scene.get(),
                                       t_state->pose_d,
                                       &obj_inst_ptr.get()->getAnchorView_ITM()->calib.intrinsics_d,
                                       r_state,
                                       r_state->raycastImage,
                                       ITMLib::ITMVisualisationEngine<TVoxel, TIndex>::RENDER_COLOUR_FROM_VOLUME,
                                       ITMLib::ITMVisualisationEngine<TVoxel, TIndex>::RENDER_FROM_OLD_RAYCAST);


      string name = "Label"+ label_ptr.get()->getLabelClassName() + ".Object" + to_string(j)+ ".Frame" +  to_string(imgNumber) + ".ppm";
      img->ChangeDims(r_state->raycastImage->noDims);
      img->SetFrom(r_state->raycastImage, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);

      SaveImageToFile(img, name.c_str());
      string name2 = "Input_Label"+ label_ptr.get()->getLabelClassName() + ".Object" + to_string(j)+ ".Frame" +  to_string(imgNumber) + ".ppm";



    }
  }
//  cout<<endl;
};

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