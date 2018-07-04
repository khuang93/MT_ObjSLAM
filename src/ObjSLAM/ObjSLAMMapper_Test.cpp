//
// Created by khuang on 6/11/18.
//
#include <iostream>
#include <fstream>
#include <tuple>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Objects/RenderStates/ITMRenderStateFactory.h>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Core/ITMBasicEngine.h>

#include "DatasetReader_LPD_Dataset.h"
#include "ObjectInstanceScene.h"
#include "ObjectView_old.h"
#include "ObjSLAMBasicEngine.h"

#include "../../External/InfiniTAM/InfiniTAM/ITMLib/ITMLibDefines.h"
#include "External/InfiniTAM/InfiniTAM/ITMLib/Objects/Tracking/ITMTrackingState.h"
#include "External/InfiniTAM/InfiniTAM/ITMLib/Objects/RenderStates/ITMRenderState.h"
#include "External/InfiniTAM/InfiniTAM/ITMLib/Objects/RenderStates/ITMRenderState_VH.h"
#include "External/InfiniTAM/InfiniTAM/ITMLib/Engines/Reconstruction/CPU/ITMSceneReconstructionEngine_CPU.h"
#include "External/InfiniTAM/InfiniTAM/ITMLib/Engines/Reconstruction/CUDA/ITMSceneReconstructionEngine_CUDA.h"
//#include "External/InfiniTAM/InfiniTAM/ITMLib/Engines/Reconstruction/Interface/ITMSceneReconstructionEngine.h"

#include "External/InfiniTAM/InfiniTAM/Apps/InfiniTAM/UIEngine.h"
#include "External/InfiniTAM/InfiniTAM/ITMLib/Engines/Reconstruction/ITMSceneReconstructionEngineFactory.h"

#include "External/InfiniTAM/InfiniTAM/ITMLib/Engines/Visualisation/CPU/ITMVisualisationEngine_CPU.h"

using namespace std;

int main(int argc, char **argv) {
  //TODO Debug output
  cout << "**Hello SLAM World!" << endl;

  //Path of the depth image file
  string path = argv[1];

  std::istringstream iss(argv[2]);
  //for LPD dataset 1 -13
  int img_number;
  iss >> img_number;
  double time = img_number * 0.1;

  //create a reader and read inputs
  DatasetReader_LPD_Dataset reader(640, 480);

  reader.setCalib_LPD();

  reader.readNext(path);



  //create scene:

  //float mu, int maxW, float voxelSize, float viewFrustum_min, float viewFrustum_max, bool stopIntegratingAtMaxW
  ITMLib::ITMSceneParams *params = new ITMLib::ITMSceneParams(0.5, 4, 0.01, 0.1, 4.0, false);


  //Init View:
  //const ITMLib::ITMRGBDCalib& calibration, Vector2i imgSize_rgb, Vector2i imgSize_d, bool useGPU, ObjCameraPose pose
  ITMLib::ITMRGBDCalib *calib = reader.getCalib();
  Vector2i imgSize = reader.getSize();

  auto *view0 = new ObjSLAM::ObjectView_New(*calib, imgSize, imgSize, false, reader.pose_cw, reader.depth_img, reader.rgb_img, reader.label_img);

  //View List
  vector<ObjSLAM::ObjectView_New *> ListofAllViews = {view0};

  //const ITMLib::ITMSceneParams *_sceneParams, bool _useSwapping, MemoryDeviceType _memoryType, ObjectVector* _objVector, ViewVector* _viewVector
  auto *object = new ObjSLAM::ObjectInstanceScene<ITMVoxel, ITMVoxelIndex>(
      params, true, MEMORYDEVICE_CPU/*, objectVector*/, ListofAllViews);


  //Tracking State
  auto *trackingState = new ITMLib::ITMTrackingState(imgSize, MEMORYDEVICE_CPU);
  trackingState->pose_d = reader.pose_cw->getSE3Pose();

  //RenderState
  auto *renderState = new ITMLib::ITMRenderState_VH(1,
                                                    imgSize,
                                                    object->sceneParams->viewFrustum_min,
                                                    object->sceneParams->viewFrustum_min,
                                                    MEMORYDEVICE_CPU);



  auto *engine_cpu = new ITMLib::ITMSceneReconstructionEngine_CPU<ITMVoxel, ITMVoxelIndex>;
//  auto* engine_gpu = new ITMLib::ITMSceneReconstructionEngine_CUDA<ITMVoxel, ITMVoxelIndex>;

//  This gives weird linker errors
//  ITMLib::ITMSceneReconstructionEngine<ITMVoxel, ITMVoxelIndex>* engine2 = ITMLib::ITMSceneReconstructionEngineFactory::MakeSceneReconstructionEngine<ITMVoxel,ITMVoxelIndex>(ITMLib::ITMLibSettings::DEVICE_CPU);
  engine_cpu->ResetScene(object);

//  string save_path = "./before/";
//  object->SaveToDirectory(save_path);
//  cout<<"noEntries"<<object->index.noTotalEntries<<endl;


  ObjSLAM::Object_View_Tuple view_tuple = view0->getObjMap().find(58)->second;

/*//  engine_cpu->AllocateSceneFromDepth((ITMLib::ITMScene<ITMVoxel, ITMVoxelIndex> *) object,
//                                     std::get<1>(view_tuple),
//                                     trackingState,
//                                     renderState);
//  engine_cpu->IntegrateIntoScene(object, std::get<1>(view_tuple), trackingState, renderState);

//  cout << std::get<0>(view_tuple)->getClassLabel().getLabelIndex() << endl;  std::cout << "DEBUG" << std::endl;
//  cout << std::get<1>(view_tuple)->depth->GetElement(154610, MEMORYDEVICE_CPU) << endl;
//  cout << (int) (std::get<1>(view_tuple)->rgb->GetElement(154610, MEMORYDEVICE_CPU).w) << endl;

//  cout << "Scene Integration finish\n";
//  cout<<"noEntries"<<object->index.noTotalEntries<<endl;
//  save_path = "./aft/";
//  object->SaveToDirectory(save_path);
//  SaveImageToFile(std::get<1>(view_tuple)->depth, "DEPTH");
//  SaveImageToFile(std::get<1>(view_tuple)->rgb, "RGB");*/

//visualize
  ObjSLAM::ObjUChar4Image *img = new ObjSLAM::ObjUChar4Image(imgSize,MEMORYDEVICE_CPU);
/*//  auto *vis_eng_cpu = new ITMLib::ITMVisualisationEngine_CPU<ITMVoxel, ITMVoxelIndex>;

//
//  vis_eng_cpu->FindVisibleBlocks(scene, pose, intrinsics, renderState_freeview);
//  vis_eng_cpu->CreateExpectedDepths(scene, pose, intrinsics, renderState_freeview);
//  vis_eng_cpu->RenderImage(scene, pose, intrinsics, renderState_freeview, renderState_freeview->raycastImage, type);

//  cout << "Debug\n";

//  vis_eng_cpu->FindVisibleBlocks((ITMLib::ITMScene<ITMVoxel, ITMVoxelIndex>*)object, reader.pose_cw->getSE3Pose(), &(calib->intrinsics_d), renderState);
//  vis_eng_cpu->CreateExpectedDepths((ITMLib::ITMScene<ITMVoxel, ITMVoxelIndex>*)object, reader.pose_cw->getSE3Pose(), &(calib->intrinsics_d), renderState);
//  vis_eng_cpu->RenderImage((ITMLi/*b::ITMScene<ITMVoxel, ITMVoxelIndex>*)object, reader.pose_cw->getSE3Pose(), &(calib->intrinsics_d),renderState,img,
//                           ITMLib::ITMVisualisationEngine<ITMVoxel,ITMVoxelIndex>::RENDER_COLOUR_FROM_VOLUME,
//                           ITMLib::ITMVisualisationEngine<ITMVoxel,ITMVoxelIndex>::RENDER_FROM_NEW_RAYCAST);

//  cout<<(int)img->GetElement(1,MEMORYDEVICE_CPU).x<<endl;
//  SaveImageToFile(img,"recon.ppm");

//  cout << "Debug\n";
//  ITMLib::ITMLibSettings *internalSettings = new ITMLib::ITMLibSettings();
//  internalSettings->deviceType=internalSettings->DEVICE_CPU;
//
//  auto *denseMapper = new ITMLib::ITMDenseMapper<ITMVoxel,ITMVoxelIndex>(internalSettings);
//  denseMapper->ProcessFrame(std::get<1>(view_tuple),trackingState,(ITMLib::ITMScene<ITMVoxel, ITMVoxelIndex>*)object,renderState);
//
//  cout << "Debug\n";
//  cout<<"noEntries"<<object->index.noTotalEntries<<endl;
//  string save_path = "./aft_2/";
//  object->SaveToDirectory(save_path);*/



  //basic engine
  ITMLib::ITMLibSettings *internalSettings = new ITMLib::ITMLibSettings();
  internalSettings->deviceType=internalSettings->DEVICE_CPU;
  int obj_class_num = 58;
//  ObjSLAM::ObjUChar4Image *img = new ObjSLAM::ObjUChar4Image(imgSize,MEMORYDEVICE_CPU);
  auto* basicEngine = new ITMLib::ITMBasicEngine<ITMVoxel,ITMVoxelIndex>(internalSettings,*calib,imgSize);
  basicEngine->SetScene((ITMLib::ITMScene<ITMVoxel,ITMVoxelIndex>*)object);
  basicEngine->ProcessFrame(std::get<1>(view0->getObjMap().find(obj_class_num)->second)->rgb,std::get<1>(view0->getObjMap().find(obj_class_num)->second)->depth);

  reader.readNext(path);
  view0 = new ObjSLAM::ObjectView_New(*calib, imgSize, imgSize, false, reader.pose_cw, reader.depth_img, reader.rgb_img, reader.label_img);
  basicEngine->ProcessFrame(std::get<1>(view0->getObjMap().find(obj_class_num)->second)->rgb,std::get<1>(view0->getObjMap().find(obj_class_num)->second)->depth);

  reader.readNext(path);
  view0 = new ObjSLAM::ObjectView_New(*calib, imgSize, imgSize, false, reader.pose_cw, reader.depth_img, reader.rgb_img, reader.label_img);
  basicEngine->ProcessFrame(std::get<1>(view0->getObjMap().find(obj_class_num)->second)->rgb,std::get<1>(view0->getObjMap().find(obj_class_num)->second)->depth);
  reader.readNext(path);
  view0 = new ObjSLAM::ObjectView_New(*calib, imgSize, imgSize, false, reader.pose_cw, reader.depth_img, reader.rgb_img, reader.label_img);
  basicEngine->ProcessFrame(std::get<1>(view0->getObjMap().find(obj_class_num)->second)->rgb,std::get<1>(view0->getObjMap().find(obj_class_num)->second)->depth);


//  basicEngine->GetImage(img,basicEngine->InfiniTAM_IMAGE_ORIGINAL_RGB,reader.pose_cw->getSE3Pose(),&(calib->intrinsics_d));
//  SaveImageToFile(img,"orig_rgb.ppm");
  basicEngine->GetImage(img,basicEngine->InfiniTAM_IMAGE_COLOUR_FROM_VOLUME,basicEngine->GetTrackingState()->pose_d,&(calib->intrinsics_d));
  SaveImageToFile(img,"vol.ppm");
//  basicEngine->GetImage(img,basicEngine->InfiniTAM_IMAGE_FREECAMERA_SHADED,reader.pose_cw->getSE3Pose(),&(calib->intrinsics_d));
//  SaveImageToFile(img,"shaded.ppm");
  basicEngine->GetImage(img,basicEngine->InfiniTAM_IMAGE_COLOUR_FROM_VOLUME,reader.pose_cw->getSE3Pose(),&(calib->intrinsics_d));
  SaveImageToFile(img,"vol_otherPose.ppm");


  ofstream of;
  of.open("track.txt");
  of<<"TrackingState\n";
  of<<basicEngine->GetTrackingState()->pose_d->GetM()<<endl;
  of<<"GroundTruth\n";
  of<<reader.pose_cw->getSE3Pose()->GetM()<<endl;


  return 0;
}


