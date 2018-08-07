//
// Created by khuang on 6/11/18.
//
#include <iostream>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Objects/RenderStates/ITMRenderStateFactory.h>

#include "DatasetReader_LPD_Dataset.h"
#include "ObjectInstanceScene.h"
//#include "ObjectInstanceScene.tpp"
#include "ObjectView_old.h"

#include "../../External/InfiniTAM/InfiniTAM/ITMLib/ITMLibDefines.h"

#include "ObjSLAMMappingEngine.h"
//#include "ObjectClassLabel_Group.h"
#include "ObjectInstance_New.h"


using namespace std;

int main(int argc, char **argv) {
  //TODO Debug output
  cout << "**Hello SLAM World!" << endl;

  //Path of the depth image file
  string path = argv[1];
  Vector2i imgSize(640, 480);

  ITMLib::ITMLibSettings *internalSettings = new ITMLib::ITMLibSettings();
  internalSettings->deviceType = ITMLib::ITMLibSettings::DEVICE_CPU;
  DatasetReader_LPD_Dataset reader(path, imgSize);
  int imgNum = reader.readNext();

  auto *mappingEngine2 =
      new ObjSLAM::ObjSLAMMappingEngine<ITMVoxel, ITMVoxelIndex>(internalSettings, reader.getCalib(), imgSize);
  mappingEngine2->UpdateImgNumber(imgNum);
  mappingEngine2->CreateView(*reader.getPose(), reader.depth_img, reader.rgb_img, reader.label_img_vector);
  mappingEngine2->UpdateTrackingState(&reader.getPose()->getSE3Pose());
  mappingEngine2->UpdateTrackingState_Orig(&reader.getPose()->getSE3Pose());
//  cout << reader.getPose()->getSE3Pose().GetM();

  //Pose test
/*  auto *pose_test = new ORUtils::SE3Pose(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
//  auto *pose_test2 = new ORUtils::SE3Pose(5.0f, 0.5f, 0.5f, 0.0f, 0.0f, 0.1f);

  cout << pose_test->GetM();
  mappingEngine2->UpdateTrackingState(pose_test);
//  mappingEngine2->UpdateTrackingState_Orig(pose_test2);*/

  mappingEngine2->ProcessFrame();


  int totFrames = 12;
for(int i = 0;i<totFrames;++i){
  imgNum = reader.readNext();
  mappingEngine2->UpdateImgNumber(imgNum);
//  cout << reader.getPose()->getSE3Pose().GetM();
  mappingEngine2->CreateView(*reader.getPose(), reader.depth_img, reader.rgb_img, reader.label_img_vector);
  mappingEngine2->UpdateTrackingState(&reader.getPose()->getSE3Pose());

  mappingEngine2->ProcessFrame();
}

  //old stuffs
/*//  ObjSLAM::Object_View_Tuple view_tuple = view0->getObjMap().find(58)->second;

//  engine_cpu->AllocateSceneFromDepth((ITMLib::ITMScene<ITMVoxel, ITMVoxelIndex> *) object,
//                                     std::get<1>(view_tuple),
//                                     trackingState,
//                                     renderState);
//  engine_cpu->IntegrateIntoScene(object, std::get<1>(view_tuple), trackingState, renderState);

//  cout << std::get<0>(view_tuple)->getClassLabel().getLabelIndex() << endl;  std::cout << "DEBUG" << std::endl;
//  cout << std::get<1>(view_tuple)->depth->GetElement(154610, MEMORYDEVICE_CPU) << endl;
//  cout << (int) (std::get<1>(view_tuple)->rgb->GetElement(154610, MEMORYDEVICE_CPU).w) << endl;

//  cout << "Scene Integration finish\n";
//  cout<<"noEntries"<<object->index.noTotalEntries<<endl;

//visualize
  ObjSLAM::ObjUChar4Image *img = new ObjSLAM::ObjUChar4Image(imgSize,MEMORYDEVICE_CPU);
//  auto *vis_eng_cpu = new ITMLib::ITMVisualisationEngine_CPU<ITMVoxel, ITMVoxelIndex>;

//
//  vis_eng_cpu->FindVisibleBlocks(scene, pose, intrinsics, renderState_freeview);
//  vis_eng_cpu->CreateExpectedDepths(scene, pose, intrinsics, renderState_freeview);
//  vis_eng_cpu->RenderImage(scene, pose, intrinsics, renderState_freeview, renderState_freeview->raycastImage, type);

//  cout << "Debug\n";

//  vis_eng_cpu->FindVisibleBlocks((ITMLib::ITMScene<ITMVoxel, ITMVoxelIndex>*)object, reader.getPose()->getSE3Pose(), &(calib->intrinsics_d), renderState);
//  vis_eng_cpu->CreateExpectedDepths((ITMLib::ITMScene<ITMVoxel, ITMVoxelIndex>*)object, reader.getPose()->getSE3Pose(), &(calib->intrinsics_d), renderState);
//  vis_eng_cpu->RenderImage((ITMLi/*b::ITMScene<ITMVoxel, ITMVoxelIndex>*)object, reader.getPose()->getSE3Pose(), &(calib->intrinsics_d),renderState,img,
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
//  object->SaveToDirectory(save_path);


  //delete mappingEngine2;
//  delete pose_test;*/

  return 0;
}