//
// Created by khuang on 6/11/18.
//
#include <iostream>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Objects/RenderStates/ITMRenderStateFactory.h>

#include "DatasetReader_LPD_Dataset.h"
#include "ObjectInstanceScene.h"
#include "ObjectView_old.h"

#include "../../External/InfiniTAM/InfiniTAM/ITMLib/ITMLibDefines.h"

#include "ObjSLAMMappingEngine.h"
#include "ObjSLAMMappingEngine.tpp"

using namespace std;

int main(int argc, char **argv) {
  //TODO Debug output
  cout << "**Hello SLAM World!" << endl;

  //Path of the depth image file
  string path = argv[1];
  Vector2i imgSize(640,480);
  auto * mappingEngine = new ObjSLAM::ObjSLAMMappingEngine<ITMVoxel, ITMVoxelIndex>(path, imgSize);

//  ObjSLAM::Object_View_Tuple view_tuple = view0->getObjMap().find(58)->second;

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
//  object->SaveToDirectory(save_path);*/



  return 0;
}