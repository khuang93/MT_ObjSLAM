//
// Created by khuang on 6/11/18.
//
#include <iostream>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Objects/RenderStates/ITMRenderStateFactory.h>

#include "DatasetReader_LPD_Dataset.h"
#include "ObjectInstanceScene.h"
#include "../../External/InfiniTAM/InfiniTAM/ITMLib/ITMLibDefines.h"
#include "ObjSLAMMappingEngine.h"
#include "ObjSLAMTrackingEngine.h"
#include "ObjectView_New.h"
#include <memory>

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

  //create tracking engine
  auto * trackingEngine = new ObjSLAM::ObjSLAMTrackingEngine(internalSettings, reader.getCalib(), imgSize);

  //create mapping engine
  auto *mappingEngine =
      new ObjSLAM::ObjSLAMMappingEngine<ITMVoxel, ITMVoxelIndex>(internalSettings, reader.getCalib(), imgSize);
  

  
  
  
  mappingEngine->UpdateImgNumber(imgNum);
  auto objview = mappingEngine->CreateView(*reader.getPose(), reader.depth_img, reader.rgb_img, reader.label_img_vector);
  trackingEngine->TrackFrame(objview.get()->getBackgroundView().get());


//  cout << reader.getPose()->getSE3Pose().GetM();
  mappingEngine->UpdateTrackingState(&reader.getPose()->getSE3Pose());
  mappingEngine->UpdateTrackingState_Orig(&reader.getPose()->getSE3Pose());


  //Pose test
/*  auto *pose_test = new ORUtils::SE3Pose(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
//  auto *pose_test2 = new ORUtils::SE3Pose(5.0f, 0.5f, 0.5f, 0.0f, 0.0f, 0.1f);

  cout << pose_test->GetM();
  mappingEngine2->UpdateTrackingState(pose_test);
//  mappingEngine2->UpdateTrackingState_Orig(pose_test2);*/

  mappingEngine->ProcessFrame();

//  mappingEngine2->outputAllLabelStats();
  mappingEngine->outputAllObjImages();

  int totFrames = 3;
  for (int i = 1; i < totFrames; ++i) {
    imgNum = reader.readNext();
    mappingEngine->UpdateImgNumber(imgNum);
//  cout << reader.getPose()->getSE3Pose().GetM();

    objview = mappingEngine->CreateView(*reader.getPose(), reader.depth_img, reader.rgb_img, reader.label_img_vector);
    trackingEngine->TrackFrame(objview.get()->getBackgroundView().get());

    mappingEngine->UpdateTrackingState(&reader.getPose()->getSE3Pose());

    mappingEngine->ProcessFrame();
    mappingEngine->outputAllObjImages();
  }



  return 0;
}