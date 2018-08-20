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


  ITMLib::ITMView * wholeView = new ITMLib::ITMView(*reader.getCalib(),imgSize,imgSize,false);
  wholeView->depth = reader.depth_img;
  wholeView->rgb = reader.rgb_img;

  //create tracking engine
  auto * trackingEngine = new ObjSLAM::ObjSLAMTrackingEngine(internalSettings, reader.getCalib(), imgSize);

  auto t_controller =trackingEngine->getTrackingController();

  //create mapping engine
  auto *mappingEngine =
      new ObjSLAM::ObjSLAMMappingEngine<ITMVoxel, ITMVoxelIndex>(internalSettings, reader.getCalib(), imgSize);
  
  mappingEngine->SetTrackingController(t_controller);
  
  
  
  mappingEngine->UpdateImgNumber(imgNum);
  auto objview = mappingEngine->CreateView(reader.depth_img, reader.rgb_img, reader.label_img_vector);

//  auto t_state = trackingEngine->TrackFrame(objview.get()->getBackgroundView().get());
  auto t_state = trackingEngine->TrackFrame(wholeView);
  mappingEngine->UpdateTrackingState(t_state);

//  mappingEngine->UpdateTrackingState(&reader.getPose()->getSE3Pose());

//  mappingEngine->UpdateTrackingState_Orig(&reader.getPose()->getSE3Pose());
//  mappingEngine->UpdateTrackingState_Orig(t_state->pose_d);

  mappingEngine->ProcessFrame();

  mappingEngine->outputAllObjImages();

  delete wholeView;


  int totFrames = 13;
  for (int i = 1; i < totFrames; ++i) {
    imgNum = reader.readNext();

    wholeView = new ITMLib::ITMView(*reader.getCalib(),imgSize,imgSize,false);
    wholeView->depth = reader.depth_img;
    wholeView->rgb = reader.rgb_img;


    mappingEngine->UpdateImgNumber(imgNum);
//  cout << reader.getPose()->getSE3Pose().GetM();

    objview = mappingEngine->CreateView(reader.depth_img, reader.rgb_img, reader.label_img_vector);


    auto t_state = trackingEngine->TrackFrame(wholeView);

//    mappingEngine->UpdateTrackingState(&reader.getPose()->getSE3Pose());
    mappingEngine->UpdateTrackingState(t_state);

    mappingEngine->ProcessFrame();
    mappingEngine->outputAllObjImages();
    delete wholeView;
  }



  return 0;
}