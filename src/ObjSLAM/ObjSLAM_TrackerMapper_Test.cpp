//
// Created by khuang on 6/11/18.
//
#include <iostream>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Objects/RenderStates/ITMRenderStateFactory.h>

#include "LPD_Dataset_Reader.h"
#include "ObjectInstanceScene.h"
#include "../../External/InfiniTAM/InfiniTAM/ITMLib/ITMLibDefines.h"
#include "ObjSLAMMappingEngine.h"
#include "ObjSLAMTrackingEngine.h"
#include "ObjectView_New.h"
#include "TeddyReader.h"
#include "TUM_Reader.h"
#include <memory>

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

#include <ctime>

#include <src/ObjSLAM/ObjSLAMVoxelSceneParams.h>

using namespace std;


void ProcessOneFrame(){

}

int main(int argc, char **argv) {
  //TODO Debug output
  cout << "**Hello SLAM World!" << endl;
  cout<<ITMLib::ITMVoxelBlockHash::noTotalEntries<<endl;


  //Path of the depth image file
  string path = argv[1];
  Vector2i imgSize(640, 480);





  ITMLib::ITMLibSettings *internalSettings = new ITMLib::ITMLibSettings();
  internalSettings->sceneParams = ITMLib::ITMSceneParams(0.04f, 10, 0.004f, 0.2, 5.0, false);
  //(0.1, 10, 0.025, 0.1, 4.0, false); //(0.02f, 100, 0.002f, 0.2f, 3.0f, false);  //(0.2, 4, 0.05, 0.1, 4.0, false);

  internalSettings->deviceType = ITMLib::ITMLibSettings::DEVICE_CPU;

//  TeddyReader reader(path, imgSize);

//  LPD_Dataset_Reader reader(path, imgSize);
  DatasetReader* reader= nullptr;
  if(path.find("Teddy")!=std::string::npos){
    cout<<"Teddy\n";
    reader = new TeddyReader(path,imgSize);
  }else if(path.find("RealisticRenderingDataset")!=std::string::npos){
    cout<<"RealisticRenderingDataset\n";
    reader = new LPD_Dataset_Reader(path,imgSize);
  }else if(path.find("rgbd")!=std::string::npos || path.find("living")!=std::string::npos){
    cout<<"TUM RGBD\n";
    reader = new TUM_Reader(path,imgSize);
  }else{
    cout<<"Dataset not supported, programm will be terminated!\n";
    return 1;
  }




  int imgNum = reader->readNext();


  shared_ptr<ITMLib::ITMView> wholeView = make_shared<ITMLib::ITMView>(*reader->getCalib(),imgSize,imgSize,false);
  wholeView->depth->SetFrom(reader->depth_img,ORUtils::Image<float>::CPU_TO_CPU);
  wholeView->rgb ->SetFrom(reader->rgb_img,ORUtils::Image<Vector4u>::CPU_TO_CPU);
  SaveImageToFile(wholeView->depth,"TEST");
  //create tracking engine
  auto * trackingEngine = new ObjSLAM::ObjSLAMTrackingEngine(internalSettings, reader->getCalib(), imgSize);

  auto t_controller =trackingEngine->getTrackingController();

  //create mapping engine
  auto *mappingEngine =
      new ObjSLAM::ObjSLAMMappingEngine<ITMVoxel, ITMVoxelIndex>(internalSettings, reader->getCalib(), imgSize);

  mappingEngine->SetTrackingController(t_controller);

  
  
  mappingEngine->UpdateImgNumber(imgNum);
  mappingEngine->CreateView(reader->depth_img, reader->rgb_img, reader->label_img_vector);

//  auto t_state = trackingEngine->TrackFrame(objview. ->getBackgroundView().get());
  auto t_state = trackingEngine->TrackFrame(wholeView.get());

  mappingEngine->UpdateTrackingState(t_state);

  mappingEngine->ProcessFrame();

  mappingEngine->outputAllObjImages();





  int totFrames =atoi( argv[2]);
  while (imgNum<=totFrames) {
    std::clock_t start;
    double time;
    start = std::clock();
    imgNum = reader->readNext();
    cout<<sceneIsBackground<<endl;
    wholeView = make_shared<ITMLib::ITMView>(*reader->getCalib(),imgSize,imgSize,false);
    wholeView->depth->SetFrom(reader->depth_img,ORUtils::Image<float>::CPU_TO_CPU);
    wholeView->rgb ->SetFrom(reader->rgb_img,ORUtils::Image<Vector4u>::CPU_TO_CPU);



    cout<<sceneIsBackground<<endl;

    mappingEngine->UpdateImgNumber(imgNum);


    mappingEngine->CreateView(reader->depth_img, reader->rgb_img, reader->label_img_vector);


    auto t_state = trackingEngine->TrackFrame(wholeView.get());

//    mappingEngine->UpdateTrackingState(&reader->getPose()->getSE3Pose());
    mappingEngine->UpdateTrackingState(t_state);

    mappingEngine->ProcessFrame();
    mappingEngine->outputAllObjImages();

    time = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;

    cout<<"Img "<<imgNum<< " Time "<<time<<endl;

  }


  delete reader;
  return 0;
}