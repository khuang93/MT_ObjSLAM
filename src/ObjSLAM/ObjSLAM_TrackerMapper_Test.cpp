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
#include "TeddyReader.h"
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
  internalSettings->deviceType = ITMLib::ITMLibSettings::DEVICE_CPU;

//  TeddyReader reader(path, imgSize);

//  DatasetReader_LPD_Dataset reader(path, imgSize);
  DatasetReader* reader= nullptr;
  if(path.find("Teddy")!=std::string::npos){
    cout<<"Teddy\n";
    reader = new TeddyReader(path,imgSize);
  }else if(path.find("RealisticRenderingDataset")!=std::string::npos){
    cout<<"RealisticRenderingDataset\n";
    reader = new DatasetReader_LPD_Dataset(path,imgSize);
  }else{
    cout<<"Dataset not supported, programm will be terminated!\n";
    return 1;
  }


  int imgNum = reader->readNext();


  ITMLib::ITMView * wholeView = new ITMLib::ITMView(*reader->getCalib(),imgSize,imgSize,false);
  wholeView->depth = reader->depth_img;
  wholeView->rgb = reader->rgb_img;

  //create tracking engine
  auto * trackingEngine = new ObjSLAM::ObjSLAMTrackingEngine(internalSettings, reader->getCalib(), imgSize);

  auto t_controller =trackingEngine->getTrackingController();

  //create mapping engine
  auto *mappingEngine =
      new ObjSLAM::ObjSLAMMappingEngine<ITMVoxel, ITMVoxelIndex>(internalSettings, reader->getCalib(), imgSize);

  mappingEngine->SetTrackingController(t_controller);
  
  
  
  mappingEngine->UpdateImgNumber(imgNum);
  auto objview = mappingEngine->CreateView(reader->depth_img, reader->rgb_img, reader->label_img_vector);

//  auto t_state = trackingEngine->TrackFrame(objview. ->getBackgroundView().get());
  auto t_state = trackingEngine->TrackFrame(wholeView);
  mappingEngine->UpdateTrackingState(t_state);

  mappingEngine->ProcessFrame();

  mappingEngine->outputAllObjImages();

  delete wholeView;



  int totFrames =atoi( argv[2]);
  while (imgNum<=totFrames) {
    std::clock_t start;
    double time;
    start = std::clock();
    imgNum = reader->readNext();

    wholeView = new ITMLib::ITMView(*reader->getCalib(),imgSize,imgSize,false);
    wholeView->depth = reader->depth_img;
    wholeView->rgb = reader->rgb_img;


    mappingEngine->UpdateImgNumber(imgNum);
//  cout << reader->getPose()->getSE3Pose().GetM();

    objview = mappingEngine->CreateView(reader->depth_img, reader->rgb_img, reader->label_img_vector);


    auto t_state = trackingEngine->TrackFrame(wholeView);

//    mappingEngine->UpdateTrackingState(&reader->getPose()->getSE3Pose());
    mappingEngine->UpdateTrackingState(t_state);

    mappingEngine->ProcessFrame();
    mappingEngine->outputAllObjImages();
    delete wholeView;
    time = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;

    cout<<"Img "<<imgNum<< " Time "<<time<<endl;
  }


  delete reader;
  return 0;
}