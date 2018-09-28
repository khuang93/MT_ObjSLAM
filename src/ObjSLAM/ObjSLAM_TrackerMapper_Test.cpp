//
// Created by khuang on 6/11/18.
//
#include <iostream>
#include <External/InfiniTAM/InfiniTAM/ITMLib/Objects/RenderStates/ITMRenderStateFactory.h>
#include <src/ObjSLAM/ObjSLAMVoxelSceneParams.h>
#include "LPD_Dataset_Reader.h"
#include "ObjectInstanceScene.h"
#include "../../External/InfiniTAM/InfiniTAM/ITMLib/ITMLibDefines.h"
#include "ObjSLAMMappingEngine.h"
#include "ObjSLAMTrackingEngine.h"
#include "ObjectView.h"
#include "TeddyReader.h"
#include "TUM_Reader.h"
#include "ObjSLAMMainEngine.h"
#include "ObjSLAMUI.h"
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
#include <sys/time.h>



using namespace std;



//static global variables
bool saveSTL = false;
int STL_Frequency = 1;
int reader_SkipFrames = 0;
int numthreads = 4;
int totFrames;
bool sceneIsBackground = false;


int main(int argc, char **argv) {




  //TODO Debug output
  cout << "**Hello SLAM World!" << endl;

  //Path of the depth image file
  string path = argv[1];
  Vector2i imgSize(640, 480);

  totFrames =atoi( argv[2]);

  if(argc>3 && atoi(argv[3])>0){
    reader_SkipFrames = atoi(argv[3]);
  }

  if(argc>4){
    saveSTL = (atoi(argv[4])!=0);
    if(saveSTL){
      STL_Frequency = atoi(argv[5]);
    }
  }


    ObjSLAM::ObjSLAMUI* ui =new ObjSLAM::ObjSLAMUI(imgSize);




//  ITMLib::ITMLibSettings *internalSettings = new ITMLib::ITMLibSettings();
  std::shared_ptr<ITMLib::ITMLibSettings> internalSettings = std::make_shared<ITMLib::ITMLibSettings>();
  internalSettings->sceneParams = ITMLib::ITMSceneParams(0.1f, 100, 0.01f, 0.1, 10.0, true);
  //(0.1, 10, 0.025, 0.1, 4.0, false); //(0.02f, 100, 0.002f, 0.2f, 3.0f, false);  //(0.2, 4, 0.05, 0.1, 4.0, false);
          //0.1f, 5, 0.01f, 0.1, 6.0, false  0.04f, 100, 0.005f, 0.2f, 5.0f, false
//  float mu, int maxW, float voxelSize, float viewFrustum_min, float viewFrustum_max, bool stopIntegratingAtMaxW

  internalSettings->deviceType = ITMLib::ITMLibSettings::DEVICE_CPU;


//  LPD_Dataset_Reader reader(path, imgSize);
  DatasetReader* reader= nullptr;
  if(path.find("Teddy")!=std::string::npos){
    cout<<"Teddy\n";
    reader = new TeddyReader(path,imgSize);
  }else if(path.find("RealisticRenderingDataset")!=std::string::npos){
    cout<<"RealisticRenderingDataset\n";
    reader = new LPD_Dataset_Reader(path,imgSize);
  }else /*if(path.find("rgbd")!=std::string::npos || path.find("traj")!=std::string::npos)*/{
    cout<<"TUM RGBD\n";
    reader = new TUM_Reader(path,imgSize);
  }/*else{
    cout<<"Dataset not supported, programm will be terminated!\n";
    return 1;
  }*/


  sceneIsBackground=true;
  ObjSLAMMainEngine* mainEngine =new ObjSLAMMainEngine(internalSettings, std::shared_ptr<DatasetReader>(reader));

  ui->setMainEngine(mainEngine);
  ui->run();

  /*int imgNum = mainEngine->readNext();
  // if(imgNum==-1) return 0;
  mainEngine->trackFrame();
  mainEngine->updateMappingEngine();
  mainEngine->mapFrame();
  mainEngine->outputPics();

  while (imgNum<=totFrames) {
      sceneIsBackground=true;
      imgNum = mainEngine->readNext();
      if(imgNum==-1) return 0;
      mainEngine->trackFrame();
      mainEngine->updateMappingEngine();
      mainEngine->mapFrame();
      mainEngine->outputPics();

  }*/


/*

//TODO parallel tracking and mapping
#pragma omp parallel sections shared(mainEngine,imgNum)
{
  #pragma omp section
  {
      while (imgNum<=totFrames) {
          if(mainEngine->framesElapsedBeforeMapping<1){
              sceneIsBackground=true;
              imgNum = mainEngine->readNext();
//              cout<<"Section 1 imgNum = "<<imgNum;
              mainEngine->trackFrame();
          }

      }
  }

  #pragma omp section
  {
      while (imgNum<=totFrames ) {
//        cout<<"Section 2 imgNum = "<<imgNum;
          if(mainEngine->mapperFree  ){
            cout<<"Section 2 mapperFree? "<<mainEngine->mapperFree;
            mainEngine->updateMappingEngine();
            mainEngine->mapFrame();
            mainEngine->outputPics();
          }
      }
  }

}
*/






/*
  int imgNum = reader->readNext();


  shared_ptr<ITMLib::ITMView> wholeView = make_shared<ITMLib::ITMView>(*reader->getCalib(),imgSize,imgSize,false);
  wholeView->depth->SetFrom(reader->depth_img,ORUtils::Image<float>::CPU_TO_CPU);
  wholeView->rgb ->SetFrom(reader->rgb_img,ORUtils::Image<Vector4u>::CPU_TO_CPU);
  SaveImageToFile(wholeView->depth,"TEST");
  //create tracking engine
  sceneIsBackground = true;
  auto * trackingEngine = new ObjSLAM::ObjSLAMTrackingEngine(internalSettings, reader->getCalib(), imgSize);

  auto t_controller =trackingEngine->getTrackingController();

  //create mapping engine
  auto *mappingEngine =
      new ObjSLAM::ObjSLAMMappingEngine<ITMVoxel, ITMVoxelIndex>(internalSettings, reader->getCalib(), imgSize);

  mappingEngine->SetTrackingController(t_controller);



  mappingEngine->UpdateImgNumber(imgNum);


//  auto t_state = trackingEngine->TrackFrame(objview. ->getBackgroundView().get());
  cout<<sceneIsBackground<<endl;
  shared_ptr<ITMLib::ITMTrackingState>  t_state = trackingEngine->TrackFrame(wholeView.get());

  mappingEngine->UpdateTrackingState(t_state);

  mappingEngine->CreateView(reader->depth_img, reader->rgb_img, reader->label_img_vector);

  mappingEngine->ProcessFrame();

  mappingEngine->outputAllObjImages();



  bool trackingOnly = true;
  int KF_freq = 1;





  while (imgNum<=totFrames) {
    trackingOnly = (imgNum%KF_freq != 0);

    std::clock_t start;
    std::clock_t start_subtask;

    std::chrono::duration<double> wctduration;


    double time;
    start = std::clock();
    start_subtask=std::clock();
    auto wcts = std::chrono::system_clock::now();
    auto wcts_sub = std::chrono::system_clock::now();


    imgNum = reader->readNext();
    if(imgNum == -1) return 0;

    time = ( std::clock() - start_subtask ) / (double) CLOCKS_PER_SEC;
    cout<<"readNext "<<time<<endl;
    start_subtask=std::clock();
    wcts_sub=std::chrono::system_clock::now();

    sceneIsBackground = true;
    wholeView = make_shared<ITMLib::ITMView>(*reader->getCalib(),imgSize,imgSize,false);

    wholeView->depth->SetFrom(reader->depth_img,ORUtils::Image<float>::CPU_TO_CPU);
    wholeView->rgb ->SetFrom(reader->rgb_img,ORUtils::Image<Vector4u>::CPU_TO_CPU);


    mappingEngine->UpdateImgNumber(imgNum);

    cout<<sceneIsBackground<<endl;
    t_state = trackingEngine->TrackFrame(wholeView.get());
    time = ( std::clock() - start_subtask ) / (double) CLOCKS_PER_SEC;
    wctduration = (std::chrono::system_clock::now() - wcts_sub);



    cout<<"Tracker Res: "<<t_state.get()->trackerResult<<endl;
    if(t_state.get()->trackerResult!=ITMLib::ITMTrackingState::TRACKING_GOOD) {
      t_state->trackerResult=ITMLib::ITMTrackingState::TRACKING_GOOD;
    }

    mappingEngine->UpdateTrackingState(t_state);
      cout<<"TrackFrame "<<wctduration.count()<<endl;
      start_subtask=std::clock();
      wcts_sub=std::chrono::system_clock::now();
    if(trackingOnly) continue; //tracking only

    mappingEngine->CreateView(reader->depth_img, reader->rgb_img, reader->label_img_vector);

    time = ( std::clock() - start_subtask ) / (double) CLOCKS_PER_SEC;
    wctduration = (std::chrono::system_clock::now() - wcts_sub);
    cout<<"CreateView "<<wctduration.count()<<endl;
    start_subtask=std::clock();
    wcts_sub=std::chrono::system_clock::now();

    mappingEngine->ProcessFrame();

    time = ( std::clock() - start_subtask ) / (double) CLOCKS_PER_SEC;
    wctduration = (std::chrono::system_clock::now() - wcts_sub);
    cout<<"ProcessFrame "<<wctduration.count()<<endl;
    start_subtask=std::clock();
    wcts_sub=std::chrono::system_clock::now();

    mappingEngine->outputAllObjImages();

    time = ( std::clock() - start_subtask ) / (double) CLOCKS_PER_SEC;
    wctduration = (std::chrono::system_clock::now() - wcts_sub);
    cout<<"outputAllObjImages "<<wctduration.count()<<endl;

    time = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    wctduration = (std::chrono::system_clock::now() - wcts);

    cout<<"Img "<<imgNum<< " Time "<<wctduration.count()<<endl<<endl;

  }
*/

//  delete mainEngine;
  delete reader;
//  delete trackingEngine;
//  delete mappingEngine;
  return 0;
}
