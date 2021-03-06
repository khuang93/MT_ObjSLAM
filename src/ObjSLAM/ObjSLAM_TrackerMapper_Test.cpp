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

//#include <g2o/core/base_vertex.h>
//#include <g2o/core/base_unary_edge.h>
//#include <g2o/core/block_solver.h>
//#include <g2o/core/optimization_algorithm_levenberg.h>
//#include <g2o/core/optimization_algorithm_gauss_newton.h>
//#include <g2o/core/optimization_algorithm_dogleg.h>
//#include <g2o/solvers/dense/linear_solver_dense.h>
//
//#include <g2o/types/slam3d/types_slam3d.h>
//#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

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
bool do_BG_cleanup = true;
bool do_Obj_cleanup = true;
bool do_Obj_tracking = true;

/**
 * @brief main function of ObjSLAM
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv) {




    //TODO Debug output
    cout << "**Hello SLAM World!" << endl;

    //Path of the depth image file
    string path = argv[1];

    //Image Size 640 x 480
    Vector2i imgSize(640, 480);

    //Parse the program arguments
    totFrames = atoi(argv[2]);
    std::cout << "totFrames = " << totFrames << std::endl;

    if (argc > 3 && atoi(argv[3]) > 0) {
        reader_SkipFrames = atoi(argv[3]);
    }

    if (argc > 4) {
        saveSTL = (atoi(argv[4]) != 0);
        if (saveSTL) {
            STL_Frequency = atoi(argv[5]);
        }
        do_BG_cleanup = (atoi(argv[6]) != 0);
        do_Obj_cleanup = (atoi(argv[7]) != 0);
        do_Obj_tracking = (atoi(argv[8]) != 0);
    }
    std::cout << "BG Cleanup = " << do_BG_cleanup << std::endl;
    std::cout << "Obj Cleanup = " << do_Obj_cleanup << std::endl;
    std::cout << "Obj Tracking = " << do_Obj_tracking << std::endl;

    //Create UI
    ObjSLAM::ObjSLAMUI *ui = new ObjSLAM::ObjSLAMUI(imgSize);

    //Create Internal settings
    std::shared_ptr<ITMLib::ITMLibSettings> internalSettings = std::make_shared<ITMLib::ITMLibSettings>();
    internalSettings->sceneParams = ITMLib::ITMSceneParams(0.1f, 100, 0.01f, 0.1, 10.0,
                                                           true);// ITMLib::ITMSceneParams(0.08f, 100, 0.008f, 0.1, 10.0, true);

    std::shared_ptr<ITMLib::ITMLibSettings> internalSettings_obj = std::make_shared<ITMLib::ITMLibSettings>();
    //Scene Params
    //  float mu, int maxW, float voxelSize, float viewFrustum_min, float viewFrustum_max, bool stopIntegratingAtMaxW
    internalSettings_obj->sceneParams = ITMLib::ITMSceneParams(0.1f, 100, 0.01f, 0.1, 10.0, true);
    //(0.1, 10, 0.025, 0.1, 4.0, false); //(0.02f, 100, 0.002f, 0.2f, 3.0f, false);  //(0.2, 4, 0.05, 0.1, 4.0, false);
    //0.1f, 5, 0.01f, 0.1, 6.0, false  0.04f, 100, 0.005f, 0.2f, 5.0f, false

    //Set to use CPU
    internalSettings->deviceType = ITMLib::ITMLibSettings::DEVICE_CPU;

    DatasetReader *reader = nullptr;

    /**
     * @brief Select which reader should be used based on the name of the dataset folder
     */
    if (path.find("Teddy") != std::string::npos) {
        cout << "Teddy\n";
        reader = new TeddyReader(path, imgSize);
    } else if (path.find("RealisticRenderingDataset") != std::string::npos) {
        cout << "RealisticRenderingDataset\n";
        reader = new LPD_Dataset_Reader(path, imgSize);
    } else /*if(path.find("rgbd")!=std::string::npos || path.find("traj")!=std::string::npos)*/{
        cout << "TUM RGBD\n";
        reader = new TUM_Reader(path, imgSize);
    }/*else{
    cout<<"Dataset not supported, programm will be terminated!\n";
    return 1;
  }*/


    sceneIsBackground = true;

    /**
     * @brief Create the main engine
     */
    ObjSLAMMainEngine *mainEngine = new ObjSLAMMainEngine(internalSettings, internalSettings_obj,
                                                          std::shared_ptr<DatasetReader>(reader));

    ui->SetMainEngine(mainEngine);
    ui->Run();

/*

//TODO parallel tracking and mapping in two threads
#pragma omp parallel sections shared(mainEngine,imgNum)
{
  #pragma omp section
  {
      while (imgNum<=totFrames) {
          if(mainEngine->framesElapsedBeforeMapping<1){
              sceneIsBackground=true;
              imgNum = mainEngine->ReadNext();
//              cout<<"Section 1 imgNum = "<<imgNum;
              mainEngine->TrackFrame();
          }

      }
  }

  #pragma omp section
  {
      while (imgNum<=totFrames ) {
//        cout<<"Section 2 imgNum = "<<imgNum;
          if(mainEngine->mapperFree  ){
            cout<<"Section 2 mapperFree? "<<mainEngine->mapperFree;
            mainEngine->UpdateMappingEngine();
            mainEngine->MapFrame();
            mainEngine->OutputPics();
          }
      }
  }

}
*/

//delete newed pointers
    delete mainEngine;
    delete reader;
    delete ui;
//  delete trackingEngine;
//  delete mappingEngine;
    return 0;
}
