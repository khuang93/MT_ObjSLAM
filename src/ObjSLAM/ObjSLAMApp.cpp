//
// Created by khuang on 6/6/18.
//
#include <iostream>

#include "ObjSLAMEngine.h"
#include "ObjSLAMEngine.tpp"

#include "../../External/InfiniTAM/InfiniTAM/ITMLib/ITMLibDefines.h"
#include "../../External/InfiniTAM/InfiniTAM/InputSource/ImageSourceEngine.h"


//#include "../../External/InfiniTAM/InfiniTAM/InputSource/OpenNIEngine.h"
//#include "../../External/InfiniTAM/InfiniTAM/InputSource/Kinect2Engine.h"
//#include "../../External/InfiniTAM/InfiniTAM/InputSource/LibUVCEngine.h"
//#include "../../External/InfiniTAM/InfiniTAM/InputSource/PicoFlexxEngine.h"
//#include "../../External/InfiniTAM/InfiniTAM/InputSource/RealSenseEngine.h"
//#include "../../External/InfiniTAM/InfiniTAM/InputSource/LibUVCEngine.h"
//#include "../../External/InfiniTAM/InfiniTAM/InputSource/RealSense2Engine.h"
//#include "../../External/InfiniTAM/InfiniTAM/InputSource/FFMPEGReader.h"

///@brief Entry point of ObjSLAM
using namespace ObjSLAM;
using namespace ITMLib;
using namespace InputSource;


int main(int argc, char ** argv) {


  std::cout<< "ObjSLAMApp"<<std::endl;
  //Similar to InfiniTAM.cpp

/*
  //Start copied from InfiniTAM.cpp
  const char *arg1 = "";
  const char *arg2 = NULL;
  const char *arg3 = NULL;
  const char *arg4 = NULL;

  int arg = 1;
  do {
    if (argv[arg] != NULL) arg1 = argv[arg]; else break;
    ++arg;
    if (argv[arg] != NULL) arg2 = argv[arg]; else break;
    ++arg;
    if (argv[arg] != NULL) arg3 = argv[arg]; else break;
    ++arg;
    if (argv[arg] != NULL) arg4 = argv[arg]; else break;
  } while (false);

  if (arg == 1) {
    printf("usage: %s [<calibfile> [<imagesource>] ]\n"
           "  <calibfile>   : path to a file containing intrinsic calibration parameters\n"
           "  <imagesource> : either one argument to specify OpenNI device ID\n"
           "                  or two arguments specifying rgb and depth file masks\n"
           "\n"
           "examples:\n"
           "  %s ./Files/Teddy/calib.txt ./Files/Teddy/Frames/%%04i.ppm ./Files/Teddy/Frames/%%04i.pgm\n"
           "  %s ./Files/Teddy/calib.txt\n\n", argv[0], argv[0], argv[0]);
  }

  printf("initialising ...\n");

*/


  ITMLibSettings *settings = new ITMLibSettings();

//  std::cout<<"debug\n";
  ImageSourceEngine *imageSource = NULL;


  //Dummy
  ITMRGBDCalib calib = ITMRGBDCalib();

//  std::cout<<"debug\n";

  Vector2i RGBSize = Vector2i(100,100);
  Vector2i DSize = Vector2i(100,100);
  //end of copy

//  CreateDefaultImageSource(imageSource, /*imuSource,*/ arg1, arg2, arg3, arg4);


//  auto *objSLAM = new ObjSLAMEngine<ITMVoxel, ITMVoxelIndex>(settings, imageSource->getCalib(), imageSource->getRGBImageSize(), imageSource->getDepthImageSize());

//  std::cout<<"debug\n";
  //Dummy
  auto *objSLAM = new ObjSLAMEngine<ITMVoxel, ITMVoxelIndex>(settings, calib, RGBSize, DSize);

  std::cout<<"debug\n";

  delete objSLAM;

  return 0;
}
