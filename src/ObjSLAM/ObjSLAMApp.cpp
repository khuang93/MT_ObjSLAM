//
// Created by khuang on 6/6/18.
//
#include <iostream>

#include "ObjSLAMEngine.h"
#include "ObjSLAMEngine.tpp"

#include "../../External/InfiniTAM/InfiniTAM/ITMLib/ITMLibDefines.h"
#include "../../External/InfiniTAM/InfiniTAM/InputSource/ImageSourceEngine.h"
#include "ObjSLAMMappingEngine.h"


///@brief Entry point of ObjSLAM
using namespace ObjSLAM;
using namespace ITMLib;
using namespace InputSource;


int main(int argc, char ** argv) {


  std::cout<< "ObjSLAMApp"<<std::endl;



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
////  auto *objSLAM = new ObjSLAMEngine<ITMVoxel, ITMVoxelIndex>(settings, calib, RGBSize, DSize);
//
//  std::cout<<"debug\n";
//
//  delete objSLAM;

  return 0;
}
