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

/*static void CreateDefaultImageSource(ImageSourceEngine* & imageSource, *//*IMUSourceEngine* & imuSource,*//* const char *arg1, const char *arg2, const char *arg3, const char *arg4)
{
  const char *calibFile = arg1;
  const char *filename1 = arg2;
  const char *filename2 = arg3;
  const char *filename_imu = arg4;

//  if (strcmp(calibFile, "viewer") == 0)
//  {
//    imageSource = new BlankImageGenerator("", Vector2i(640, 480));
//    printf("starting in viewer mode: make sure to press n first to initiliase the views ... \n");
//    return;
//  }

  printf("using calibration file: %s\n", calibFile);

  if ((imageSource == NULL) && (filename2 != NULL))
  {
    printf("using rgb images: %s\nusing depth images: %s\n", filename1, filename2);
    if (filename_imu == NULL)
    {
      ImageMaskPathGenerator pathGenerator(filename1, filename2);
      imageSource = new ImageFileReader<ImageMaskPathGenerator>(calibFile, pathGenerator);
    }
    else
    {
      printf("using imu data: %s\n", filename_imu);
      imageSource = new RawFileReader(calibFile, filename1, filename2, Vector2i(320, 240), 0.5f);
//      imuSource = new IMUSourceEngine(filename_imu);
    }

    if (imageSource->getDepthImageSize().x == 0)
    {
      delete imageSource;
//      if (imuSource != NULL) delete imuSource;
//      imuSource = NULL;
      imageSource = NULL;
    }
  }

 *//* if ((imageSource == NULL) && (filename1 != NULL) && (filename_imu == NULL))
  {
    imageSource = new InputSource::FFMPEGReader(calibFile, filename1, filename2);
    if (imageSource->getDepthImageSize().x == 0)
    {
      delete imageSource;
      imageSource = NULL;
    }
  }

  if (imageSource == NULL)
  {
    // If no calibration file specified, use the factory default calibration
    bool useInternalCalibration = !calibFile || strlen(calibFile) == 0;

    printf("trying OpenNI device: %s - calibration: %s\n",
           filename1 ? filename1 : "<OpenNI default device>",
           useInternalCalibration ? "internal" : "from file");
    imageSource = new OpenNIEngine(calibFile, filename1, useInternalCalibration);
    if (imageSource->getDepthImageSize().x == 0)
    {
      delete imageSource;
      imageSource = NULL;
    }
  }

  if (imageSource == NULL)
  {
    printf("trying UVC device\n");
    imageSource = new LibUVCEngine(calibFile);
    if (imageSource->getDepthImageSize().x == 0)
    {
      delete imageSource;
      imageSource = NULL;
    }
  }

  if (imageSource == NULL)
  {
    printf("trying RealSense device\n");
    imageSource = new RealSenseEngine(calibFile);
    if (imageSource->getDepthImageSize().x == 0)
    {
      delete imageSource;
      imageSource = NULL;
    }
  }

  if (imageSource == NULL)
  {
    printf("trying RealSense device with SDK 2.X (librealsense2)\n");
    imageSource = new RealSense2Engine(calibFile);
    if (imageSource->getDepthImageSize().x == 0)
    {
      delete imageSource;
      imageSource = NULL;
    }
  }

  if (imageSource == NULL)
  {
    printf("trying MS Kinect 2 device\n");
    imageSource = new Kinect2Engine(calibFile);
    if (imageSource->getDepthImageSize().x == 0)
    {
      delete imageSource;
      imageSource = NULL;
    }
  }

  if (imageSource == NULL)
  {
    printf("trying PMD PicoFlexx device\n");
    imageSource = new PicoFlexxEngine(calibFile);
    if (imageSource->getDepthImageSize().x == 0)
    {
      delete imageSource;
      imageSource = NULL;
    }
  }*//*
}*/


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
