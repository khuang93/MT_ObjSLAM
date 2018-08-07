//
// Created by khuang on 7/26/18.
//

#ifndef MT_OBJSLAM_OBJSLAMCAMERA_H
#define MT_OBJSLAM_OBJSLAMCAMERA_H

#include <External/InfiniTAM/InfiniTAM/ITMLib/Objects/Camera/ITMRGBDCalib.h>
#include <External/InfiniTAM/InfiniTAM/ORUtils/SE3Pose.h>
#include "ObjSLAMDataTypes.h"
#include "External/InfiniTAM/InfiniTAM/ORUtils/Image.h"
#include "ObjCameraPose.h"

namespace ObjSLAM{
class ObjSLAMCamera {
 private:
  const ITMLib::ITMRGBDCalib *calib;
  int width, height;
  Vector2i imgSize;

 public:
  //constructors
  ObjSLAMCamera(const ITMLib::ITMRGBDCalib* _calib, Vector2i _imgSize):calib(_calib),imgSize(_imgSize){
    width=imgSize.width;
    height=imgSize.height;
  }

  ObjSLAMCamera(ITMLib::ITMRGBDCalib* _calib, int w, int h):calib(_calib),imgSize(w, h), width(w), height(h){}

 public:
  bool projectPointCloud2Img(ORUtils::Image<Vector4f> * PCL, ObjFloatImage* out, ObjCameraPose pose);

  ORUtils::Vector6<float> projectImg2PointCloud(ObjFloatImage* in, ORUtils::Image<Vector4f> * PCL,  ObjCameraPose pose);

  //getters
  const  ITMLib::ITMRGBDCalib * GetCalib(){ return calib;}
  Vector2i GetImgSize(){return imgSize;}
  int GetWidth(){return width;}
  int GetHeight(){ return height;}
};
}
#endif //MT_OBJSLAM_OBJSLAMCAMERA_H
