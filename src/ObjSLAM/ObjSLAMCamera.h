//
// Created by khuang on 7/26/18.
//

#ifndef MT_OBJSLAM_OBJSLAMCAMERA_H
#define MT_OBJSLAM_OBJSLAMCAMERA_H

#include <External/InfiniTAM/InfiniTAM/ITMLib/Objects/Camera/ITMRGBDCalib.h>
#include "ObjSLAMDataTypes.h"

namespace ObjSLAM{
class ObjSLAMCamera {
 public:
  ITMLib::ITMRGBDCalib *calib;
  int width, height;
  Vector2i imgSize;


  ObjFloatImage projectPointCloud2Img(ORUtils::Image<Vector4f> PCL);

};
}
#endif //MT_OBJSLAM_OBJSLAMCAMERA_H
