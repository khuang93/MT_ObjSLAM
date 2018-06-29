//
// Created by khuang on 6/29/18.
//

#ifndef MT_OBJSLAM_OBJSLAMBASICENGINE_H
#define MT_OBJSLAM_OBJSLAMBASICENGINE_H

#include "External/InfiniTAM/InfiniTAM/ITMLib/Core/ITMBasicEngine.h"
#include "ObjSLAMDataTypes.h"

namespace ObjSLAM{
using namespace ITMLib;

//template <typename TVoxel, typename TIndex>
class ObjSLAMBasicEngine:public ITMBasicEngine{

// public:
//  ITMLib::ITMTrackingState::TrackingResult ProcessFrame(ITMUChar4Image *rgbImage, ObjFloatImage *depthImage, ITMLib::ITMIMUMeasurement *imuMeasurement = NULL);
//
//
 public:
  ObjSLAMBasicEngine(const ITMLib::ITMLibSettings *settings, const ITMLib::ITMRGBDCalib& calib, Vector2i imgSize_rgb, Vector2i imgSize_d = Vector2i(-1, -1)):ITMLib::ITMBasicEngine(settings,  calib,  imgSize_rgb,  imgSize_d){}


};
}
#endif //MT_OBJSLAM_OBJSLAMBASICENGINE_H
