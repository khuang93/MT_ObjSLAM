//
// Created by khuang on 6/29/18.
//

#ifndef MT_OBJSLAM_OBJSLAMBASICENGINE_H
#define MT_OBJSLAM_OBJSLAMBASICENGINE_H

#include "External/InfiniTAM/InfiniTAM/ITMLib/Core/ITMBasicEngine.h"
#include "ObjSLAMDataTypes.h"
#include "External/InfiniTAM/InfiniTAM/ITMLib/Objects/Tracking/ITMTrackingState.h"

namespace ObjSLAM{
using namespace ITMLib;

template <typename TVoxel, typename TIndex>
class ObjSLAMBasicEngine : public  ITMBasicEngine<TVoxel,TIndex>{

 public:
  ITMLib::ITMTrackingState::TrackingResult ProcessFrame(ITMUChar4Image *rgbImage, ObjFloatImage *depthImage, ITMLib::ITMIMUMeasurement *imuMeasurement = NULL);



  ObjSLAMBasicEngine<TVoxel,TIndex>(const ITMLib::ITMLibSettings *settings, const ITMLib::ITMRGBDCalib& calib, Vector2i imgSize_rgb, Vector2i imgSize_d = Vector2i(-1, -1))
      :ITMBasicEngine<TVoxel,TIndex>(settings,  calib,  imgSize_rgb,  imgSize_d){}


};
}
#endif //MT_OBJSLAM_OBJSLAMBASICENGINE_H
