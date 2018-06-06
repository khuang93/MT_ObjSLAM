//
// Created by khuang on 6/6/18.
//

#ifndef MT_OBJSLAM_OBJECTSLAMTRACKER_H
#define MT_OBJSLAM_OBJECTSLAMTRACKER_H
#include "../../External/InfiniTAM/InfiniTAM/ITMLib/Trackers/CPU/ITMExtendedTracker_CPU.h"
#include "../../External/InfiniTAM/InfiniTAM/ITMLib/Trackers/Interface/ITMExtendedTracker.h"
#include "../../External/InfiniTAM/InfiniTAM/ITMLib/Engines/LowLevel/Interface/ITMLowLevelEngine.h"

//#include "../../External/InfiniTAM/InfiniTAM/ITMLib/Trackers/CPU/ITMExtendedTracker_CPU.h"
namespace ObjSLAM {

  class ObjectSLAMTracker : public ITMLib::ITMExtendedTracker_CPU{

    //Constructor Inherited from parent
    using ITMLib::ITMExtendedTracker_CPU::ITMExtendedTracker_CPU;
   private:

  /* public:
    ObjectSLAMTracker(Vector2i imgSize_d,
                      Vector2i imgSize_rgb,
                      bool useDepth,
                      bool useColour,
                      float colourWeight,
                      ITMLib::TrackerIterationType *trackingRegime,
                      int noHierarchyLevels,
                      float terminationThreshold,
                      float failureDetectorThreshold,
                      float viewFrustum_min,
                      float viewFrustum_max,
                      float minColourGradient,
                      float tukeyCutOff,
                      int framesToSkip,
                      int framesToWeight,
                      const ITMLib::ITMLowLevelEngine *lowLevelEngine,
                      *//*MemoryDeviceType memoryType*//*) : ITMExtendedTracker_CPU(imgSize_d,
                                                                      imgSize_rgb,
                                                                      useDepth,
                                                                      useColour,
                                                                      colourWeight,
                                                                      *trackingRegime,
                                                                      noHierarchyLevels,
                                                                      terminationThreshold,
                                                                      failureDetectorThreshold,
                                                                      viewFrustum_min,
                                                                      viewFrustum_max,
                                                                      minColourGradient,
                                                                      tukeyCutOff,
                                                                      framesToSkip,
                                                                      framesToWeight,
                                                                      *lowLevelEngine//,
                                                                      *//*memoryType*//*)*/
   public:


   ~ObjectSLAMTracker(){

   }
  };

}
#endif //MT_OBJSLAM_OBJECTSLAMTRACKER_H
