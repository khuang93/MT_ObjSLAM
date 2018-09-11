//
// Created by khuang on 8/29/18.
//

#ifndef MT_OBJSLAM_OBJSLAMVOXELBLOCKHASH_H
#define MT_OBJSLAM_OBJSLAMVOXELBLOCKHASH_H

#include <External/InfiniTAM/InfiniTAM/ITMLib/Objects/Scene/ITMVoxelBlockHash.h>


namespace ObjSLAM{

class ObjSLAMVoxelBlockHash : public ITMLib::ITMVoxelBlockHash{
  private:

  public:
  ObjSLAMVoxelBlockHash(bool isBackGround, MemoryDeviceType memoryDeviceType):ITMVoxelBlockHash(memoryDeviceType){

  }




};







}
#endif //MT_OBJSLAM_OBJSLAMVOXELBLOCKHASH_H
