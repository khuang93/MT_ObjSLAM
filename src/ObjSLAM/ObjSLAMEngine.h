//
// Created by khuang on 6/6/18.
//

#ifndef MT_OBJSLAM_OBJSLAMENGINE_H
#define MT_OBJSLAM_OBJSLAMENGINE_H
#include<iostream>
#include "ObjCameraPose.h"
#include "ObjectScene.h"

#include "../../External/InfiniTAM/InfiniTAM/ITMLib/Core/ITMBasicEngine.h"

namespace ObjSLAM {

template <class TVoxel, class TIndex>
class ObjSLAMEngine : ITMLib::ITMBasicEngine<TVoxel, TIndex> {

  private:
  int infiniTAMCreator;
//  ObjCameraPose *pose;

  //TODO: scene< > using template args

  ObjectScene<TVoxel, TIndex> *scene;



 public:

  /**
   * Empty Constructor
   */
  ObjSLAMEngine(const ITMLib::ITMLibSettings *settings, const ITMLib::ITMRGBDCalib& calib, Vector2i imgSize_rgb, Vector2i imgSize_d) :
      ITMLib::ITMBasicEngine<TVoxel,TIndex>(settings, calib, imgSize_rgb, imgSize_d){
    std::cout<<"ObjSLAMEngine Constructor\n";
    createPose();
    createEmptyScene();

  }

  /**
 * Empty Destructor
 */
//  ~ObjSLAMEngine(){
//    std::cout<<"ObjSLAMEngine Destructor\n";
////    delete pose;
//
//  }

  int init_InfiniTAMCreator();

  void createPose();

  void createEmptyScene();


};

}
#endif //MT_OBJSLAM_OBJSLAMENGINE_H
