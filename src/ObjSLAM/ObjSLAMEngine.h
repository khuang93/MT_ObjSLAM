//
// Created by khuang on 6/6/18.
//

#ifndef MT_OBJSLAM_OBJSLAMENGINE_H
#define MT_OBJSLAM_OBJSLAMENGINE_H
#include<iostream>
#include "ObjCameraPose.h"

namespace ObjSLAM {
class ObjSLAMEngine {
 private:
  int infiniTAMCreator;
  ObjCameraPose *pose;
 public:

  /**
   * Empty Constructor
   */
  ObjSLAMEngine() {
    std::cout<<"ObjSLAMEngine Constructor\n";
    createPose();

  }

  /**
 * Empty Destructor
 */
  ~ObjSLAMEngine() {
    std::cout<<"ObjSLAMEngine Destructor\n";
    delete pose;

  }

  int init_InfiniTAMCreator();

  void createPose();
};

}
#endif //MT_OBJSLAM_OBJSLAMENGINE_H
