//
// Created by khuang on 6/6/18.
//

#ifndef MT_OBJSLAM_OBJSLAMENGINE_H
#define MT_OBJSLAM_OBJSLAMENGINE_H
#include<iostream>
#include "ObjCameraPose.h"
#include "ObjectScene.h"

namespace ObjSLAM {
class ObjSLAMEngine {
 private:
  int infiniTAMCreator;
  ObjCameraPose *pose;

  //TODO: scene< > using template args
  ObjectScene *scene;



 public:

  /**
   * Empty Constructor
   */
  ObjSLAMEngine() {
    std::cout<<"ObjSLAMEngine Constructor\n";
    createPose();
    createEmptyScene()

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

  void createEmptyScene();


};

}
#endif //MT_OBJSLAM_OBJSLAMENGINE_H
