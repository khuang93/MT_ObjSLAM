//
// Created by khuang on 6/6/18.
//

#include "ObjSLAMEngine.h"

namespace ObjSLAM {

  int ObjSLAMEngine::init_InfiniTAMCreator() {
    return 10;
  }

  void ObjSLAMEngine::createPose(){
    pose = new ObjCameraPose;
  }

  void ObjSLAMEngine::createEmptyScene() {
    scene = new ObjectScene;
  }

}