//
// Created by khuang on 6/6/18.
//

#ifndef MT_OBJSLAM_OBJCAMERAPOSE_H
#define MT_OBJSLAM_OBJCAMERAPOSE_H

#include <iostream>
namespace ObjSLAM{

//TODO: dummy class. Need to add the data type to represent pose in a form of a matrix
class ObjCameraPose {
 public:
  ObjCameraPose(){
    std::cout<<"ObjCameraPose created! \n";
  }

  ~ObjCameraPose(){
    std::cout<<"ObjCameraPose deleted! \n";
  }
};

}
#endif //MT_OBJSLAM_OBJCAMERAPOSE_H
