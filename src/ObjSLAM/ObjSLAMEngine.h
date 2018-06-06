//
// Created by khuang on 6/6/18.
//

#ifndef MT_OBJSLAM_OBJSLAMENGINE_H
#define MT_OBJSLAM_OBJSLAMENGINE_H
#include<iostream>

namespace ObjSLAM {
class ObjSLAMEngine {
  int infiniTAMCreator;

 public:

  /**
   * Empty Constructor
   */
  ObjSLAMEngine() {
    std::cout<<"ObjSLAMEngine Constructor\n";

  }

  /**
 * Empty Destructor
 */
  ~ObjSLAMEngine() {
    std::cout<<"ObjSLAMEngine Destructor\n";

  }

  int init_InfiniTAMCreator();
};

}
#endif //MT_OBJSLAM_OBJSLAMENGINE_H
