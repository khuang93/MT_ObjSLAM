//
// Created by khuang on 6/6/18.
//

#ifndef MT_OBJSLAM_OBJECTINSTANCE_H
#define MT_OBJSLAM_OBJECTINSTANCE_H

#include <vector>
#include <iostream>


#include "ObjectClassLabel.h"

//#include "ObjectView_old.h"

namespace ObjSLAM {

class ObjectInstance {
 private:
  ObjectClassLabel label;
//  std::vector<ObjectView_old> listOfVisibleViews;

  //TODO: complete the class with voxel blocks:
  //private member of allocated voxels;


 public:

  //Constructor
  ObjectInstance(ObjectClassLabel _label):
      label(_label){

    //debug
    std::cout<<"Created ObjectInstance of class "<<label.getLabelClassName()<<std::endl;
  }

  //getters

  ObjectClassLabel getClassLabel();


//  std::vector<ObjectView_old>  getListOfVisibleViews();


  //setters
  void setClassLabel(ObjectClassLabel _label);

//  void setListOfVisibleViews(std::vector<ObjectView_old> _views);

};

}
#endif //MT_OBJSLAM_OBJECTINSTANCE_H
