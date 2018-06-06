//
// Created by khuang on 6/6/18.
//

#ifndef MT_OBJSLAM_OBJECTINSTANCE_H
#define MT_OBJSLAM_OBJECTINSTANCE_H

#include <vector>
#include <iostream>

#include "ObjectClassLabel.h"
#include "ObjectView.h"git

namespace ObjSLAM {

class ObjectInstance {
 private:
  ObjectClassLabel label;
  std::vector<ObjectView> listOfVisibleViews;
  //private member of allocated voxels;

 public:

  //Constructor
  ObjectInstance(ObjectClassLabel _label, std::vector<ObjectView> _views):
      label(_label), listOfVisibleViews(_views){

    //debug
    std::cout<<"Created ObjectInstance of class "<<label.getLabelClassName()<<std::endl;
  }

  //getters
  ObjectClassLabel getClassLabel();


  std::vector<ObjectView> getListOfVisibleViews();


  //setters
  void setClassLabel(ObjectClassLabel _label);

//  void setListOfVisibleViews(std::vector<ObjectView> _views);

};

}
#endif //MT_OBJSLAM_OBJECTINSTANCE_H
