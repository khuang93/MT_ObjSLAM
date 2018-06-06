//
// Created by khuang on 6/6/18.
//

#include "ObjectInstance.h"

namespace ObjSLAM{

  //getters
  ObjectClassLabel ObjectInstance::getClassLabel(){
    return label;
  }


   std::vector<ObjectView> *ObjectInstance::getListOfVisibleViews(){
    return listOfVisibleViews;
  }


  //setters
  void ObjectInstance::setClassLabel(ObjectClassLabel _label){
    label = _label;
    std::cout<<"Label set: "<<label.getLabelClassName()<<std::endl;
  }

//  void ObjectInstance::setListOfVisibleViews(std::vector<ObjectView> _views){
//    listOfVisibleViews = _views;
//  }


}