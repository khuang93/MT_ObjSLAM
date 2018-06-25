//
// Created by khuang on 6/6/18.
//

#include "ObjectView_old.h"
#include "External/InfiniTAM/InfiniTAM/ORUtils/MemoryBlock.h"
#include "External/InfiniTAM/InfiniTAM/ORUtils/Image.h"
#include <algorithm>
//#include "src/ObjSLAM/ObjCameraPose.h"

namespace ObjSLAM{

ObjCameraPose* ObjectView_old::getCameraPose(){
  return camera_Pose;
}


void ObjectView_old::setCameraPose(ObjCameraPose *_pose)
{
camera_Pose = _pose;
}

void ObjectView_old::setListOfObjects(){

  std::cout<<"Setting Obj List..."<<std::endl;

  std::vector<int> labelIndexVector;

  for(int i=0; i  < segmentation_Mask->dataSize; i++){
    int labelIndex = segmentation_Mask->GetElement(i, MEMORYDEVICE_CPU);
//    std::cout<<"Get index "<<labelIndex<<std::endl;
    //if vector does not contain this label
    if(std::find(labelIndexVector.begin(), labelIndexVector.end(), labelIndex) == labelIndexVector.end()){
      labelIndexVector.push_back(labelIndex);
      std::cout<<"Added index "<<labelIndex<<std::endl;
    }
  }
  for(int i = 0; i < labelIndexVector.size();i++){
    ObjectClassLabel newLabel (labelIndexVector.at(i),std::to_string(labelIndexVector.at(i)));
    ObjectInstance* newObjInstance= new ObjSLAM::ObjectInstance(newLabel);
    objectInstanceVector.push_back(*newObjInstance);
  }
}

}