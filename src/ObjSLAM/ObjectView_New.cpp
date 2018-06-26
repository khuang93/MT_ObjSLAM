//
// Created by khuang on 6/25/18.
//

#include "ObjectView_New.h"

namespace ObjSLAM{

ObjCameraPose* ObjectView_New::getCameraPose(){
  return camera_Pose;
}


void ObjectView_New::setCameraPose(ObjCameraPose *_pose)
{
  camera_Pose = _pose;
}

void ObjectView_New::setListOfObjects(){

  std::cout<<"Setting Obj List..."<<std::endl;

  std::vector<int> labelIndexVector;

  for(int i=0; i  < segmentation_Mask->dataSize; i++){
    int labelIndex = segmentation_Mask->GetElement(i, MEMORYDEVICE_CPU);
//    std::cout<<"Get index "<<labelIndex<<std::endl;

    //if vector does not contain this label
    if(std::find(labelIndexVector.begin(), labelIndexVector.end(), labelIndex) == labelIndexVector.end()){
      labelIndexVector.push_back(labelIndex);
      std::cout<<"Added index "<<labelIndex<<std::endl;

      for(int j = i; j < segmentation_Mask->dataSize;j++){

      }

    }
  }
  for(int i = 0; i < labelIndexVector.size();i++){
    ObjectClassLabel newLabel (labelIndexVector.at(i),std::to_string(labelIndexVector.at(i)));
    ObjectInstance* newObjInstance= new ObjSLAM::ObjectInstance(newLabel);
    objectInstanceVector.push_back(*newObjInstance);

    auto* single_obj_ITMView = new ITMLib::ITMView(calibration,imgSize,imgSize, false);

    Object_View_Pair tmp_pair (newObjInstance,single_obj_ITMView );
    object_view_pair_vector.push_back(tmp_pair);

    obj_map.insert(std::pair<int, Object_View_Pair >(labelIndexVector.at(i),tmp_pair));

  }
}

void ObjectView_New::setListOfViews() {

  for(int i=0; i  < segmentation_Mask->dataSize; i++){
//    if(segmentation_Mask->GetElement(i, MEMORYDEVICE_CPU))
  }

}


std::map<int, Object_View_Pair> ObjectView_New::getObjMap(){
  return obj_map;
}


}