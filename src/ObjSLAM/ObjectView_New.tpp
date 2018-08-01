//
// Created by khuang on 8/1/18.
//

#include "ObjectView_New.h"

#include "ObjectInstance_New.h"

namespace ObjSLAM{

template<typename TVoxel, typename TIndex>
ObjCameraPose ObjectView_New<TVoxel,TIndex>::getCameraPose(){
  return camera_Pose;
}

template<typename TVoxel, typename TIndex>
void ObjectView_New<TVoxel,TIndex>::setCameraPose(ObjCameraPose _pose)
{
  camera_Pose = _pose;
}

template<typename TVoxel, typename TIndex>
void ObjectView_New<TVoxel,TIndex>::setListOfObjects() {
  std::cout << "Setting Obj List...";
  for(LabelImgVec::iterator it = label_img_vector.begin(); it!=label_img_vector.end();it++){

    int labelIndex = 0;
//    auto* single_obj_ITMView =  new ITMLib::ITMView(calibration, imgSize_rgb, imgSize_d, false);
    auto single_obj_ITMView = std::make_shared<ITMLib::ITMView>(calibration, imgSize_rgb, imgSize_d, false);

    //it over pixels
    for (int i = 0; i < (*it)->dataSize; i++) {

      if((*it)->GetElement(i, MEMORYDEVICE_CPU)!=0){
        if(labelIndex==0) {labelIndex = (*it)->GetElement(i, MEMORYDEVICE_CPU);}

        //Set value of the each pixel
        single_obj_ITMView->depth->GetData(MEMORYDEVICE_CPU)[i]=this->depth_Image->GetData(MEMORYDEVICE_CPU)[i];
        single_obj_ITMView->rgb->GetData(MEMORYDEVICE_CPU)[i]=this->rgb_Image->GetData(MEMORYDEVICE_CPU)[i];
//        cout<<(*it)->GetElement(i, MEMORYDEVICE_CPU)<<endl;
      }
    }
    //set all object instance map
    if(labelIndex!=0){
      //label
      ObjectClassLabel_Group<TVoxel,TIndex> label(labelIndex, std::to_string(labelIndex));

      //TODO
      //is new object?
      //if yes:

      //create a object instance
      auto new_obj_instance = std::make_shared<ObjectInstance_New<TVoxel, TIndex>>(label);
      Object_View_Tup<TVoxel,TIndex> object_view_tuple(new_obj_instance, single_obj_ITMView);

      obj_map.insert(std::pair<int, Object_View_Tup<TVoxel,TIndex>>(obj_map.size()+1, object_view_tuple));
    }
  }


  //background
//  auto* single_obj_ITMView =  new ITMLib::ITMView(calibration, imgSize_rgb, imgSize_d, false);
  auto single_obj_ITMView = std::make_shared<ITMLib::ITMView>(calibration, imgSize_rgb, imgSize_d, false);

  ObjectClassLabel_Group<TVoxel,TIndex> git (0, std::to_string(0));

  auto new_obj_instance = std::make_shared<ObjectInstance_New<TVoxel, TIndex>>(label);
  
  Object_View_Tup<TVoxel,TIndex> object_view_tuple(new_obj_instance, single_obj_ITMView);
  obj_map.insert(std::pair<int, Object_View_Tup<TVoxel,TIndex>>(0, object_view_tuple));

  //it over pixels
  for (int i = 0; i < this->depth_Image->dataSize; i++) {
    bool is_background = true;
    for(LabelImgVec::iterator it = label_img_vector.begin(); it!=label_img_vector.end();it++){

      if((*it)->GetElement(i,MEMORYDEVICE_CPU)!=0){
        is_background=false;
        break;
      }
    }
    if(is_background) {
      single_obj_ITMView->depth->GetData(MEMORYDEVICE_CPU)[i] = this->depth_Image->GetData(MEMORYDEVICE_CPU)[i];
      single_obj_ITMView->rgb->GetData(MEMORYDEVICE_CPU)[i] = this->rgb_Image->GetData(MEMORYDEVICE_CPU)[i];
    }
  }
  std::cout << "FINISHED" << std::endl;
}


template<typename TVoxel, typename TIndex>
void ObjectView_New<TVoxel,TIndex>::setListOfViews() {

  for(int i=0; i  < segmentation_Mask->dataSize; i++){
//    if(segmentation_Mask->GetElement(i, MEMORYDEVICE_CPU))
  }

}

template<typename TVoxel, typename TIndex>
std::map<int, Object_View_Tup<TVoxel,TIndex>> ObjectView_New<TVoxel,TIndex>::getObjMap(){
  return obj_map;
}


}