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
std::shared_ptr<ObjectClassLabel_Group<TVoxel,TIndex>> ObjectView_New<TVoxel,TIndex>::addLabelToVector(std::vector<shared_ptr<ObjectClassLabel_Group<TVoxel,TIndex>>>& label_ptr_vector, std::shared_ptr<ObjectClassLabel_Group<TVoxel,TIndex>> new_label){
  if(label_ptr_vector.size()==0) {
    label_ptr_vector.push_back(new_label);
//    cout<<new_label.get()->getLabelIndex();
    return new_label ;
  }else{
    bool isNew=true;
    for(size_t t=0; t<label_ptr_vector.size();++t){
      if(label_ptr_vector.at(t).get()->getLabelIndex()==new_label.get()->getLabelIndex()){
        isNew=false;
        return label_ptr_vector.at(t);
//        break;
      }
    }
    if(isNew){
      label_ptr_vector.push_back(new_label);
      return new_label;
//      cout<<new_label.get()->getLabelIndex()<<endl;
    }
  }
}

template<typename TVoxel, typename TIndex>
void ObjectView_New<TVoxel,TIndex>::setListOfObjects(std::vector<shared_ptr<ObjectClassLabel_Group<TVoxel,TIndex>>>& label_ptr_vector) {

//  std::cout << "Setting Obj List...";
//  std::vector<shared_ptr<ObjectClassLabel_Group<TVoxel,TIndex>>> label_ptr_vector;
  for(LabelImgVec::iterator it = label_img_vector.begin(); it!=label_img_vector.end();++it){

    int labelIndex = 0;

    auto single_obj_ITMView = std::make_shared<ITMLib::ITMView>(calibration, imgSize_rgb, imgSize_d, false);

    //it over pixels
    for (int i = 0; i < (*it)->dataSize; ++i) {
      //if the label is not empty
      if((*it)->GetElement(i, MEMORYDEVICE_CPU)!=0){
        //if label index == 0 it means it is the first labeled pixel, only one possible number beside the 0s in one label img
        if(labelIndex==0) {labelIndex = (*it)->GetElement(i, MEMORYDEVICE_CPU);}

        //Set value of the each pixel in the segmented itm view
        single_obj_ITMView->depth->GetData(MEMORYDEVICE_CPU)[i]=this->depth_Image->GetData(MEMORYDEVICE_CPU)[i];
        single_obj_ITMView->rgb->GetData(MEMORYDEVICE_CPU)[i]=this->rgb_Image->GetData(MEMORYDEVICE_CPU)[i];
//      cout<<(*it)->GetElement(i, MEMORYDEVICE_CPU)<<endl;
      }
    }
    //set all object instance map
    if(labelIndex!=0){
      //label


      //TODO find a way to check label existence before creating new ones
      //create a new label
      auto label_ptr_new = std::make_shared<ObjectClassLabel_Group<TVoxel,TIndex>>(labelIndex, std::to_string(labelIndex));

      //returns the new label if it is new. if the same class already exists, return the old label instance and discard the new one.
      auto label_ptr = addLabelToVector(label_ptr_vector,label_ptr_new);



      //TODO
      //is new object?
      //if yes:

      //create a object instance
      auto new_obj_instance = std::make_shared<ObjectInstance_New<TVoxel, TIndex>>(label_ptr);

      new_obj_instance.get()->setAnchorView(this->shared_from_this());
      new_obj_instance.get()->setAnchorView_ITM(single_obj_ITMView);

//      new_obj_instance.get()->addObjectInstanceToLabel();



      Object_View_Tup<TVoxel,TIndex> object_view_tuple(new_obj_instance, single_obj_ITMView);

      obj_map.insert(std::pair<int, Object_View_Tup<TVoxel,TIndex>>(obj_map.size()+1, object_view_tuple));

    }
  }


  //background
//  auto* single_obj_ITMView =  new ITMLib::ITMView(calibration, imgSize_rgb, imgSize_d, false);
  auto single_obj_ITMView = std::make_shared<ITMLib::ITMView>(calibration, imgSize_rgb, imgSize_d, false);

//  ObjectClassLabel_Group<TVoxel,TIndex> label_ptr(0, std::to_string(0));
  auto label_ptr = std::make_shared<ObjectClassLabel_Group<TVoxel,TIndex>>(0, std::to_string(0));

  auto new_obj_instance = std::make_shared<ObjectInstance_New<TVoxel, TIndex>>(label_ptr);
  new_obj_instance.get()->setAnchorView(this->shared_from_this());
  new_obj_instance.get()->setAnchorView_ITM(single_obj_ITMView);
//  new_obj_instance.get()->addObjectInstanceToLabel();


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
//  std::cout << "FINISHED" << std::endl;


//  return label_ptr_vector;
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