//
// Created by khuang on 8/1/18.
//

#include "ObjectView_New.h"

#include "ObjectInstance_New.h"
#include "ObjSLAMCamera.h"

namespace ObjSLAM {

template<typename TVoxel, typename TIndex>
ObjCameraPose ObjectView_New<TVoxel, TIndex>::getCameraPose() {
  return *camera_Pose;
}

template<typename TVoxel, typename TIndex>
void ObjectView_New<TVoxel, TIndex>::setCameraPose(ObjCameraPose _pose) {
  camera_Pose = new ObjCameraPose(_pose.getSE3Pose());
}

template<typename TVoxel, typename TIndex>
void ObjectView_New<TVoxel, TIndex>::setCameraPose(const ORUtils::SE3Pose *_se3pose) {
  this->camera_Pose = new ObjCameraPose(*_se3pose);
}

//returns the new label if it is new. if the same class already exists, return the old label instance and discard the new one.
template<typename TVoxel, typename TIndex>
std::shared_ptr<ObjectClassLabel_Group<TVoxel, TIndex>> ObjectView_New<TVoxel, TIndex>::addLabelToVector(
    std::vector<shared_ptr<ObjectClassLabel_Group<TVoxel, TIndex>>> &label_ptr_vector,
    std::shared_ptr<ObjectClassLabel_Group<TVoxel, TIndex>> new_label) {
  if (label_ptr_vector.size() == 0) {
    label_ptr_vector.push_back(new_label);
//    cout<<new_label.get()->getLabelIndex();
    return new_label;
  } else {
    bool isNew = true;
    for (size_t t = 0; t < label_ptr_vector.size(); ++t) {
      if (label_ptr_vector.at(t)->getLabelIndex() == new_label->getLabelIndex()) {
        isNew = false;
        return label_ptr_vector.at(t);
//        break;
      }
    }
    if (isNew) {
      label_ptr_vector.push_back(new_label);
      return new_label;
//      cout<<new_label->getLabelIndex()<<endl;
    }
  }
}

template<typename TVoxel, typename TIndex>
void ObjectView_New<TVoxel, TIndex>::setListOfObjects(
    std::vector<shared_ptr<ObjectClassLabel_Group<TVoxel, TIndex>>> &label_ptr_vector) {

  cout << "Setting Obj List...\n";
//  std::vector<shared_ptr<ObjectClassLabel_Group<TVoxel,TIndex>>> label_ptr_vector;
  auto label_ptr_bg_new = std::make_shared<ObjectClassLabel_Group<TVoxel, TIndex>>(0, std::to_string(0));
  shared_ptr<ObjectClassLabel_Group<TVoxel, TIndex>> label_ptr_bg = addLabelToVector(label_ptr_vector,
                                                                                     label_ptr_bg_new);

  auto *cam= new ObjSLAMCamera(&calibration, imgSize_d);
  d_to_rgb_correspondence = cam->projectDepthPixelToRGB(this->depth_Image);
#ifdef WITH_OPENMP
#pragma omp parallel for
#endif
  for (LabelImgVec::iterator it = label_img_vector.begin(); it != label_img_vector.end(); ++it) {

    int labelIndex = 0;

    auto single_obj_ITMView = std::make_shared<ITMLib::ITMView>(calibration, imgSize_rgb, imgSize_d, false);

    //it over pixels
    for (int i = 0; i < (*it)->dataSize; ++i) {
      /* int y = i / imgSize_d.x;
       int x = i % imgSize_d.x;
       //TODO build a look up list between rgb and d instead of calculate it each time
       Vector4f pix_rgb ((float)x,(float)y,1.0f,1.0f);
       Matrix4f K_rgb_inv;
       cam->getK_rgb().inv(K_rgb_inv);
       Vector4f pix_d = cam->getK_d()*calibration.trafo_rgb_to_depth.calib*K_rgb_inv*pix_rgb;
 //      cout<<"pix_d"<<pix_d<<endl;
       int x_d = round(pix_d.x);
       int y_d = round(pix_d.y);
       int idx_d=0;
       if(x_d>=0 && y_d>=0){
         idx_d = y*imgSize_d.x+x;*/
//      int idx_d = this->rgb_d_pixel_idx_vec.at(i);
      Vector2i rgb_pixel_loc = d_to_rgb_correspondence->GetElement(i,MEMORYDEVICE_CPU);
      int idx_rgb = rgb_pixel_loc.y*imgSize_d.x+rgb_pixel_loc.x;

      if(rgb_pixel_loc.x==-1 || rgb_pixel_loc.y==-1){
          idx_rgb=-1;
      }

      if (idx_rgb != -1) {
        //if the label is not empty
        if ((*it)->GetElement(idx_rgb, MEMORYDEVICE_CPU) != 0) {
          //if label index == 0 it means it is the first labeled pixel, only one possible number beside the 0s in one label img
          if (labelIndex == 0) { labelIndex = (*it)->GetElement(idx_rgb, MEMORYDEVICE_CPU); }



          //Set value of the each pixel in the segmented itm view
          //TODO depth img is shifted, needed to shift the itm view the tracker is using

          single_obj_ITMView->depth->GetData(MEMORYDEVICE_CPU)[i] = this->depth_Image->GetData(
              MEMORYDEVICE_CPU)[i];
          single_obj_ITMView->rgb->GetData(MEMORYDEVICE_CPU)[i] = this->rgb_Image->GetData(
              MEMORYDEVICE_CPU)[idx_rgb];
        }
      }
//      }



    }

    //set all object instance map
    if (labelIndex != 0) {
      //label
      //create a new label
      auto label_ptr_new = std::make_shared<ObjectClassLabel_Group<TVoxel, TIndex>>(labelIndex,
                                                                                    std::to_string(
                                                                                        labelIndex));

      //returns the new label if it is new. if the same class already exists, return the old label instance and discard the new one.
      auto label_ptr = addLabelToVector(label_ptr_vector, label_ptr_new);
      
      //create a object instance
      auto new_obj_instance = std::make_shared<ObjectInstance_New<TVoxel, TIndex>>(label_ptr);

      new_obj_instance->setAnchorView(this->shared_from_this());
      new_obj_instance->setAnchorView_ITM(single_obj_ITMView);
//      new_obj_instance->addObjectInstanceToLabel();

      
      Object_View_Tup<TVoxel, TIndex> object_view_tuple(new_obj_instance, single_obj_ITMView);

//      obj_map.insert(std::pair<int, Object_View_Tup<TVoxel,TIndex>>(obj_map.size()+1, object_view_tuple));

      obj_vec.push_back(object_view_tuple);
    }
  }

  //background

  bg_itmview = make_shared<ITMLib::ITMView>(calibration, imgSize_rgb, imgSize_d, false);

  auto new_obj_instance = std::make_shared<ObjectInstance_New<TVoxel, TIndex>>(label_ptr_bg);

//        cout << "label" << new_obj_instance->getClassLabel()->getLabelIndex() << endl;

#ifdef WITH_OPENMP
#pragma omp parallel for
#endif

  //it over pixels
  for (int i = 0; i < this->depth_Image->dataSize; i++) {
//    int idx_d = this->rgb_d_pixel_idx_vec.at(i);
    Vector2i rgb_pixel_loc = d_to_rgb_correspondence->GetElement(i,MEMORYDEVICE_CPU);
    int idx_rgb = rgb_pixel_loc.y*imgSize_d.x+rgb_pixel_loc.x;

    if(rgb_pixel_loc.x==-1 || rgb_pixel_loc.y==-1){
      idx_rgb=-1;
    }

//    int idx_d = i;
    if (idx_rgb != -1) {

      bool is_background = true;
      for (LabelImgVec::iterator it = label_img_vector.begin(); it != label_img_vector.end(); it++) {

        if ((*it)->GetElement(idx_rgb, MEMORYDEVICE_CPU) != 0) {
          is_background = false;
          break;
        }
      }
      if (is_background) {
        bg_itmview->depth->GetData(MEMORYDEVICE_CPU)[i] = this->depth_Image->GetData(
            MEMORYDEVICE_CPU)[i];
        bg_itmview->rgb->GetData(MEMORYDEVICE_CPU)[i] = this->rgb_Image->GetData(MEMORYDEVICE_CPU)[idx_rgb];
      }
    }
  }

//TODO
//  new_obj_instance->setAnchorView(this->shared_from_this());
  new_obj_instance->setAnchorView_ITM(bg_itmview);

  Object_View_Tup<TVoxel, TIndex> object_view_tuple(new_obj_instance, bg_itmview);
//  obj_map.insert(std::pair<int, Object_View_Tup<TVoxel,TIndex>>(0, object_view_tuple));
  obj_vec.push_back(object_view_tuple);

//  SaveImageToFile(single_obj_ITMView_bg->depth,"test.ppm");
//  cout<<"size"<<this->obj_map.size()<<endl;

  //  std::cout << "FINISHED" << std::endl;
//  return label_ptr_vector;
//  delete cam;
}

template<typename TVoxel, typename TIndex>
void ObjectView_New<TVoxel, TIndex>::setListOfViews() {

  for (int i = 0; i < segmentation_Mask->dataSize; i++) {
//    if(segmentation_Mask->GetElement(i, MEMORYDEVICE_CPU))
  }

}

//template<typename TVoxel, typename TIndex>
//std::map<int, Object_View_Tup<TVoxel,TIndex>> ObjectView_New<TVoxel,TIndex>::getObjMap(){
//  return obj_map;
//}

template<typename TVoxel, typename TIndex>
std::vector<Object_View_Tup<TVoxel, TIndex>> ObjectView_New<TVoxel, TIndex>::getObjVec() {
  return obj_vec;
}

template<typename TVoxel, typename TIndex>
std::shared_ptr<ITMLib::ITMView> ObjectView_New<TVoxel, TIndex>::getBackgroundView() {
  if (bg_itmview.get() != NULL) {
    return this->bg_itmview;
  }

}

}