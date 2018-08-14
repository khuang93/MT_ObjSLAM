//
// Created by khuang on 8/1/18.
//

#ifndef MT_OBJSLAM_OBJECTVIEW_NEW_H
#define MT_OBJSLAM_OBJECTVIEW_NEW_H

#include <vector>
#include <memory>

#include "src/ObjSLAM/ObjSLAMDataTypes.h"

#include "External/InfiniTAM/InfiniTAM/ITMLib/Objects/Views/ITMView.h"
#include "External/InfiniTAM/InfiniTAM/ITMLib/Objects/Camera/ITMRGBDCalib.h"
#include "src/ObjSLAM/ObjCameraPose.h"
#include "ObjSLAMDataTypes.h"


#include <utility>
#include <map>

#include "External/InfiniTAM/InfiniTAM/ORUtils/MemoryBlock.h"
#include "External/InfiniTAM/InfiniTAM/ORUtils/Image.h"


namespace ObjSLAM{
template<typename TVoxel, typename TIndex>
class ObjectInstance_New;

template<typename TVoxel, typename TIndex>
class ObjectClassLabel_Group;


template<typename TVoxel, typename TIndex>
using Object_View_Tup = std::tuple<std::shared_ptr<ObjectInstance_New<TVoxel,TIndex>>, std::shared_ptr<ITMLib::ITMView>>;

using LabelImgVec = std::vector<std::shared_ptr<ObjSLAM::ObjUIntImage>>;

template<typename TVoxel, typename TIndex>
class ObjectView_New :public enable_shared_from_this<ObjectView_New<TVoxel,TIndex>> {
 private:
  ObjCameraPose camera_Pose;

  std::shared_ptr<ObjUIntImage> segmentation_Mask;
  LabelImgVec label_img_vector;
  const ObjUChar4Image *rgb_Image;
  const ObjFloatImage *depth_Image;
  ObjFloat4Image *depth_normal;

  const ITMLib::ITMRGBDCalib& calibration;
  Vector2i imgSize_rgb;
  Vector2i imgSize_d;


  std::vector<Object_View_Tup<TVoxel,TIndex>> obj_vec;
  std::vector<ITMLib::ITMView*> ITMViewVector_each_Object;


  std::map<int, Object_View_Tup<TVoxel,TIndex>> obj_map; //int is the raw value in seg mask and tuple contains a obj instance and corresbonding ITMView



  void setListOfViews();


 public:

  //Constructor
  //using ITMLib::ITMView::ITMView;

  ObjectView_New(const ITMLib::ITMRGBDCalib& _calibration, Vector2i _imgSize, bool useGPU, ObjCameraPose pose):
      calibration(_calibration), imgSize_rgb(_imgSize), camera_Pose(pose){

    //TODO debug info
    std::cout<<"ObjectView simple created!\n";
  }

  ObjectView_New(const ITMLib::ITMRGBDCalib& _calibration, Vector2i _imgSize_rgb, Vector2i _imgSize_d, bool useGPU, ObjCameraPose pose,
             ObjFloatImage* _depth, ObjUChar4Image* _rgb, LabelImgVec _label_img_vector):
      calibration(_calibration), imgSize_rgb(_imgSize_rgb), imgSize_d(_imgSize_d), camera_Pose(pose),
      depth_Image(_depth), rgb_Image(_rgb), label_img_vector(_label_img_vector){


    //TODO debug info
//    std::cout<<"ObjectView complete created!\n";

  }
  //Destructor
  ~ObjectView_New(){
//    delete camera_Pose;
    delete rgb_Image;
    delete depth_Image;
//    delete segmentation_Mask;
    if(depth_normal!=NULL){
      delete depth_normal;
    }
  }

void setListOfObjects(std::vector<shared_ptr<ObjectClassLabel_Group<TVoxel,TIndex>>>& label_ptr_vector);

  std::map<int, Object_View_Tup<TVoxel,TIndex>> getObjMap();
  std::map<int, Object_View_Tup<TVoxel,TIndex>> getObjVec();

  static std::shared_ptr<ObjectClassLabel_Group<TVoxel,TIndex>> addLabelToVector(std::vector<shared_ptr<ObjectClassLabel_Group<TVoxel,TIndex>>>& label_ptr_vector, std::shared_ptr<ObjectClassLabel_Group<TVoxel,TIndex>> new_label);

  ObjCameraPose getCameraPose();
  void setCameraPose(ObjCameraPose _pose);




  ObjectView_New(const ObjectView_New&);
  ObjectView_New& operator=(const ObjectView_New&);
};

}

#include "ObjectView_New.tpp"
#endif //MT_OBJSLAM_OBJECTVIEW_NEW_H
