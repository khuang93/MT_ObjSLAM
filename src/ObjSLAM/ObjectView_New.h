//
// Created by khuang on 6/25/18.
//

#ifndef MT_OBJSLAM_OBJECTVIEW_NEW_H
#define MT_OBJSLAM_OBJECTVIEW_NEW_H


#include <vector>

#include "src/ObjSLAM/ObjSLAMDataTypes.h"

#include "External/InfiniTAM/InfiniTAM/ITMLib/Objects/Views/ITMView.h"
#include "External/InfiniTAM/InfiniTAM/ITMLib/Objects/Camera/ITMRGBDCalib.h"
#include "src/ObjSLAM/ObjCameraPose.h"
#include "ObjSLAMDataTypes.h"
#include "ObjectInstance.h"

#include <utility>
#include <map>

#include "External/InfiniTAM/InfiniTAM/ORUtils/MemoryBlock.h"
#include "External/InfiniTAM/InfiniTAM/ORUtils/Image.h"



namespace ObjSLAM {

using Object_View_Tuple = std::tuple<ObjectInstance*, ITMLib::ITMView*>;
using LabelImgVector = std::vector<ObjSLAM::ObjUIntImage *>;


class ObjectView_New {

 private:
  ObjCameraPose camera_Pose;

  ObjUIntImage *segmentation_Mask;
  LabelImgVector label_img_vector;
  const ObjUChar4Image *rgb_Image;
  const ObjFloatImage *depth_Image;
  ObjFloat4Image *depth_normal;

  const ITMLib::ITMRGBDCalib& calibration;
  Vector2i imgSize_rgb;
  Vector2i imgSize_d;


  std::vector<ObjectInstance> objectInstanceVector;
  std::vector<ITMLib::ITMView*> ITMViewVector_each_Object;


//  std::vector<Object_View_Tuple> object_view_pair_vector;

  std::map<int, Object_View_Tuple> obj_map; //int is the raw value in seg mask and tuple contains a obj instance and corresbonding ITMView

  void setListOfObjects_old();
  void setListOfObjects();
  void setListOfViews();


 public:

  //Constructor
  //using ITMLib::ITMView::ITMView;

  ObjectView_New(const ITMLib::ITMRGBDCalib& _calibration, Vector2i _imgSize, bool useGPU, ObjCameraPose pose):
      calibration(_calibration), imgSize_rgb(_imgSize), camera_Pose(pose){

    setListOfObjects_old();

    //TODO debug info
    std::cout<<"ObjectView_New simple created!\n";
  }

  //Constructor
  //using ITMLib::ITMView::ITMView;
  ObjectView_New(const ITMLib::ITMRGBDCalib& _calibration, Vector2i _imgSize_rgb, Vector2i _imgSize_d, bool useGPU, ObjCameraPose pose,
                 ObjFloatImage* _depth, ObjUChar4Image* _rgb, ObjUIntImage* _label):
      calibration(_calibration), imgSize_rgb(_imgSize_rgb), imgSize_d(_imgSize_d), camera_Pose(pose),
      depth_Image(_depth), rgb_Image(_rgb), segmentation_Mask(_label){

    setListOfObjects_old();
    //TODO debug info
    std::cout<<"ObjectView_New complete created!\n";

  }

  ObjectView_New(const ITMLib::ITMRGBDCalib& _calibration, Vector2i _imgSize_rgb, Vector2i _imgSize_d, bool useGPU, ObjCameraPose pose,
                 ObjFloatImage* _depth, ObjUChar4Image* _rgb, LabelImgVector _label_img_vector):
      calibration(_calibration), imgSize_rgb(_imgSize_rgb), imgSize_d(_imgSize_d), camera_Pose(pose),
      depth_Image(_depth), rgb_Image(_rgb), label_img_vector(_label_img_vector){

    setListOfObjects();
    //TODO debug info
    std::cout<<"ObjectView_New complete created!\n";

  }

  //Destructor
  ~ObjectView_New(){
//    delete camera_Pose;
    delete rgb_Image;
    delete depth_Image;
    delete segmentation_Mask;
//     ITMLib::ITMView::~ITMView();
  }


  std::map<int, Object_View_Tuple> getObjMap();

  ObjCameraPose getCameraPose();
  void setCameraPose(ObjCameraPose _pose);




  ObjectView_New(const ObjectView_New&);
  ObjectView_New& operator=(const ObjectView_New&);
};

}
#endif //MT_OBJSLAM_OBJECTVIEW_NEW_H
